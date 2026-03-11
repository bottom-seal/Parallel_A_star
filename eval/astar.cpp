// src/astar.cpp  (HDA*-style with: spatial ownership + sched_yield wait + batched sends + pending counter)
//
// DROP-IN VERSION:
//   - buffered outboxes + FLUSH_THRESHOLD
//   - global node arrays: g/f/parent/st indexed by idx
//   - aligned Mailbox/ThreadData + padded atomics
//   - when open empty: flush then sched_yield()
//   - IMPORTANT FIX: run_astar signature matches layout (SoA/AoS) to satisfy linker

#include "helper.h"

#if defined(USE_AOS)
    #include "layout_aos.h"
#elif defined(USE_SOA)
    #include "layout_soa.h"
#else
    #error "Must define either USE_AOS or USE_SOA"
#endif

#include <queue>
#include <vector>
#include <atomic>
#include <pthread.h>
#include <limits>
#include <cstdint>
#include <memory>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <sched.h>   // sched_yield
#include <iostream>

#include "astar_interface.h"

// Pick the exact state type so the exported symbol matches what main.cpp expects.
#if defined(USE_SOA)
using AStarStateT = AStarStateSoA;
#elif defined(USE_AOS)
using AStarStateT = AStarStateAoS;
#else
using AStarStateT = AStarState;
#endif

// --------------------------- Tunables ----------------------------------------
#ifndef TILE_SIZE
#define TILE_SIZE 64
#endif

#ifndef FLUSH_THRESHOLD
#define FLUSH_THRESHOLD 128
#endif

#ifndef WAIT_USEC
#define WAIT_USEC 200
#endif

// -----------------------------------------------------------------------------
// Open list (per-thread)
// -----------------------------------------------------------------------------
struct OpenEntry {
    int   idx;
    float f;
};
struct CompareF {
    bool operator()(const OpenEntry &a, const OpenEntry &b) const {
        return a.f > b.f; // min-heap
    }
};
using OpenSet = std::priority_queue<OpenEntry, std::vector<OpenEntry>, CompareF>;

// -----------------------------------------------------------------------------
// Heuristic
// -----------------------------------------------------------------------------
static inline float heuristic_manhattan(const GridMap &map, int idx) {
    int W = map.width;
    int x = idx % W;
    int y = idx / W;
    int dx = x - map.goal_x;
    int dy = y - map.goal_y;
    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;
    return float(dx + dy);
}

std::vector<float> precompute_heuristic_manhattan(const GridMap &map) {
    int N = map.width * map.height;
    std::vector<float> h(N);
    for (int idx = 0; idx < N; ++idx) h[idx] = heuristic_manhattan(map, idx);
    return h;
}

// -----------------------------------------------------------------------------
// Spatial ownership by tiles
// -----------------------------------------------------------------------------
static inline int owner_tile(const GridMap &map, int idx, int P) {
    int W = map.width;
    int x = idx % W;
    int y = idx / W;

    int tx = x / TILE_SIZE;
    int ty = y / TILE_SIZE;

    int tiles_x = (map.width  + TILE_SIZE - 1) / TILE_SIZE;
    int tile_id = ty * tiles_x + tx;

    return tile_id % P;
}

// -----------------------------------------------------------------------------
// idx -> owner
// -----------------------------------------------------------------------------
struct OwnerMap {
    int P = 1;
    int N = 0;
    std::vector<uint16_t> owner; // owner[idx]
};

static OwnerMap build_owner_map(const GridMap &map, int P) {
    OwnerMap m;
    m.P = P;
    m.N = map.width * map.height;
    int N = m.N;
    m.owner.resize(N);
    for (int idx = 0; idx < N; ++idx) {
        m.owner[idx] = (uint16_t)owner_tile(map, idx, P);
    }
    return m;
}

// -----------------------------------------------------------------------------
// Messaging
// -----------------------------------------------------------------------------
struct Message {
    int   idx;
    float g;
    int   parent;
};

// Mailbox: pending counter so receivers can avoid locking if empty.
struct alignas(64) Mailbox {
    pthread_mutex_t mtx;
    pthread_cond_t  cv;
    std::vector<Message> q;
    std::atomic<uint32_t> pending{0};

    Mailbox() {
        pthread_mutex_init(&mtx, nullptr);
        pthread_cond_init(&cv, nullptr);
        q.reserve(4096);
    }
    ~Mailbox() {
        pthread_cond_destroy(&cv);
        pthread_mutex_destroy(&mtx);
    }
};

struct alignas(64) ThreadData {
    OpenSet open;
    std::vector<std::vector<Message>> outboxes;
};

struct alignas(64) PaddedAtomicBool {
    std::atomic<bool> v{false};
    char pad[64 - sizeof(std::atomic<bool>)];
};
struct alignas(64) PaddedAtomicFloat {
    std::atomic<float> v;
    char pad[64 - sizeof(std::atomic<float>)];
};

// -----------------------------------------------------------------------------
// Context (global node arrays)
// -----------------------------------------------------------------------------
struct AStarContext {
    const GridMap            *map = nullptr;
    const std::vector<float> *h   = nullptr;

    int P = 1;
    int start_idx = -1;
    int goal_idx  = -1;

    OwnerMap owners;

    std::vector<float>   g;
    std::vector<float>   f;
    std::vector<int32_t> parent;
    std::vector<uint8_t> st;   // 0=unvisited, 1=open, 2=closed

    std::vector<std::unique_ptr<ThreadData>> tdata;
    std::vector<std::unique_ptr<Mailbox>> inbox;

    PaddedAtomicBool  goal_found;
    PaddedAtomicFloat best_goal_g;
};

struct WorkerArgs {
    AStarContext *ctx;
    int tid;
};

static inline float best_goal(const AStarContext *ctx) {
    return ctx->best_goal_g.v.load(std::memory_order_relaxed);
}

// Flush buf -> dest mailbox, with pending counter + signal-on-transition.
static inline void flush_outbox_to(AStarContext *ctx, int dest, std::vector<Message> &buf) {
    if (buf.empty()) return;

    Mailbox &mb = *ctx->inbox[dest];
    uint32_t n   = (uint32_t)buf.size();

    uint32_t old = mb.pending.fetch_add(n, std::memory_order_release);

    pthread_mutex_lock(&mb.mtx);
    mb.q.insert(mb.q.end(), buf.begin(), buf.end());
    pthread_mutex_unlock(&mb.mtx);

    if (old == 0) pthread_cond_signal(&mb.cv);

    buf.clear();
}

// Wake all sleeping threads (goal found)
static inline void wake_all(AStarContext *ctx) {
    for (int t = 0; t < ctx->P; ++t) {
        Mailbox &mb = *ctx->inbox[t];
        pthread_mutex_lock(&mb.mtx);
        pthread_cond_broadcast(&mb.cv);
        pthread_mutex_unlock(&mb.mtx);
    }
}

// -----------------------------------------------------------------------------
// Worker
// -----------------------------------------------------------------------------
static void* astar_worker(void *arg) {
    WorkerArgs *wa = static_cast<WorkerArgs*>(arg);
    AStarContext *ctx = wa->ctx;
    int tid = wa->tid;

    const GridMap &map = *ctx->map;
    const std::vector<float> &h = *ctx->h;

    ThreadData &td = *ctx->tdata[tid];
    OpenSet &open = td.open;

    const int P = ctx->P;
    const int goal = ctx->goal_idx;
    const int W = map.width;

    const int dx[4] = { 1, -1,  0,  0 };
    const int dy[4] = { 0,  0,  1, -1 };

    td.outboxes.clear();
    td.outboxes.resize(P);
    for (int d = 0; d < P; ++d) td.outboxes[d].reserve(1024);

    std::vector<Message> msgs;
    msgs.reserve(4096);

    Mailbox &my_mb = *ctx->inbox[tid];

    while (true) {
        if (ctx->goal_found.v.load(std::memory_order_relaxed)) break;

        // 1) Drain inbox only if pending > 0
        msgs.clear();
        uint32_t pend = my_mb.pending.load(std::memory_order_acquire);
        if (pend != 0) {
            pthread_mutex_lock(&my_mb.mtx);
            msgs.swap(my_mb.q);
            pthread_mutex_unlock(&my_mb.mtx);

            if (!msgs.empty()) {
                my_mb.pending.fetch_sub((uint32_t)msgs.size(), std::memory_order_acq_rel);
            }
        }

        // 1.1) Apply messages
        for (const auto &msg : msgs) {
            int idx = msg.idx;

#ifndef NDEBUG
            if (ctx->owners.owner[idx] != (uint16_t)tid) {
                fprintf(stderr, "Ownership violation: tid=%d idx=%d owner=%u\n",
                        tid, idx, (unsigned)ctx->owners.owner[idx]);
                std::abort();
            }
#endif

            float new_g = msg.g;
            float fval = new_g + h[idx];

            if (fval >= best_goal(ctx)) continue;
            if (ctx->st[idx] == 2) continue;

            if (new_g < ctx->g[idx]) {
                ctx->g[idx] = new_g;
                ctx->f[idx] = fval;
                ctx->parent[idx] = msg.parent;
                ctx->st[idx] = 1;
                open.push(OpenEntry{idx, fval});
            }
        }

        // 2) Reverted behavior: flush then yield
        if (open.empty()) {
            for (int dest = 0; dest < P; ++dest) {
                if (!td.outboxes[dest].empty()) {
                    flush_outbox_to(ctx, dest, td.outboxes[dest]);
                }
            }
            if (ctx->goal_found.v.load(std::memory_order_relaxed)) break;
            sched_yield();
            continue;
        }

        // 3) Pop best
        OpenEntry cur = open.top();
        open.pop();
        int n = cur.idx;

#ifndef NDEBUG
        if (ctx->owners.owner[n] != (uint16_t)tid) {
            fprintf(stderr, "Open contains foreign node: tid=%d n=%d owner=%u\n",
                    tid, n, (unsigned)ctx->owners.owner[n]);
            std::abort();
        }
#endif

        if (ctx->st[n] == 2) continue; // stale

        // 4) Goal test
        if (n == goal) {
            ctx->st[n] = 2;
            float goal_g = ctx->g[n];
            ctx->best_goal_g.v.store(goal_g, std::memory_order_relaxed);
            ctx->goal_found.v.store(true, std::memory_order_relaxed);
            wake_all(ctx);
            break;
        }

        // 5) Close + expand
        ctx->st[n] = 2;
        float g_n = ctx->g[n];

        int x = n % W;
        int y = n / W;

        for (int k = 0; k < 4; ++k) {
            int nx = x + dx[k];
            int ny = y + dy[k];
            if (!map.in_bounds(nx, ny) || map.is_wall(nx, ny)) continue;

            int m = ny * W + nx;
            float new_g = g_n + 1.0f;
            float f_m = new_g + h[m];

            if (f_m >= best_goal(ctx)) continue;

            int owner = ctx->owners.owner[m];

            if (owner == tid) {
                if (new_g < ctx->g[m]) {
                    ctx->g[m] = new_g;
                    ctx->f[m] = f_m;
                    ctx->parent[m] = n;
                    ctx->st[m] = 1;
                    open.push(OpenEntry{m, f_m});
                }
            } else {
                auto &buf = td.outboxes[owner];
                buf.push_back(Message{m, new_g, n});
                if ((int)buf.size() >= FLUSH_THRESHOLD) {
                    flush_outbox_to(ctx, owner, buf);
                }
            }
        }
    }

    for (int dest = 0; dest < ctx->P; ++dest) {
        if (!td.outboxes[dest].empty()) {
            flush_outbox_to(ctx, dest, td.outboxes[dest]);
        }
    }

    return nullptr;
}

// -----------------------------------------------------------------------------
// Public API (SIGNATURE FIXED HERE)
// -----------------------------------------------------------------------------
bool run_astar(const GridMap &map,
               AStarStateT &state,
               const std::vector<float> &h,
               int num_threads,
               AStarTimings *out_tm)
{
    const int N = map.width * map.height;
    const int start = map.start_idx;
    const int goal  = map.goal_idx;

    int P = num_threads;
    if (P < 1) P = 1;

    double ms_ctx_init  = 0.0;
    double ms_build_map = 0.0;
    double ms_alloc     = 0.0;
    double ms_seed      = 0.0;
    double ms_work      = 0.0;
    double ms_merge     = 0.0;

    Timer t_total;

    AStarContext ctx;

    // 0) ctx init + atomics
    {
        Timer t;
        ctx.map = &map;
        ctx.h   = &h;
        ctx.P   = P;
        ctx.start_idx = start;
        ctx.goal_idx  = goal;

        ctx.goal_found.v.store(false, std::memory_order_relaxed);
        ctx.best_goal_g.v.store(std::numeric_limits<float>::infinity(),
                                std::memory_order_relaxed);
        ms_ctx_init = t.elapsed_ms();
    }

    // 1) Build ownership mapping
    {
        Timer t;
        ctx.owners = build_owner_map(map, P);
        ms_build_map = t.elapsed_ms();
    }

    // 2) Allocate global node arrays + per-thread state + mailboxes
    {
        Timer t;

        ctx.g.assign(N, std::numeric_limits<float>::infinity());
        ctx.f.assign(N, std::numeric_limits<float>::infinity());
        ctx.parent.assign(N, -1);
        ctx.st.assign(N, 0);

        ctx.tdata.resize(P);
        ctx.inbox.resize(P);
        for (int t2 = 0; t2 < P; ++t2) {
            ctx.tdata[t2] = std::make_unique<ThreadData>();
            ctx.inbox[t2] = std::make_unique<Mailbox>();
        }

        ms_alloc = t.elapsed_ms();
    }

    // 3) Seed start message
    {
        Timer t;
        int owner_start = ctx.owners.owner[start];
        Mailbox &mb = *ctx.inbox[owner_start];

        uint32_t old = mb.pending.fetch_add(1u, std::memory_order_release);

        pthread_mutex_lock(&mb.mtx);
        mb.q.push_back(Message{start, 0.0f, -1});
        pthread_mutex_unlock(&mb.mtx);

        if (old == 0) pthread_cond_signal(&mb.cv);

        ms_seed = t.elapsed_ms();
    }

    // 4) Spawn + join threads
    {
        Timer t;

        std::vector<pthread_t> threads(P);
        std::vector<WorkerArgs> args(P);

        for (int t2 = 0; t2 < P; ++t2) {
            args[t2] = WorkerArgs{&ctx, t2};
            pthread_create(&threads[t2], nullptr, astar_worker, &args[t2]);
        }
        for (int t2 = 0; t2 < P; ++t2) {
            pthread_join(threads[t2], nullptr);
        }

        ms_work = t.elapsed_ms();
    }

    bool found = ctx.goal_found.v.load(std::memory_order_relaxed);
    double ms_overhead = ms_ctx_init + ms_build_map + ms_alloc + ms_seed;

    if (!found) {
        double total_ms = t_total.elapsed_ms();
        std::cout << "[run_astar breakdown ms] "
                  << "overhead=" << ms_overhead
                  << " work=" << ms_work
                  << " merge=0"
                  << " total=" << total_ms
                  << "\n";
        return false;
    }

    // 5) Merge/export parents (goal->start chain) from global parent[idx]
    {
        Timer t;

        set_parent(state, start, -1);

        int cur = goal;
        int guard = 0;
        while (cur != -1 && guard++ < N + 5) {
            int p = ctx.parent[cur];
            set_parent(state, cur, p);
            if (cur == start) break;
            cur = p;
        }

        ms_merge = t.elapsed_ms();
    }

    double total_ms = t_total.elapsed_ms();
    if (out_tm) {
        out_tm->overhead_ms = ms_overhead;
        out_tm->work_ms     = ms_work;
        out_tm->merge_ms    = found ? ms_merge : 0.0;
        out_tm->total_ms    = total_ms;
    }

    return true;
}
