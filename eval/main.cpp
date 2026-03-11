// main.cpp
#include <iostream>
#include <vector>
#include <string>
#include <cstdlib>
#include <thread>

#include "helper.h"
#include "astar_interface.h"

static int parse_pos_int_or_throw(const char* s, const char* what) {
    char* end = nullptr;
    long v = std::strtol(s, &end, 10);
    if (!s || *s == '\0' || (end && *end != '\0')) {
        throw std::runtime_error(std::string("Invalid ") + what + ": '" + (s ? s : "") + "'");
    }
    if (v <= 0 || v > 1'000'000) {
        throw std::runtime_error(std::string(what) + " out of range: " + std::to_string(v));
    }
    return static_cast<int>(v);
}

int main(int argc, char **argv) {
    // Defaults
    std::string map_file;
    int num_threads = 1;
    int num_runs = 10;

#ifdef PTHREAD_ASTAR
    unsigned hc = std::thread::hardware_concurrency();
    num_threads = (hc == 0 ? 4 : static_cast<int>(hc));
#endif

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <map_file> [-t threads] [-r runs]\n";
        return 1;
    }

    map_file = argv[1];

    try {
        for (int i = 2; i < argc; ++i) {
            std::string a = argv[i];

            if ((a == "-t" || a == "--threads") && i + 1 < argc) {
                num_threads = parse_pos_int_or_throw(argv[++i], "threads");
            } else if ((a == "-r" || a == "--runs") && i + 1 < argc) {
                num_runs = parse_pos_int_or_throw(argv[++i], "runs");
            } else if (a == "-h" || a == "--help") {
                std::cout << "Usage: " << argv[0] << " <map_file> [-t threads] [-r runs]\n";
                std::cout << "  -t, --threads   Number of worker threads (parallel build only)\n";
                std::cout << "  -r, --runs      Number of repeated runs for averaging\n";
                return 0;
            } else {
                throw std::runtime_error("Unknown/invalid argument: " + a);
            }
        }

#ifdef PTHREAD_ASTAR
        std::cout << "[Config] Parallel A* (pthreads) with " << num_threads << " threads\n";
#else
        if (num_threads != 1) {
            std::cout << "[Config] Serial A* (no threading). Ignoring -t " << num_threads << "\n";
        } else {
            std::cout << "[Config] Serial A* (no threading)\n";
        }
        num_threads = 1;
#endif

        GridMap map = load_grid_map_from_file(map_file.c_str());
        int N = map.width * map.height;

        std::vector<float> hvals = precompute_heuristic_manhattan(map);

        double total_ms = 0.0;
        bool found_any = false;

        AStarState best_state(N);
        bool have_best_state = false;

        // --- accumulate breakdown timings ---
        AStarTimings sum_tm{};
        sum_tm.overhead_ms = 0.0;
        sum_tm.work_ms     = 0.0;
        sum_tm.merge_ms    = 0.0;
        sum_tm.total_ms    = 0.0;

        for (int i = 0; i < num_runs; ++i) {
            AStarState state(N);

            Timer t_wall;                 // your existing wall timer (optional)
            AStarTimings tm{};            // per-run breakdown

            bool found = run_astar(map, state, hvals, num_threads, &tm);

            double wall_ms = t_wall.elapsed_ms();
            total_ms += wall_ms;

            // sum breakdown
            sum_tm.overhead_ms += tm.overhead_ms;
            sum_tm.work_ms     += tm.work_ms;
            sum_tm.merge_ms    += tm.merge_ms;
            sum_tm.total_ms    += tm.total_ms;

            // Keep the first run's solution/path as "best" (same as your original)
            if (i == 0) {
                found_any = found;
                if (found) {
                    best_state = state;
                    have_best_state = true;
                }
            }
        }

        double avg_ms = total_ms / num_runs;

        auto avg = [&](double x) { return x / (double)num_runs; };

        std::cout << "Found: " << found_any << "\n";
        std::cout << "Average search time over " << num_runs
                << " runs : " << avg_ms << " ms\n";

        // --- average breakdown from run_astar's internal timers ---
        std::cout << "[avg run_astar breakdown ms] "
                << "overhead=" << avg(sum_tm.overhead_ms)
                << " work="    << avg(sum_tm.work_ms)
                << " merge="   << avg(sum_tm.merge_ms)
                << " total="   << avg(sum_tm.total_ms)
                << "\n";

        if (found_any && have_best_state) {
            auto path = reconstruct_path(map, best_state);
            if (!path.empty()) {
                int path_length_edges = static_cast<int>(path.size()) - 1;
                std::cout << "Path length (edges) : " << path_length_edges << "\n";
                std::cout << "Path length (nodes) : " << path.size() << "\n";

                save_path_as_grid_txt(map, path, "best_path.txt");
                std::cout << "Path written to best_path.txt\n";
            }
        }
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
