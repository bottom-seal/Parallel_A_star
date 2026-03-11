# Parallel A* on Grid Map

> Final Project — **NYCU Parallel Programming 2025**

This project implements **Hashed Distributed A\*** with several implementation optimizations.

Reference implementation:  
https://github.com/jinnaiyuu/Hash-Distributed-Astar

A* is inherently difficult to parallelize because each iteration depends on the result of the previous one.

<br>

<p align="center">
<img width="864" src="https://github.com/user-attachments/assets/c16d56a3-e5b1-4f72-9989-ee60ee205c73">
</p>

---

# Table of Contents

- [Proposed Solution](#proposed-solution)
- [Implementation Details](#implementation-details-and-deviation)
- [Experimental Methodology](#experimental-methodology)
- [Experimental Results](#experimental-results)
- [Discussion](#discussion)
- [Using the Code](#using-the-code)

---

# Proposed Solution

Our initial proposal of **grid partitioning** was inefficient.  
Instead, we implemented **HDA\*** (Hash Distributed A\*).

The key insight is that **each thread performs node expansion independently**.

Core principles:

```
1. Each thread performs iterations with its own open set
2. Each node is hashed to a thread, which becomes its owner
3. Each thread maintains a local OPEN SET
4. Nodes belonging to other threads are forwarded to their owner
5. Owners update f(n) and keep the minimum value
```

### Algorithm

```text
while not goal found globally do
    Check global goal found flag
    Drain inbox for pending messages
    Prune nodes by minimum f(n)

    if open set not empty then
        Pop node with minimum f(n)

        if node is goal then
            Set global goal found flag
            break
        end if

        Expand neighbors

        for each neighbor do
            if owner is self then
                Update local open set
            else
                Add to outbox
            end if
        end for

        Send outbox if buffer full
    else
        sched_yield()
    end if
end while
```

---

# Implementation Details and Deviation

## Synchronization and Communication

The **global node map** stores the complete state of all nodes:

- visited status  
- parent pointers  
- g values  
- f values  

This map is shared across all threads.

Although this introduces potential **false sharing**, experiments show the performance gain from avoiding local node maps outweighs the cache coherence cost.

### Communication Design

The main challenge is **efficient inter-thread communication**.

Our first implementation used **mutex-protected per-message communication**, which scaled poorly.

The final design reduces synchronization using **message batching**.

Each thread maintains:

| Structure | Purpose |
|----------|--------|
| Outbox buffer | Batch messages for each target thread |
| Inbox queue | Receive messages protected by a single mutex |

Messages are sent **in batches** when the buffer fills, greatly reducing lock contention.

---

## Work Partitioning Strategy

Hash-based partitioning distributes nodes by **index rather than spatial location**, ensuring balanced workload regardless of maze structure.

However, this causes frequent cross-thread communication.

To mitigate this, we introduced **tile-based ownership**:

1. Partition the maze into fixed-size tiles
2. Assign tiles to workers
3. Each worker owns nodes within its tiles

This reduces cross-worker communication because neighboring nodes often stay within the same tile.

---

## Thread Scheduling Strategy

When a thread's open set becomes empty, it must wait for work.

Two strategies were evaluated.

### `pthread_cond_timedwait()`

- Threads sleep using condition variables
- High system call overhead

### `sched_yield()`

- Threads yield CPU time
- Allows active workers to continue processing
- Much lower overhead

`sched_yield()` provided the best balance between responsiveness and efficiency.

---

# Experimental Methodology

## Test Environment

| Component | Specification |
|----------|---------------|
| CPU | Intel Core i5-14400F |
| Cores | 10 cores / 16 threads |
| Memory | 32 GB |

---

## Input Data Sets

Maze sizes ranged from **1600² to 12800²**.

```
1600²
3200²
4800²
6400²
8000²
9600²
11200²
12800²
```

Maze generation guarantees:

- solvable paths
- multiple valid routes
- non-trivial pathfinding difficulty

---

## Performance Metrics

| Metric | Description |
|------|-------------|
| Execution Time | Wall-clock time from start to goal |
| Speedup | Sequential time / parallel time |

Sequential A* is used as the baseline.

---

# Experimental Results

## Speedup vs Thread Count

<p align="center">
<img width="925" src="https://github.com/user-attachments/assets/ce9a4943-1c82-4a6e-8cba-27ea0100a688">
</p>

<p align="center">
<img width="925" src="https://github.com/user-attachments/assets/9cb472a9-bf0f-49e2-96a2-c3d5f453da69">
</p>

---

## Performance Breakdown

<p align="center">
<img width="933" src="https://github.com/user-attachments/assets/9e5505b2-54a4-4030-bd5f-394d109e8275">
</p>

Performance breakdown for maze size **6400²**.

<p align="center">
<img width="932" src="https://github.com/user-attachments/assets/b29b126a-ec28-4f1c-b1d4-b63d5243247a">
</p>

Performance breakdown for maze size **12800²**.

---

## Scalability Across Maze Sizes

<p align="center">
<img width="1546" src="https://github.com/user-attachments/assets/02eeaa21-f1cf-4e5d-890e-f26cc24ec358">
</p>

<p align="center">
<img width="1546" src="https://github.com/user-attachments/assets/c22f2e7f-82dd-4f6d-9879-23f5dfaedbde">
</p>

---

# Discussion

## Performance

Speedup increases with maze size, consistent with **Amdahl's Law**.

For small mazes (1600²):

- parallel overhead offsets benefits

For large mazes (12800²):

- significant acceleration observed
- 16-thread configuration shows strong speedup

As the problem size increases, computation dominates overhead, improving scalability.

---

## Correctness

Correctness was validated by comparing path lengths between sequential and parallel implementations.

Results show:

- valid path found whenever one exists
- path length closely matches sequential baseline
- small deviations may occur due to optimization

---

# Using the Code

Only essential components are included to reproduce the results.

## Generate a Map

```
cd map
activate venv
python generate_maze_map.py [size] [seed] [loops]
```

Example

```
python generate_maze_map.py 12800 777 4800
```

More loops create more possible routes.

---

## Run Serial A*

```
cd serial
make
./astar [map_path]
```

Example

```
./astar ../map/maze_map_12800.txt
```

---

## Run HDA*

```
cd eval
make
./astar [map_path] -t [thread_num]
```

Example

```
./astar ../map/maze_map_12800.txt -t 16
```
