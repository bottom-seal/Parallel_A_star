# Parallel A* on grid map
The project is for final project of NYCU parallel programming 2025.<br>
This is an implementation of Hashed Distributed A* with implementation tuning.<br>
(https://github.com/jinnaiyuu/Hash-Distributed-Astar)<br>
A* algorithm is inherently difficult to parallelize as each iteration depends on the result of the last
<img width="864" height="387" alt="image" src="https://github.com/user-attachments/assets/c16d56a3-e5b1-4f72-9989-ee60ee205c73" />

## Proposed Solution
Our initial proposal of partitioning grid map was proved to be inefficient, so we implemented HDA*(Hash Distributed A*) algorithm, the key insight is that each thread can do node expansion in parallel.<br>
for complete algorithm, please check the linked source.
```
1. Each thread can do the iteration with its openset
2. Each node hashed to a thread, who becomes the owner of it.
3. Each thread keeps local OPEN SET, no global OPEN SET is kept.
4. If one thread discover node belonging to other thread, need to send info of the node to the owner.
5. Upon receiving info of its node, thread compare f(n), update to min. (ensures optimal)
```
Below is the implementation algorithm
```text
1:  while not goal found globally do
2:      Check global goal found flag
3:      Drain inbox for pending messages
4:      Prune: compare with discovered node, keep only minimum f(n) for each node
5:      if open set not empty then
6:          Pop node with minimum f(n)
7:          if node is goal then
8:              Set global goal found flag
9:              BREAK
10:         end if
11:         Expand neighbors
12:         for each neighbor do
13:             if owner is self then
14:                 Update local open set
15:             else
16:                 Add to outbox for owner thread
17:             end if
18:         end for
19:         Send outbox if buffer full
20:     else
21:         sched_yield() to other threads
22:     end if
23: end while
```
## Implementation details and deviation
### Synchronization and Communication
The global node map stores the complete state of all nodes (visited status, parent pointers, g and f values) and is shared across all
threads. While this introduces potential false sharing, our experiments demonstrate that the benefit of avoiding local node map parsing
overhead outweighs the cache coherence costs.

The most challenging aspect of the implementation is managing interthread communication efficiently. Our initial implementation used
immediate mutex-protected message passing, which resulted in severe performance degradation as thread count increased. The key
insight was to reduce synchronization frequency through buffering.<br>
Each thread maintains:<br>

• A private outbox buffer (per-target-thread) for batching messages<br>
• An inbox queue protected by a single mutex for receiving messages<br>

Messages are sent in batches when the outbox buffer fills, reducing
mutex contention by orders of magnitude compared to per-message
locking. The receiving thread processes its entire inbox in one critical
section, further minimizing lock hold time.

### Work Partitioning Strategy
The hash-based partitioning strategy provides several advantages over
spatial decomposition approaches. By distributing nodes based on
their index rather than their location, we ensure balanced workload
distribution regardless of maze structure.

However, such approach introduced another problem, whenever a new node is discovered, it likely belongs to another thread, which causes constant communication between threads and contention for lock. So we adopted applied a tiling-based spatial
ownership scheme: we first partition the maze into fixed-size tiles,
then assign tiles to workers so that each worker “owns” all nodes
within its tiles. This reduces cross-worker communication because
expanded nodes are more likely to remain within the same tile region
and therefore be owned by the same worker. 

The tradeoff between load balancing and communication proved to be benefitial in the experiment.
### Thread Scheduling Strategy
When a thread’s open set becomes empty (meaning all its nodes have
been explored or are waiting for updates), it must decide how to wait
for more work. We evaluated two strategies:<br>

```pthread cond timedwait()```: Threads sleep on a condition variable
with timeout. This proved too costly due to the overhead of the timed
wait system call.<br>
```sched yield()```: Threads yield their time slice to other threads. This
provides the best balance - minimal overhead while allowing productive threads to continue work.<br>

The sched yield() approach allows idle threads to quickly wake up
when new messages arrive (sent by other threads) without the heavyweight system call overhead of condition variables.

## Experimental Methodology
###  Test Environment
• Processor: Intel(R) Core(TM) i5-14400F, 10 cores 16 threads<br>
• Memory: 32GB<br>
###  Input Data Sets
We generated maze datasets with varying sizes to evaluate scalability,  
ranging from 1600<sup>2</sup> to 12800<sup>2</sup> cells with increments of 1600  
(1600<sup>2</sup>, 3200<sup>2</sup>, 4800<sup>2</sup>, 6400<sup>2</sup>, 8000<sup>2</sup>, 9600<sup>2</sup>, 11200<sup>2</sup>, and 12800<sup>2</sup>).

Maze generation ensures solvability with a guaranteed path from
start to goal. The maze complexity is controlled to provide meaningful pathfinding challenges while avoiding trivial straight-line solutions. Additionally, we enforce that multiple distinct paths exist
between start and goal, so the solver must choose among alternatives,
this helps validate correctness.
###  Performance Metrics
• Execution Time: Total wall-clock time from start to goal discovery<br>
• Speedup: Ratio of sequential execution time to parallel execution time<br>
The sequential A* baseline is used fir speedup comparison

## Experimental Results
### Speed up with different number of threads on and 6400<sup>2</sup> and 12800<sup>2</sup>
<img width="925" height="509" alt="image" src="https://github.com/user-attachments/assets/ce9a4943-1c82-4a6e-8cba-27ea0100a688" />
<img width="925" height="509" alt="image" src="https://github.com/user-attachments/assets/9cb472a9-bf0f-49e2-96a2-c3d5f453da69" />

### Performance Breakdown Analysis
<img width="933" height="462" alt="image" src="https://github.com/user-attachments/assets/9e5505b2-54a4-4030-bd5f-394d109e8275" />
Performance breakdown for maze size 6400<sup>2</sup>
showing overhead, work, and merge time across different thread counts. The dashed line indicates sequential execution time.
<img width="932" height="462" alt="image" src="https://github.com/user-attachments/assets/b29b126a-ec28-4f1c-b1d4-b63d5243247a" />
Performance breakdown for maze size 12800<sup>2</sup>

###  Scalability Across Maze Sizes
<img width="1546" height="766" alt="image" src="https://github.com/user-attachments/assets/02eeaa21-f1cf-4e5d-890e-f26cc24ec358" />
Speedup comparison between sequential A* and 16-thread
HDA* across different maze sizes. Pink bars show speedup percentage (right axis), while lines show absolute execution times (left axis).
<img width="1546" height="766" alt="image" src="https://github.com/user-attachments/assets/c22f2e7f-82dd-4f6d-9879-23f5dfaedbde" />
Speedup comparison using local node maps with sleepbased synchronization. The implementation shows negative speedup
(slowdown) for small mazes and limited improvement for larger mazes.

## Discussion
### Performance
Our experiments demonstrate that speedup scales positively with
maze size, consistent with Amdahl’s Law predictions. For small
mazes (1600<sup>2</sup>
), the parallel implementation performs comparably to
the single-threaded version, as parallelization overhead offsets the
benefits of multi-threading. However, when the maze size increases
to 12800<sup>2</sup>
, we observe significant acceleration, with the 16-thread
configuration achieving substantial speedup 

This behavior aligns with Amdahl’s Law expectations: as problem
size grows, the total amount of search work increases, so overheads
(even though they also grow)—such as map-sized initialization and
communication overhead—become a smaller fraction of the overall
runtime. In small problems, these costs take up a disproportionate
share and dominate performance; in larger problems, the increased
computation amortizes them, resulting in better overall speedup and
making our approach more suitable for large-scale pathfinding tasks.
###  Correctness
We verified correctness by comparing path lengths between sequential and parallel implementations across all test cases. The parallel
version consistently found a valid path whenever one exists, and its
path length closely matched the sequential baseline. While it did not
always return the exact optimal path length, the deviation was small,
indicating that our optimizations and pruning strategies preserve solution quality even when strict A* optimality is not guaranteed







