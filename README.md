# Parallel A* on grid map
The project is for final project of NYCU parallel programming 2025.<br>
This is an implementation of Hashed Distributed A* with implementation tuning.<br>
(https://github.com/jinnaiyuu/Hash-Distributed-Astar)<br>
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
