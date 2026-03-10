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
