# Parallel A* on grid map
The project is for final project of NYCU parallel programming 2025.<br>
This is an implementation of Hashed Distributed A* with implementation tuning.<br>
(https://github.com/jinnaiyuu/Hash-Distributed-Astar)<br>
<img width="864" height="387" alt="image" src="https://github.com/user-attachments/assets/c16d56a3-e5b1-4f72-9989-ee60ee205c73" />

## Proposed Solution
Our initial proposal of partitioning grid map was proved to be inefficient, so we implemented HDA*(Hash Distributed A*) algorithm, the key insight is that each 
