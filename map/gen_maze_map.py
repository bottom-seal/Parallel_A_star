import numpy as np
# import matplotlib.pyplot as plt  # PNG output disabled
import sys
from typing import List, Tuple, Dict
from collections import deque


# ------------------------------------------------------------
# Basic helpers
# ------------------------------------------------------------

def is_power_of_two_or_mult16(n: int) -> bool:
    """Allow sizes that are power-of-two OR multiple of 16."""
    return n > 0 and (((n & (n - 1)) == 0) or (n % 16 == 0))


def visualize_map(grid: np.ndarray, out_path: str) -> None:
    """
    Visualize a 0/1 map (square) and save as PNG.
    1 = wall (black), 0 = path (white)

    NOTE: PNG output is disabled in CLI below; keep this if you want it later.
    """
    # Lazy import so matplotlib isn't required unless you call this function.
    import matplotlib.pyplot as plt

    grid = np.array(grid, dtype=int)

    if grid.ndim != 2:
        raise ValueError("Map must be 2D (H x W).")

    h, w = grid.shape
    if h != w:
        raise ValueError(f"Map must be square, got {h}x{w}.")
    if not is_power_of_two_or_mult16(h):
        raise ValueError(
            f"Map size must be power of 2 (32, 64, 128, ...) or multiple of 16. Got {h}."
        )
    if not np.all(np.isin(grid, [0, 1])):
        raise ValueError("Map must contain only 0 and 1.")

    img = np.where(grid == 1, 0, 255).astype(np.uint8)

    plt.figure(figsize=(4, 4))
    plt.imshow(img, cmap="gray", interpolation="nearest", vmin=0, vmax=255)
    plt.axis("off")
    plt.tight_layout(pad=0)
    plt.savefig(out_path, dpi=200, bbox_inches="tight", pad_inches=0)
    plt.close()
    print(f"Saved image to {out_path}")


# ------------------------------------------------------------
# Maze generation on cell grid (odd coordinates)
# ------------------------------------------------------------

def generate_perfect_maze_cells(
    size: int,
    seed: int = 42,
) -> Tuple[np.ndarray, int, int]:
    """
    Generate a perfect maze (spanning tree) on a cell grid embedded
    inside an NxN grid.

    - N = size (must be power-of-two OR multiple of 16)
    - We treat positions with odd coordinates as cell centers:
        cell (rc, cc) -> grid coords (2*rc+1, 2*cc+1)
      with rc,cc in [0, H-1], where H = (N-1)//2.

    Returns:
      grid : NxN array with 1=wall, 0=carved paths (maze corridors)
      H, W : number of cells in each dimension (H == W)
    """
    if not is_power_of_two_or_mult16(size):
        raise ValueError("Size must be power of 2 (32, 64, 128, ...) or multiple of 16.")
    if size < 8:
        raise ValueError("Use size >= 8 to make sense.")

    N = size
    H = (N - 1) // 2  # number of cells per dimension
    W = H

    rng = np.random.default_rng(seed)

    # All walls initially
    grid = np.ones((N, N), dtype=int)

    # DFS on cell grid
    visited = np.zeros((H, W), dtype=bool)

    # Start near your logical start point at (1, N-1).
    # The interior cell closest to that is top-right cell:
    start_cell = (0, W - 1)  # cell indices
    stack: List[Tuple[int, int]] = [start_cell]
    visited[start_cell] = True

    # Carve the starting cell
    sr_cell, sc_cell = start_cell
    sr, sc = 2 * sr_cell + 1, 2 * sc_cell + 1
    grid[sr, sc] = 0

    def cell_neighbors(rc: int, cc: int) -> List[Tuple[int, int]]:
        out: List[Tuple[int, int]] = []
        if rc > 0:
            out.append((rc - 1, cc))
        if rc < H - 1:
            out.append((rc + 1, cc))
        if cc > 0:
            out.append((rc, cc - 1))
        if cc < W - 1:
            out.append((rc, cc + 1))
        return out

    while stack:
        rc, cc = stack[-1]
        neighbors = cell_neighbors(rc, cc)
        rng.shuffle(neighbors)

        moved = False
        for nrc, ncc in neighbors:
            if not visited[nrc, ncc]:
                visited[nrc, ncc] = True

                # carve wall between (rc,cc) and (nrc,ncc)
                r1, c1 = 2 * rc + 1, 2 * cc + 1
                r2, c2 = 2 * nrc + 1, 2 * ncc + 1
                mr, mc = (r1 + r2) // 2, (c1 + c2) // 2  # wall coordinate

                grid[mr, mc] = 0
                grid[r2, c2] = 0

                stack.append((nrc, ncc))
                moved = True
                break

        if not moved:
            stack.pop()

    return grid, H, W


# ------------------------------------------------------------
# Build cell graph & shortest path in cell-space
# ------------------------------------------------------------

def build_cell_graph_from_grid(
    grid: np.ndarray,
    H: int,
    W: int,
) -> Dict[Tuple[int, int], List[Tuple[int, int]]]:
    """
    Build adjacency graph between cells based on carved walls in grid.

    cell (rc,cc) -> grid coords (2*rc+1, 2*cc+1)
    Two cells are adjacent if the wall between them is open (grid == 0).
    """
    graph: Dict[Tuple[int, int], List[Tuple[int, int]]] = {}
    N = grid.shape[0]

    for rc in range(H):
        for cc in range(W):
            node = (rc, cc)
            graph[node] = []
            r_base, c_base = 2 * rc + 1, 2 * cc + 1

            # up
            if rc > 0:
                mr, mc = r_base - 1, c_base
                if 0 <= mr < N and grid[mr, mc] == 0:
                    graph[node].append((rc - 1, cc))
            # down
            if rc < H - 1:
                mr, mc = r_base + 1, c_base
                if 0 <= mr < N and grid[mr, mc] == 0:
                    graph[node].append((rc + 1, cc))
            # left
            if cc > 0:
                mr, mc = r_base, c_base - 1
                if 0 <= mr < N and grid[mr, mc] == 0:
                    graph[node].append((rc, cc - 1))
            # right
            if cc < W - 1:
                mr, mc = r_base, c_base + 1
                if 0 <= mr < N and grid[mr, mc] == 0:
                    graph[node].append((rc, cc + 1))

    return graph


def shortest_cell_path(
    graph: Dict[Tuple[int, int], List[Tuple[int, int]]],
    start_cell: Tuple[int, int],
    goal_cell: Tuple[int, int],
) -> List[Tuple[int, int]]:
    """BFS shortest path on cell graph."""
    q = deque([start_cell])
    visited = {start_cell}
    parent: Dict[Tuple[int, int], Tuple[int, int]] = {}

    while q:
        u = q.popleft()
        if u == goal_cell:
            break
        for v in graph[u]:
            if v not in visited:
                visited.add(v)
                parent[v] = u
                q.append(v)
    else:
        raise RuntimeError("No path between start and goal in cell graph (should not happen).")

    path: List[Tuple[int, int]] = []
    cur = goal_cell
    while cur != start_cell:
        path.append(cur)
        cur = parent[cur]
    path.append(start_cell)
    path.reverse()
    return path


# ------------------------------------------------------------
# Add guaranteed extra path along S->G
# ------------------------------------------------------------

def add_extra_path_along_sg(
    grid: np.ndarray,
    H: int,
    W: int,
    cell_path: List[Tuple[int, int]],
    rng: np.random.Generator,
) -> None:
    """
    Ensure there is at least one additional path from S to G by opening
    an extra connection between two non-consecutive cells on the S->G
    cell path, if possible.
    """
    index_of: Dict[Tuple[int, int], int] = {cell: idx for idx, cell in enumerate(cell_path)}
    N = grid.shape[0]
    candidates: List[Tuple[int, int]] = []

    for idx, (rc, cc) in enumerate(cell_path):
        r_base, c_base = 2 * rc + 1, 2 * cc + 1

        neighbors = [
            (rc - 1, cc, r_base - 1, c_base),  # up
            (rc + 1, cc, r_base + 1, c_base),  # down
            (rc, cc - 1, r_base, c_base - 1),  # left
            (rc, cc + 1, r_base, c_base + 1),  # right
        ]

        for nrc, ncc, mr, mc in neighbors:
            if not (0 <= nrc < H and 0 <= ncc < W):
                continue
            neighbor_cell = (nrc, ncc)
            if neighbor_cell not in index_of:
                continue

            j = index_of[neighbor_cell]
            if abs(j - idx) <= 1:
                continue

            if 0 <= mr < N and 0 <= mc < N and grid[mr, mc] == 1:
                candidates.append((mr, mc))

    if not candidates:
        return

    mr, mc = candidates[int(rng.integers(low=0, high=len(candidates)))]
    grid[mr, mc] = 0


# ------------------------------------------------------------
# Add some random extra loops elsewhere (optional)
# ------------------------------------------------------------

def add_random_extra_loops(
    grid: np.ndarray,
    H: int,
    W: int,
    extra_loops: int,
    rng: np.random.Generator,
) -> None:
    """
    Randomly open additional walls between neighboring cells to create
    more cycles throughout the maze.
    """
    N = grid.shape[0]
    if extra_loops <= 0:
        return

    for _ in range(extra_loops * 5):
        rc = int(rng.integers(low=0, high=H))
        cc = int(rng.integers(low=0, high=W))

        r_base, c_base = 2 * rc + 1, 2 * cc + 1
        directions = [
            (rc - 1, cc, r_base - 1, c_base),
            (rc + 1, cc, r_base + 1, c_base),
            (rc, cc - 1, r_base, c_base - 1),
            (rc, cc + 1, r_base, c_base + 1),
        ]
        rng.shuffle(directions)

        for nrc, ncc, mr, mc in directions:
            if not (0 <= nrc < H and 0 <= ncc < W):
                continue
            if 0 <= mr < N and 0 <= mc < N and grid[mr, mc] == 1:
                grid[mr, mc] = 0
                extra_loops -= 1
                break

        if extra_loops <= 0:
            break


# ------------------------------------------------------------
# Main generator
# ------------------------------------------------------------

def generate_maze_map(
    size: int,
    seed: int = 42,
    extra_loops: int = 10,
) -> np.ndarray:
    """
    Generate an "actual maze"-like map:

      - DFS perfect maze on embedded cell grid
      - Start = (1, size-1)
      - Goal  = (size-2, 0)
      - Adds at least one extra S->G cycle
      - Adds extra random loops

    Returns:
      grid : NxN array with 1 = wall, 0 = path.
    """
    N = size
    if not is_power_of_two_or_mult16(N):
        raise ValueError("Size must be power of 2 (32, 64, 128, ...) or multiple of 16.")

    rng = np.random.default_rng(seed)

    grid, H, W = generate_perfect_maze_cells(N, seed=seed)

    graph = build_cell_graph_from_grid(grid, H, W)

    start_cell = (0, W - 1)
    goal_cell = (H - 1, 0)

    cell_path = shortest_cell_path(graph, start_cell, goal_cell)

    add_extra_path_along_sg(grid, H, W, cell_path, rng)
    add_random_extra_loops(grid, H, W, extra_loops=extra_loops, rng=rng)

    # Start border opening
    grid[1, N - 1] = 0
    grid[1, N - 2] = 0

    # Goal border opening
    grid[N - 2, 0] = 0
    grid[N - 2, 1] = 0

    return grid


# ------------------------------------------------------------
# CLI
# ------------------------------------------------------------

if __name__ == "__main__":
    """
    Usage:
      python gen_maze_map.py
      python gen_maze_map.py 128
      python gen_maze_map.py 128 123
      python gen_maze_map.py 128 123 30
    """
    if len(sys.argv) >= 2:
        size = int(sys.argv[1])
    else:
        size = 64

    if len(sys.argv) >= 3:
        seed = int(sys.argv[2])
    else:
        seed = 42

    if len(sys.argv) >= 4:
        extra_loops = int(sys.argv[3])
    else:
        extra_loops = 10

    grid = generate_maze_map(size=size, seed=seed, extra_loops=extra_loops)

    txt_name = f"maze_map_{size}.txt"
    png_name = f"maze_map_{size}.png"  # PNG output disabled

    np.savetxt(txt_name, grid, fmt="%d")
    print(f"Saved maze map data to {txt_name}")

    #visualize_map(grid, png_name)  # PNG output disabled
