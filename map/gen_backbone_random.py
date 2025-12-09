import numpy as np
import matplotlib.pyplot as plt
import sys
from typing import List, Tuple, Dict


def is_power_of_two(n: int) -> bool:
    return n > 0 and (n & (n - 1)) == 0


def visualize_map(grid: np.ndarray, out_path: str) -> None:
    """
    Visualize a 0/1 map (square, power-of-two side) and save as PNG.
    1 = wall (black), 0 = path (white)
    """
    grid = np.array(grid, dtype=int)

    if grid.ndim != 2:
        raise ValueError("Map must be 2D (H x W).")

    h, w = grid.shape
    if h != w:
        raise ValueError(f"Map must be square, got {h}x{w}.")
    if not is_power_of_two(h):
        raise ValueError(f"Map size must be power of 2 (32, 64, 128, ...). Got {h}.")
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


def neighbors_4(r: int, c: int, size: int) -> List[Tuple[int, int]]:
    """4-neighbor moves inside the grid."""
    cand = [
        (r - 1, c),  # up
        (r + 1, c),  # down
        (r, c - 1),  # left
        (r, c + 1),  # right
    ]
    return [(rr, cc) for rr, cc in cand if 0 <= rr < size and 0 <= cc < size]


def generate_backbone_random_path(
    size: int,
    seed: int = 42,
) -> Tuple[np.ndarray, List[Tuple[int, int]]]:
    """
    Generate a single simple backbone path from start to goal,
    allowed moves: up / down / left / right.

    Start: (1, size-1)
    Goal:  (size-2, 0)

    Method:
      - Run a random DFS (iterative) on the full grid from the start.
      - This builds a random spanning tree (parent pointers).
      - When we reach the goal, we reconstruct the path along the tree.
    """
    if not is_power_of_two(size):
        raise ValueError("Size must be power of 2 (32, 64, 128, ...).")
    if size < 8:
        raise ValueError("Use size >= 8 to make sense.")

    start = (1, size - 1)
    goal = (size - 2, 0)

    rng = np.random.default_rng(seed)

    visited = np.zeros((size, size), dtype=bool)
    parent: Dict[Tuple[int, int], Tuple[int, int]] = {}

    stack: List[Tuple[int, int]] = [start]
    visited[start] = True

    found_goal = False

    while stack:
        r, c = stack.pop()
        if (r, c) == goal:
            found_goal = True
            break

        neigh = neighbors_4(r, c, size)
        rng.shuffle(neigh)

        for nr, nc in neigh:
            if not visited[nr, nc]:
                visited[nr, nc] = True
                parent[(nr, nc)] = (r, c)
                stack.append((nr, nc))

    if not found_goal:
        raise RuntimeError("DFS failed to reach goal, which should not happen on a full grid.")

    # Reconstruct path from goal back to start using parent pointers
    path: List[Tuple[int, int]] = []
    cur = goal
    while cur != start:
        path.append(cur)
        cur = parent[cur]
    path.append(start)
    path.reverse()  # now from start -> goal

    # Build grid: 1 = wall, 0 = path
    grid = np.ones((size, size), dtype=int)
    for (r, c) in path:
        grid[r, c] = 0

    # Ensure start/goal are open
    sr, sc = start
    gr, gc = goal
    grid[sr, sc] = 0
    grid[gr, gc] = 0

    return grid, path


if __name__ == "__main__":
    """
    Usage:
      python gen_backbone_random.py
          # size=64, seed=42

      python gen_backbone_random.py 128
          # size=128, seed=42

      python gen_backbone_random.py 128 123
          # size=128, seed=123 (different random backbone)
    """
    if len(sys.argv) >= 2:
        size = int(sys.argv[1])
    else:
        size = 64

    if len(sys.argv) >= 3:
        seed = int(sys.argv[2])
    else:
        seed = 42

    grid, path = generate_backbone_random_path(size=size, seed=seed)

    txt_name = f"backbone_random_{size}_seed{seed}.txt"
    png_name = f"backbone_random_{size}_seed{seed}.png"

    np.savetxt(txt_name, grid, fmt="%d")
    print(f"Saved backbone map data to {txt_name}")
    print(f"Backbone path length = {len(path)} steps")

    visualize_map(grid, png_name)
