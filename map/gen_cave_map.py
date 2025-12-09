import numpy as np
import matplotlib.pyplot as plt
import sys
from typing import List, Tuple, Dict


# ------------------------------------------------------------
# Basic helpers
# ------------------------------------------------------------

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


# ------------------------------------------------------------
# Backbone generation (4-direction DFS)
# ------------------------------------------------------------

def generate_backbone_random_path(
    size: int,
    seed: int = 42,
) -> Tuple[np.ndarray, List[Tuple[int, int]]]:
    """
    Generate a single simple backbone path from start to goal,
    allowed moves: up / down / left / right.

    Start: (1, size-1)
    Goal:  (size-2, 0)
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

    # Full DFS (explore entire grid)
    while stack:
        r, c = stack.pop()
        neigh = neighbors_4(r, c, size)
        rng.shuffle(neigh)
        for nr, nc in neigh:
            if not visited[nr, nc]:
                visited[nr, nc] = True
                parent[(nr, nc)] = (r, c)
                stack.append((nr, nc))

    if not visited[goal]:
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


# ------------------------------------------------------------
# Branch generation: use DFS between two backbone points
# ------------------------------------------------------------

def carve_branch_between_dfs(
    grid: np.ndarray,
    start: Tuple[int, int],
    target: Tuple[int, int],
    rng: np.random.Generator,
    max_expanded: int = 2000,
) -> bool:
    """
    Carve a branch path from 'start' to 'target' using a local DFS,
    similar in style to the backbone DFS:

      - DFS explores mostly in areas that are still walls (grid == 1)
        to create new corridor.
      - It is allowed to step into 'target' even if it's not a wall.
      - If DFS reaches target, we reconstruct the path via parent[]
        and carve it (set grid[r,c] = 0 along that path).

    max_expanded is a soft limit to avoid exploring the whole map
    if target is hard to reach using only walls.
    """
    size = grid.shape[0]
    sr, sc = start
    tr, tc = target

    if start == target:
        return False

    visited = set()
    parent: Dict[Tuple[int, int], Tuple[int, int]] = {}

    stack: List[Tuple[int, int]] = [start]
    visited.add(start)

    expanded = 0
    found = False

    while stack:
        r, c = stack.pop()
        expanded += 1
        if expanded > max_expanded:
            break

        if (r, c) == target:
            found = True
            break

        neigh = neighbors_4(r, c, size)
        rng.shuffle(neigh)

        moved = False
        for nr, nc in neigh:
            if (nr, nc) in visited:
                continue

            # Always allow stepping onto the target
            if (nr, nc) == target:
                parent[(nr, nc)] = (r, c)
                visited.add((nr, nc))
                stack.append((nr, nc))
                moved = True
                break

            # Otherwise, prefer to go into walls (grid == 1) so we create new tunnels
            if grid[nr, nc] == 1:
                parent[(nr, nc)] = (r, c)
                visited.add((nr, nc))
                stack.append((nr, nc))
                moved = True
                # we only push one neighbor at a time, DFS-style
                break

        if not moved:
            # dead end at this node, backtrack automatically by continuing the while loop
            continue

    if not found and target not in visited:
        return False

    # Reconstruct path from target back to start
    path_cells: List[Tuple[int, int]] = []
    cur = target
    while cur != start:
        path_cells.append(cur)
        cur = parent[cur]
    path_cells.append(start)
    path_cells.reverse()

    # Carve it
    for (r, c) in path_cells:
        grid[r, c] = 0

    return True


def add_random_branches_on_backbone(
    grid: np.ndarray,
    path: List[Tuple[int, int]],
    branch_count: int = 8,
    branch_seed: int = 123,
    min_index_gap: int = 8,
    max_expanded_per_branch: int = 2000,
) -> None:
    """
    Add 'branch_count' random DFS-style branches between different points
    on the backbone.

    For each branch:
      - Choose two indices i < j on the backbone path with j - i >= min_index_gap.
      - Run a local DFS from path[i] trying to reach path[j], exploring mostly
        walls (grid == 1).
      - If DFS reaches target, carve that DFS path.

    This creates cycles that look like the backbone: winding, non-straight.
    """
    rng = np.random.default_rng(branch_seed)
    n = len(path)
    if n < min_index_gap + 2 or branch_count <= 0:
        return

    for _ in range(branch_count):
        success = False
        for _attempt in range(100):
            # pick i not too close to start/goal
            i = int(rng.integers(low=1, high=n - 1 - min_index_gap))
            j_min = i + min_index_gap
            if j_min >= n - 1:
                continue
            j = int(rng.integers(low=j_min, high=n - 1))

            start = path[i]
            target = path[j]

            if carve_branch_between_dfs(
                grid,
                start,
                target,
                rng,
                max_expanded=max_expanded_per_branch,
            ):
                success = True
                break
        # if not success after many attempts, we just skip that branch


# ------------------------------------------------------------
# Combined generator
# ------------------------------------------------------------

def generate_multipath_random_map(
    size: int,
    backbone_seed: int = 42,
    branch_count: int = 8,
    branch_seed: int = 123,
    max_expanded_per_branch: int = 2000,
) -> Tuple[np.ndarray, List[Tuple[int, int]]]:
    """
    Generate a map with:
      - A random backbone path (DFS) from start to goal.
      - Additional DFS-based branches connecting different backbone points.

    Returns:
      grid : 2D array (1 = wall, 0 = path)
      path : the original backbone path
    """
    grid, path = generate_backbone_random_path(size=size, seed=backbone_seed)
    add_random_branches_on_backbone(
        grid,
        path,
        branch_count=branch_count,
        branch_seed=branch_seed,
        min_index_gap=8,
        max_expanded_per_branch=max_expanded_per_branch,
    )
    return grid, path


# ------------------------------------------------------------
# CLI
# ------------------------------------------------------------

if __name__ == "__main__":
    """
    Usage:
      python gen_multipath_random.py
          # size=64, backbone_seed=42, branch_count=8

      python gen_multipath_random.py 128
          # size=128, backbone_seed=42, branch_count=8

      python gen_multipath_random.py 128 42 12
          # size=128, backbone_seed=42, branch_count=12

      python gen_multipath_random.py 128 42 12 4000
          # size=128, backbone_seed=42, branches=12, allow larger DFS search
    """
    if len(sys.argv) >= 2:
        size = int(sys.argv[1])
    else:
        size = 64

    if len(sys.argv) >= 3:
        backbone_seed = int(sys.argv[2])
    else:
        backbone_seed = 42

    if len(sys.argv) >= 4:
        branch_count = int(sys.argv[3])
    else:
        branch_count = 8

    if len(sys.argv) >= 5:
        max_expanded_per_branch = int(sys.argv[4])
    else:
        max_expanded_per_branch = 2000

    grid, path = generate_multipath_random_map(
        size=size,
        backbone_seed=backbone_seed,
        branch_count=branch_count,
        branch_seed=backbone_seed + 1,
        max_expanded_per_branch=max_expanded_per_branch,
    )

    txt_name = f"cave_map_{size}.txt"
    png_name = f"cave_map_{size}.png"

    np.savetxt(txt_name, grid, fmt="%d")
    print(f"Saved map data to {txt_name}")
    print(f"Backbone path length = {len(path)} steps")

    visualize_map(grid, png_name)
