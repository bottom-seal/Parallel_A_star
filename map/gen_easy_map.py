import numpy as np
import matplotlib.pyplot as plt
import sys
from typing import List, Tuple, Set


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


# ------------------------------------------------------------
# Segment generators (backbone)
# ------------------------------------------------------------

def generate_vertical_segments(
    num_segments: int,
    N: int,
    start_r: int,
    goal_r: int,
    rng: np.random.Generator,
) -> List[int]:
    """
    Generate vertical segment deltas (positive = down, negative = up) so that
    we go from start_r to goal_r in num_segments segments.
    """
    segments: List[int] = []
    r = start_r

    for _ in range(num_segments - 1):
        max_step = max(1, (N - 2) // 4)
        mag = int(rng.integers(low=1, high=max_step + 1))

        # Bias overall direction towards goal row
        want_down = goal_r > r
        if want_down:
            dir_down_prob = 0.7
        else:
            dir_down_prob = 0.3

        if rng.random() < dir_down_prob:
            dir_ = +1  # down
        else:
            dir_ = -1  # up

        if dir_ == +1:
            max_down = (N - 2) - r
            if max_down <= 0:
                max_up = r - 1
                if max_up <= 0:
                    v = 0
                else:
                    mag = min(mag, max_up)
                    v = -mag
            else:
                mag = min(mag, max_down)
                v = +mag
        else:
            max_up = r - 1
            if max_up <= 0:
                max_down = (N - 2) - r
                if max_down <= 0:
                    v = 0
                else:
                    mag = min(mag, max_down)
                    v = +mag
            else:
                mag = min(mag, max_up)
                v = -mag

        r += v
        segments.append(v)

    v_last = goal_r - r
    segments.append(v_last)

    return segments


def generate_horizontal_segments(
    num_segments: int,
    N: int,
    start_c: int,
    goal_c: int,
    rng: np.random.Generator,
) -> List[int]:
    """
    Generate horizontal segment deltas (positive = right, negative = left) so that
    we go from start_c to goal_c, stay in [0, N-1], and include at least one
    right detour (off-optimal).
    """
    while True:
        segments: List[int] = []
        c = start_c
        has_right = False

        for _ in range(num_segments - 1):
            max_step = max(1, N // 4)
            mag = int(rng.integers(low=1, high=max_step + 1))

            # Bias overall leftwards toward goal
            want_left = goal_c < c
            if want_left:
                dir_left_prob = 0.7
            else:
                dir_left_prob = 0.3

            if rng.random() < dir_left_prob:
                dir_ = -1  # left
            else:
                dir_ = +1  # right

            if dir_ == -1:
                max_left = c
                if max_left <= 0:
                    max_right = (N - 1) - c
                    if max_right <= 0:
                        h = 0
                    else:
                        mag = min(mag, max_right)
                        h = +mag
                else:
                    mag = min(mag, max_left)
                    h = -mag
            else:
                max_right = (N - 1) - c
                if max_right <= 0:
                    max_left = c
                    if max_left <= 0:
                        h = 0
                    else:
                        mag = min(mag, max_left)
                        h = -mag
                else:
                    mag = min(mag, max_right)
                    h = +mag

            c += h
            segments.append(h)
            if h > 0:
                has_right = True

        h_last = goal_c - c
        segments.append(h_last)
        if h_last > 0:
            has_right = True

        # Check bounds quickly
        c_check = start_c
        ok = True
        for h in segments:
            step = 1 if h > 0 else -1
            for _ in range(abs(h)):
                c_check += step
                if not (0 <= c_check <= N - 1):
                    ok = False
                    break
            if not ok:
                break

        if ok and has_right:
            return segments
        # else retry


# ------------------------------------------------------------
# Backbone with self-avoid constraint
# ------------------------------------------------------------

def generate_easy_backbone(
    size: int,
    seed: int = 42,
    num_segments: int = 8,
    max_attempts: int = 500,
) -> Tuple[np.ndarray, List[Tuple[int, int]]]:
    """
    Generate a backbone path from start to goal with:

      - num_segments vertical blocks and num_segments horizontal blocks,
        interleaved: V, H, V, H, ...
      - Vertical segments can go up/down, horizontal left/right.
      - Direction changes always alternate axis (V ↔ H).
      - Path is strictly self-avoiding: no cell visited twice.
      - Start: (1, size-1), Goal: (size-2, 0).

    Retries up to max_attempts times if overlap happens.
    """
    if not is_power_of_two(size):
        raise ValueError("Size must be power of 2 (32, 64, 128, ...).")
    if size < 8:
        raise ValueError("Use size >= 8 to make sense.")

    N = size
    start = (1, N - 1)
    goal = (N - 2, 0)

    rng = np.random.default_rng(seed)

    for _attempt in range(max_attempts):
        # generate segment lengths
        v_segments = generate_vertical_segments(
            num_segments=num_segments,
            N=N,
            start_r=start[0],
            goal_r=goal[0],
            rng=rng,
        )
        h_segments = generate_horizontal_segments(
            num_segments=num_segments,
            N=N,
            start_c=start[1],
            goal_c=goal[1],
            rng=rng,
        )

        # carve while enforcing no overlap
        grid = np.ones((N, N), dtype=int)
        path: List[Tuple[int, int]] = []
        visited: Set[Tuple[int, int]] = set()

        r, c = start
        grid[r, c] = 0
        path.append((r, c))
        visited.add((r, c))

        def step_move(dr: int, dc: int) -> bool:
            """Move one step; return False if out-of-bounds or overlap."""
            nonlocal r, c
            nr, nc = r + dr, c + dc
            if not (0 <= nr < N and 0 <= nc < N):
                return False
            if (nr, nc) in visited:
                return False
            r, c = nr, nc
            grid[r, c] = 0
            path.append((r, c))
            visited.add((r, c))
            return True

        ok = True

        # interleave vertical and horizontal
        for v, h in zip(v_segments, h_segments):
            # vertical
            if v != 0:
                v_step = 1 if v > 0 else -1
                for _ in range(abs(v)):
                    if not step_move(v_step, 0):
                        ok = False
                        break
                if not ok:
                    break

            # horizontal
            if h != 0 and ok:
                h_step = 1 if h > 0 else -1
                for _ in range(abs(h)):
                    if not step_move(0, h_step):
                        ok = False
                        break
            if not ok:
                break

        # check final position & success
        if ok and (r, c) == goal:
            # ensure endpoints open
            grid[start[0], start[1]] = 0
            grid[goal[0], goal[1]] = 0
            return grid, path

        # else retry with new random segments

    raise RuntimeError(f"Failed to generate self-avoiding backbone after {max_attempts} attempts.")


# ------------------------------------------------------------
# Branches: segment-based, connecting to existing open cells
# ------------------------------------------------------------

def carve_connecting_branch(
    grid: np.ndarray,
    open_set: Set[Tuple[int, int]],
    start_r: int,
    start_c: int,
    rng: np.random.Generator,
    max_segments: int,
    min_seg_len: int,
    max_seg_len: int,
    max_attempts: int = 20,
) -> bool:
    """
    Try to carve a branch that:

      - Starts at (start_r, start_c) (which is already open).
      - Uses up to max_segments segments.
      - Each segment has length in [min_seg_len, max_seg_len].
      - Segments alternate axis: V, H, V, H, ...
      - During growth, stays entirely in walls (not in open_set).
      - At the end, the tip is adjacent to some *other* open cell
        (in open_set, not equal to the start), i.e., the branch
        connects two open regions.

    If successful, commits the carved cells into grid and open_set.
    Returns True if a connecting branch was created, False otherwise.
    """
    N = grid.shape[0]

    for _ in range(max_attempts):
        r, c = start_r, start_c
        last_axis = None  # "v" or "h"
        new_cells: List[Tuple[int, int]] = []

        for seg_idx in range(max_segments):
            # choose axis (alternate with previous)
            if last_axis is None:
                axis = "v" if rng.random() < 0.5 else "h"
            else:
                axis = "h" if last_axis == "v" else "v"

            # candidate directions for this axis
            if axis == "v":
                directions = [(1, 0), (-1, 0)]  # down, up
            else:
                directions = [(0, 1), (0, -1)]  # right, left

            rng.shuffle(directions)

            seg_done = False
            for dr, dc in directions:
                # find maximum allowed steps in this direction:
                #   - within bounds
                #   - not stepping into any existing open cell
                #     (open_set), nor into new_cells
                Lmax = 0
                nr, nc = r, c
                while True:
                    nr += dr
                    nc += dc
                    if not (0 <= nr < N and 0 <= nc < N):
                        break
                    if (nr, nc) in open_set or (nr, nc) in new_cells:
                        break
                    Lmax += 1
                    if Lmax >= max_seg_len:
                        break

                if Lmax < min_seg_len:
                    continue

                # choose actual length
                length = int(rng.integers(low=min_seg_len, high=Lmax + 1))

                nr, nc = r, c
                seg_cells: List[Tuple[int, int]] = []
                for _ in range(length):
                    nr += dr
                    nc += dc
                    seg_cells.append((nr, nc))

                # apply this segment
                r, c = nr, nc
                new_cells.extend(seg_cells)
                last_axis = axis
                seg_done = True
                break  # segment carved

            if not seg_done:
                # no feasible direction for this segment → stop branch
                break

        if not new_cells:
            # nothing carved in this attempt
            continue

        # Check if tip connects to another open cell (not the start).
        r_end, c_end = r, c
        neighbors = []
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r_end + dr, c_end + dc
            if (nr, nc) in open_set and (nr, nc) != (start_r, start_c):
                neighbors.append((nr, nc))

        if not neighbors:
            # this branch doesn't connect → retry a new attempt
            continue

        # We found at least one neighbor, so commit new cells.
        for (rr, cc) in new_cells:
            grid[rr, cc] = 0
            open_set.add((rr, cc))

        # connection neighbor is already open; no need to carve it
        return True

    return False


def add_easy_branches(
    grid: np.ndarray,
    backbone_path: List[Tuple[int, int]],
    branch_count: int = 4,
    max_seg_len: int = 12,
    seed: int = 123,
) -> None:
    """
    Add branches that mimic the backbone style and try to connect regions:

      - use 2–4 segments per branch
      - each segment has several cells
      - alternate axis V/H
      - most branches end adjacent to some other open cell (so they connect
        two parts of the graph, not just dead-ends)
    """
    if branch_count <= 0 or max_seg_len <= 0:
        return

    rng = np.random.default_rng(seed)
    N = grid.shape[0]

    if len(backbone_path) <= 2:
        return

    # initial open set includes all currently open cells (backbone)
    open_cells = np.argwhere(grid == 0)
    open_set: Set[Tuple[int, int]] = set(map(tuple, open_cells))

    max_segments = 4
    min_seg_len = max(2, max_seg_len // 3)
    max_seg_len = max(min_seg_len + 1, max_seg_len)

    successes = 0
    attempts = 0
    max_total_attempts = branch_count * 3  # allow some failures

    while successes < branch_count and attempts < max_total_attempts:
        attempts += 1
        # random backbone index, avoid endpoints
        start_idx = int(rng.integers(low=1, high=len(backbone_path) - 1))
        start_r, start_c = backbone_path[start_idx]

        ok = carve_connecting_branch(
            grid,
            open_set,
            start_r,
            start_c,
            rng,
            max_segments=max_segments,
            min_seg_len=min_seg_len,
            max_seg_len=max_seg_len,
            max_attempts=20,
        )

        if ok:
            successes += 1


# ------------------------------------------------------------
# Wrapper: backbone + branches
# ------------------------------------------------------------

def generate_easy_map_with_branches(
    size: int,
    backbone_seed: int = 42,
    num_segments: int = 8,
    branch_seed: int = 123,
    branch_count: int = 4,
    max_branch_seg_len: int = 12,
) -> np.ndarray:
    """
    Generate an "easy" map:

      - Self-avoiding backbone from (1, N-1) to (N-2, 0),
        alternating vertical/horizontal segments.
      - Branches that also use segment-based carving (few turns, long-ish
        segments) and usually connect two open regions.
    """
    grid, backbone_path = generate_easy_backbone(
        size=size,
        seed=backbone_seed,
        num_segments=num_segments,
    )

    add_easy_branches(
        grid,
        backbone_path,
        branch_count=branch_count,
        max_seg_len=max_branch_seg_len,
        seed=branch_seed,
    )

    return grid


# ------------------------------------------------------------
# CLI
# ------------------------------------------------------------

if __name__ == "__main__":
    """
    Usage:
      python gen_easy_backbone.py
          # size=64, backbone_seed=42, seg=8, 4 branches

      python gen_easy_backbone.py 128
          # size=128, same params

      python gen_easy_backbone.py 128 42 8 6 16
          # size=128, backbone_seed=42, num_segments=8,
          # branch_count=6, branch branch-segment length up to 16

      python gen_easy_backbone.py 128 42 12 8 20
          # more backbone segments, more branches, longer branch segments
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
        num_segments = int(sys.argv[3])
    else:
        num_segments = 8

    if len(sys.argv) >= 5:
        branch_count = int(sys.argv[4])
    else:
        branch_count = 4

    if len(sys.argv) >= 6:
        max_branch_seg_len = int(sys.argv[5])
    else:
        max_branch_seg_len = 12

    grid = generate_easy_map_with_branches(
        size=size,
        backbone_seed=backbone_seed,
        num_segments=num_segments,
        branch_seed=backbone_seed + 1,
        branch_count=branch_count,
        max_branch_seg_len=max_branch_seg_len,
    )

    txt_name = (
        f"easy_map_{size}.txt"
    )
    png_name = (
        f"easy_map_{size}.png"
    )

    np.savetxt(txt_name, grid, fmt="%d")
    print(f"Saved easy map with branches to {txt_name}")

    visualize_map(grid, png_name)
