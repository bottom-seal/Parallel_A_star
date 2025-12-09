import numpy as np
import matplotlib.pyplot as plt
import sys


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


def split_into_k_positive(total: int, k: int) -> list[int]:
    """
    Split 'total' into 'k' positive integers that sum to 'total', as evenly as possible.
    Example: total=15, k=4 -> [4, 4, 4, 3]
    """
    if total < k:
        raise ValueError(f"Cannot split total={total} into {k} positive parts.")

    base = total // k
    rem = total % k
    parts = []
    for i in range(k):
        length = base + (1 if i < rem else 0)
        parts.append(length)
    return parts


def generate_grouped_xy_path_map(
    size: int,
    num_subparts: int = 16,
    num_groups: int = 8,
    seed: int | None = 42,
) -> np.ndarray:
    """
    Generate a size x size map with:
      - 1 = wall, 0 = path
      - start: (1, size-1)  (top-right corner, 1 down)
      - goal:  (size-2, 0)  (bottom-left corner, 1 up)

    Logic:
      - Horizontal distance (total_left) -> split into num_subparts positive sub-parts.
      - Vertical distance (total_down) -> split into num_subparts positive sub-parts.
      - Randomly assign each sub-part to one of num_groups (0..num_groups-1).
        * A group can get 0, 1, or many sub-parts.
      - For each group i:
          * X_i = sum of all horizontal sub-parts assigned to group i
          * Y_i = sum of all vertical sub-parts assigned to group i
          * Move left X_i, then down Y_i (if 0, that move is skipped).
      - Total X_i = total_left, total Y_i = total_down, so path ends at goal.
    """
    if not is_power_of_two(size):
        raise ValueError("Size must be a power of 2 (32, 64, 128, ...).")
    if size < 32:
        raise ValueError("Size must be at least 32 (so 16 subparts fit nicely).")

    if num_groups > num_subparts:
        # groups can be empty; this is fine, we just don't require 1 per group
        pass

    start_r, start_c = 1, size - 1
    goal_r, goal_c = size - 2, 0

    total_down = goal_r - start_r   # steps downward
    total_left = start_c - goal_c   # steps left

    if total_left < num_subparts or total_down < num_subparts:
        raise ValueError(
            f"Size {size} too small for {num_subparts} subparts "
            f"(need total_left >= {num_subparts}, total_down >= {num_subparts})."
        )

    # 1) Split width & height into num_subparts positive sub-parts
    h_subparts = split_into_k_positive(total_left, num_subparts)
    v_subparts = split_into_k_positive(total_down, num_subparts)

    # 2) Randomly assign each sub-part index to a group (0..num_groups-1)
    rng = np.random.default_rng(seed)

    h_assign = rng.integers(low=0, high=num_groups, size=num_subparts)
    v_assign = rng.integers(low=0, high=num_groups, size=num_subparts)

    # 3) Sum sub-parts per group
    h_group_lengths = [0] * num_groups
    v_group_lengths = [0] * num_groups

    for idx, length in enumerate(h_subparts):
        g = int(h_assign[idx])
        h_group_lengths[g] += length

    for idx, length in enumerate(v_subparts):
        g = int(v_assign[idx])
        v_group_lengths[g] += length

    # Sanity: sums must match totals
    assert sum(h_group_lengths) == total_left
    assert sum(v_group_lengths) == total_down

    # 4) Carve the path
    grid = np.ones((size, size), dtype=int)

    r, c = start_r, start_c
    grid[r, c] = 0

    for gh, gv in zip(h_group_lengths, v_group_lengths):
        # X-part: move left
        for _ in range(gh):
            c -= 1
            grid[r, c] = 0

        # Y-part: move down
        for _ in range(gv):
            r += 1
            grid[r, c] = 0

    # End must be at goal
    assert r == goal_r and c == goal_c, f"Ended at ({r},{c}), expected ({goal_r},{goal_c})"

    grid[start_r, start_c] = 0
    grid[goal_r, goal_c] = 0

    return grid


if __name__ == "__main__":
    """
    Usage:
      python gen_simple_map.py                 # default size 64, seed 42
      python gen_simple_map.py 128             # size 128, seed 42
      python gen_simple_map.py 128 123         # size 128, seed 123 (different random layout)
    """
    if len(sys.argv) >= 2:
        size = int(sys.argv[1])
    else:
        size = 64

    if len(sys.argv) >= 3:
        seed = int(sys.argv[2])
    else:
        seed = 42  # fixed seed for reproducible maps

    grid = generate_grouped_xy_path_map(
        size,
        num_subparts=16,
        num_groups=8,
        seed=seed,
    )

    txt_name = f"simple_map_{size}.txt"
    png_name = f"simple_map_{size}.png"

    np.savetxt(txt_name, grid, fmt="%d")
    print(f"Saved map data to {txt_name}")

    visualize_map(grid, png_name)
