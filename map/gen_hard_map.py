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


def generate_hard_map(size: int, vertical_step: int = 4) -> np.ndarray:
    """
    Generate a 'hard' map with a long snake-like path:

      - 1 = wall, 0 = path
      - start: (1, size-1)  (top-right corner, 1 down)
      - goal:  (size-2, 0)  (bottom-left corner, 1 up)

    Pattern:
      - From the current row, move horizontally toward the opposite side,
        stopping when there is 1 column margin (col=1 or col=size-2).
      - Move down `vertical_step` rows.
      - Flip horizontal direction and repeat.
      - Near the bottom, do a final horizontal sweep **left to column 0**,
        then go straight down to the goal row.

    `vertical_step > 2` makes the snake sparser (fewer turns, longer vertical jumps).
    """
    if not is_power_of_two(size):
        raise ValueError("Size must be a power of 2 (32, 64, 128, ...).")
    if size < 8:
        raise ValueError("Use size >= 8 for this hard pattern to make sense.")
    if vertical_step < 1:
        raise ValueError("vertical_step must be >= 1")

    grid = np.ones((size, size), dtype=int)

    start_r, start_c = 1, size - 1
    goal_r, goal_c = size - 2, 0

    r, c = start_r, start_c
    grid[r, c] = 0

    direction = "L"  # current horizontal direction ("L" or "R")

    while True:
        remaining_rows = goal_r - r
        if remaining_rows <= 0:
            break  # safety

        if remaining_rows > vertical_step:
            # Regular snake segment: move to margin, then down vertical_step rows
            if direction == "L":
                target_col = 1
                while c > target_col:
                    c -= 1
                    grid[r, c] = 0
            else:  # direction == "R"
                target_col = size - 2
                while c < target_col:
                    c += 1
                    grid[r, c] = 0

            # Move down vertical_step rows
            for _ in range(vertical_step):
                r += 1
                grid[r, c] = 0

            # Flip direction for next horizontal run
            direction = "R" if direction == "L" else "L"

        else:
            # Final segment:
            # Always go left to column 0 (goal_c),
            # then go straight down to the goal row.
            while c > goal_c:
                c -= 1
                grid[r, c] = 0

            for _ in range(remaining_rows):
                r += 1
                grid[r, c] = 0
            break

    # Sanity: ensure we ended at the goal
    assert r == goal_r and c == goal_c, f"Ended at ({r},{c}), expected ({goal_r},{goal_c})"

    grid[start_r, start_c] = 0
    grid[goal_r, goal_c] = 0

    return grid


if __name__ == "__main__":
    """
    Usage:
      python gen_hard_map.py              # default size 64, vertical_step=4
      python gen_hard_map.py 128          # size 128, vertical_step=4
      python gen_hard_map.py 128 6        # size 128, vertical_step=6 (even sparser)
    """
    if len(sys.argv) >= 2:
        size = int(sys.argv[1])
    else:
        size = 64  # default

    if len(sys.argv) >= 3:
        vertical_step = int(sys.argv[2])
    else:
        vertical_step = 4

    grid = generate_hard_map(size, vertical_step=vertical_step)

    txt_name = f"hard_map_{size}.txt"
    png_name = f"hard_map_{size}.png"

    np.savetxt(txt_name, grid, fmt="%d")
    print(f"Saved hard map data to {txt_name}")

    visualize_map(grid, png_name)
