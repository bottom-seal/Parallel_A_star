import numpy as np
import matplotlib.pyplot as plt
import sys
from pathlib import Path


def is_power_of_two(n: int) -> bool:
    return n > 0 and (n & (n - 1)) == 0


def visualize_map(grid: np.ndarray, out_path: str = "map.png") -> None:
    """
    Visualize a 0/1 map (square: 32x32, 64x64, 128x128, ...) and save as PNG.

    1 = wall (black)
    0 = path (white)
    """
    grid = np.array(grid, dtype=int)

    # basic checks
    if grid.ndim != 2:
        raise ValueError("Map must be 2D (H x W).")

    h, w = grid.shape
    if h != w:
        raise ValueError(f"Map must be square, got {h}x{w}.")
    if not is_power_of_two(h):
        raise ValueError(f"Map size must be power of 2 (32, 64, 128, ...). Got {h}.")

    if not np.all(np.isin(grid, [0, 1])):
        raise ValueError("Map must contain only 0 and 1.")

    # 1 -> wall (black / 0), 0 -> path (white / 255)
    img = np.where(grid == 1, 0, 255).astype(np.uint8)

    plt.figure(figsize=(4, 4))
    plt.imshow(img, cmap="gray", interpolation="nearest", vmin=0, vmax=255)
    plt.axis("off")
    plt.tight_layout(pad=0)

    plt.savefig(out_path, dpi=200, bbox_inches="tight", pad_inches=0)
    plt.close()
    print(f"Saved map image to {out_path}")


def load_map_from_txt(path: str) -> np.ndarray:
    """
    Load a map from a text file with 0/1 entries.
    Each row in the file should be space-separated or no spaces, e.g.:

    0 1 0 0 1
    1 1 0 0 0
    ...
    """
    return np.loadtxt(path, dtype=int)


def generate_random_map(size: int = 64, wall_prob: float = 0.3) -> np.ndarray:
    """
    Generate a random size×size map with 1 = wall, 0 = path.
    wall_prob controls density of walls.
    """
    if not is_power_of_two(size):
        raise ValueError("Size must be power of 2 (32, 64, 128, ...).")

    rng = np.random.default_rng()
    grid = (rng.random((size, size)) < wall_prob).astype(int)
    return grid


if __name__ == "__main__":
    """
    Usage:
      1) Demo (random map):
         python visualize_map.py

      2) Visualize from existing text map:
         python visualize_map.py map.txt output.png
    """
    if len(sys.argv) == 1:
        # Demo: generate a random 64x64 map and visualize it
        size = 64
        grid = generate_random_map(size=size, wall_prob=0.3)

        # Save map to text too, so you can reuse it in C/C++ later
        np.savetxt("map.txt", grid, fmt="%d")
        print(f"Saved random map to map.txt ({size}x{size})")

        visualize_map(grid, "map.png")

    elif len(sys.argv) >= 2:
        in_path = sys.argv[1]
        out_path = sys.argv[2] if len(sys.argv) >= 3 else "map.png"

        if not Path(in_path).is_file():
            print(f"Error: {in_path} does not exist.", file=sys.stderr)
            sys.exit(1)

        grid = load_map_from_txt(in_path)
        visualize_map(grid, out_path)

    else:
        print("Usage: python visualize_map.py [map.txt [out.png]]")
