import argparse
import numpy as np
import matplotlib.pyplot as plt


def compute_rbf_kernel(distance_matrix: np.ndarray, gamma: float) -> np.ndarray:
    """
    Compute RBF (Gaussian) kernel values given pairwise distances.

    K(r) = exp(-gamma * r^2)

    Args:
        distance_matrix: Array of distances r.
        gamma: Positive scalar controlling the kernel width (gamma = 1 / (2 * length_scale^2)).

    Returns:
        Array of kernel values with the same shape as distance_matrix.
    """
    if gamma <= 0:
        raise ValueError("gamma must be positive")
    return np.exp(-gamma * np.square(distance_matrix))


def generate_1d_plot(max_distance: float, gammas: list[float], num_points: int = 400):
    """
    Plot K(r) versus distance r for multiple gamma values.
    """
    distances = np.linspace(-max_distance, max_distance, num_points)

    plt.subplot(1, 2, 1)
    for gamma in gammas:
        kernel_values = compute_rbf_kernel(distances, gamma)
        label = f"gamma={gamma:g} (ell={np.sqrt(1.0/(2.0*gamma)):.2f})"
        plt.plot(distances, kernel_values, label=label, linewidth=2)

    plt.title("RBF Kernel vs Distance")
    plt.xlabel("distance r")
    plt.ylabel("K(r)")
    plt.ylim(-0.05, 1.05)
    plt.grid(True, alpha=0.3)
    plt.legend(frameon=False)


def generate_2d_heatmap(grid_extent: float, gamma: float, num_points: int = 200):
    """
    Plot a 2D heatmap of the RBF kernel centered at the origin.

    Shows K(||x - c||) where c = (0, 0).
    """
    axis_vals = np.linspace(-grid_extent, grid_extent, num_points)
    xs, ys = np.meshgrid(axis_vals, axis_vals)
    distances = np.sqrt(xs**2 + ys**2)
    kernel_values = compute_rbf_kernel(distances, gamma)

    plt.subplot(1, 2, 2)
    im = plt.imshow(
        kernel_values,
        extent=[-grid_extent, grid_extent, -grid_extent, grid_extent],
        origin="lower",
        cmap="viridis",
        vmin=0.0,
        vmax=1.0,
        interpolation="bilinear",
    )
    plt.title(f"2D RBF Heatmap (gamma={gamma:g})")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.colorbar(im, fraction=0.046, pad=0.04, label="K(||x||)")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize RBF (Gaussian) kernel")
    parser.add_argument(
        "--gammas",
        type=float,
        nargs="+",
        default=[0.5, 1.0, 2.0],
        help="List of gamma values for 1D curves (default: 0.5 1.0 2.0)",
    )
    parser.add_argument(
        "--max-distance",
        type=float,
        default=3.0,
        help="Half-range of r for 1D plot; r in [-max, +max] (default: 3.0)",
    )
    parser.add_argument(
        "--grid-extent",
        type=float,
        default=3.0,
        help="Half-width of the square grid for 2D heatmap (default: 3.0)",
    )
    parser.add_argument(
        "--heatmap-gamma",
        type=float,
        default=1.0,
        help="Gamma value for 2D heatmap (default: 1.0)",
    )
    parser.add_argument(
        "--points-1d",
        type=int,
        default=400,
        help="Number of sample points for 1D plot (default: 400)",
    )
    parser.add_argument(
        "--points-2d",
        type=int,
        default=200,
        help="Number of sample points per axis for 2D heatmap (default: 200)",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    plt.figure(figsize=(12, 5))
    generate_1d_plot(
        max_distance=args.max_distance,
        gammas=args.gammas,
        num_points=args.points_1d,
    )
    generate_2d_heatmap(
        grid_extent=args.grid_extent,
        gamma=args.heatmap_gamma,
        num_points=args.points_2d,
    )
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()

