#!/usr/bin/env python3
"""
Path Comparison Script
Compares planned path with actual trajectory using evo and custom metrics.

Features:
- Reads TUM format trajectory files
- Calculates RMSE, Max Error, Mean Error
- Generates comparison plots
- Optionally calls evo for APE/RPE analysis

Usage:
    python3 path_comparison.py --planned planned.tum --actual actual.tum
    python3 path_comparison.py --planned planned.tum --actual actual.tum --evo
"""

import argparse
import os
import subprocess
import sys
from typing import List, Tuple, Optional

import numpy as np

try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("Warning: matplotlib not found. Plotting disabled.")


def load_tum_trajectory(filepath: str) -> Tuple[np.ndarray, np.ndarray]:
    """
    Load trajectory from TUM format file.
    
    Returns:
        timestamps: (N,) array of timestamps
        positions: (N, 3) array of [x, y, z] positions
    """
    data = np.loadtxt(filepath)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    
    timestamps = data[:, 0]
    positions = data[:, 1:4]  # x, y, z
    
    return timestamps, positions


def compute_distance_errors(
    planned_pos: np.ndarray,
    actual_pos: np.ndarray
) -> Tuple[np.ndarray, float, float, float]:
    """
    Compute point-to-trajectory distance errors.
    For each actual point, find the closest point on the planned path.
    
    Returns:
        errors: Array of per-point errors
        rmse: Root Mean Square Error
        max_error: Maximum error
        mean_error: Mean error
    """
    errors = []
    
    for actual_pt in actual_pos:
        # Find minimum distance to any planned point
        distances = np.linalg.norm(planned_pos[:, :2] - actual_pt[:2], axis=1)
        min_dist = np.min(distances)
        errors.append(min_dist)
    
    errors = np.array(errors)
    rmse = np.sqrt(np.mean(errors ** 2))
    max_error = np.max(errors)
    mean_error = np.mean(errors)
    
    return errors, rmse, max_error, mean_error


def plot_trajectories(
    planned_pos: np.ndarray,
    actual_pos: np.ndarray,
    errors: np.ndarray,
    output_path: Optional[str] = None
):
    """Plot planned vs actual trajectories with error visualization."""
    if not HAS_MATPLOTLIB:
        return
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    
    # Trajectory comparison plot
    ax1 = axes[0]
    ax1.plot(planned_pos[:, 0], planned_pos[:, 1], 'b-', linewidth=2, label='Planned Path')
    ax1.plot(actual_pos[:, 0], actual_pos[:, 1], 'r-', linewidth=2, label='Actual Trajectory')
    ax1.scatter(planned_pos[0, 0], planned_pos[0, 1], c='green', s=100, marker='o', label='Start', zorder=5)
    ax1.scatter(planned_pos[-1, 0], planned_pos[-1, 1], c='purple', s=100, marker='x', label='End', zorder=5)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Trajectory Comparison')
    ax1.legend()
    ax1.axis('equal')
    ax1.grid(True, alpha=0.3)
    
    # Error plot
    ax2 = axes[1]
    ax2.plot(errors, 'r-', linewidth=1)
    ax2.axhline(y=np.mean(errors), color='b', linestyle='--', label=f'Mean: {np.mean(errors):.4f} m')
    ax2.fill_between(range(len(errors)), 0, errors, alpha=0.3, color='red')
    ax2.set_xlabel('Sample Index')
    ax2.set_ylabel('Lateral Error (m)')
    ax2.set_title('Tracking Error')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Plot saved to: {output_path}")
    else:
        plt.show()


def run_evo_analysis(planned_path: str, actual_path: str):
    """Run evo APE and RPE analysis."""
    print("\n" + "="*60)
    print("Running evo APE (Absolute Pose Error) analysis...")
    print("="*60)
    
    try:
        subprocess.run([
            'evo_ape', 'tum', planned_path, actual_path,
            '-v', '-a', '--plot'
        ], check=True)
    except FileNotFoundError:
        print("Error: evo not found. Install with: pip3 install evo")
        return
    except subprocess.CalledProcessError as e:
        print(f"evo_ape failed: {e}")
    
    print("\n" + "="*60)
    print("Running evo RPE (Relative Pose Error) analysis...")
    print("="*60)
    
    try:
        subprocess.run([
            'evo_rpe', 'tum', planned_path, actual_path,
            '-v', '-a', '--plot'
        ], check=True)
    except subprocess.CalledProcessError as e:
        print(f"evo_rpe failed: {e}")


def main():
    parser = argparse.ArgumentParser(description='Compare planned path with actual trajectory')
    parser.add_argument('--planned', '-p', required=True, help='Planned path TUM file')
    parser.add_argument('--actual', '-a', required=True, help='Actual trajectory TUM file')
    parser.add_argument('--output', '-o', help='Output plot file path')
    parser.add_argument('--evo', action='store_true', help='Run evo APE/RPE analysis')
    parser.add_argument('--no-plot', action='store_true', help='Disable plotting')
    
    args = parser.parse_args()
    
    # Validate input files
    if not os.path.exists(args.planned):
        print(f"Error: Planned path file not found: {args.planned}")
        sys.exit(1)
    if not os.path.exists(args.actual):
        print(f"Error: Actual trajectory file not found: {args.actual}")
        sys.exit(1)
    
    # Load trajectories
    print(f"Loading planned path: {args.planned}")
    planned_ts, planned_pos = load_tum_trajectory(args.planned)
    print(f"  Points: {len(planned_ts)}")
    
    print(f"Loading actual trajectory: {args.actual}")
    actual_ts, actual_pos = load_tum_trajectory(args.actual)
    print(f"  Points: {len(actual_ts)}")
    
    # Compute errors
    print("\nComputing tracking errors...")
    errors, rmse, max_error, mean_error = compute_distance_errors(planned_pos, actual_pos)
    
    # Print results
    print("\n" + "="*60)
    print("  TRAJECTORY COMPARISON RESULTS")
    print("="*60)
    print(f"  Planned Path Points:   {len(planned_pos)}")
    print(f"  Actual Trajectory Points: {len(actual_pos)}")
    print("-"*60)
    print(f"  RMSE (Root Mean Square Error):  {rmse:.4f} m")
    print(f"  Max Error:                      {max_error:.4f} m")
    print(f"  Mean Error:                     {mean_error:.4f} m")
    print(f"  Std Dev:                        {np.std(errors):.4f} m")
    print("="*60)
    
    # Quality assessment
    if rmse < 0.05:
        quality = "EXCELLENT (RMSE < 5cm)"
    elif rmse < 0.10:
        quality = "GOOD (RMSE < 10cm)"
    elif rmse < 0.20:
        quality = "ACCEPTABLE (RMSE < 20cm)"
    else:
        quality = "POOR (RMSE >= 20cm)"
    print(f"  Quality: {quality}")
    print("="*60 + "\n")
    
    # Plot
    if not args.no_plot:
        plot_trajectories(planned_pos, actual_pos, errors, args.output)
    
    # Run evo analysis
    if args.evo:
        run_evo_analysis(args.planned, args.actual)


if __name__ == '__main__':
    main()
