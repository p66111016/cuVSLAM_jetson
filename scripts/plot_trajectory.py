#!/usr/bin/env python3
"""Plot a recorded mono cuVSLAM trajectory in its optical odometry frame."""

import argparse
import csv
from pathlib import Path
from statistics import median

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt


def plot_aligned(path, truth_path, output_dir):
    from align_trajectory_sim3 import read_csv
    rows, truth = read_csv(path), read_csv(truth_path)
    if not rows or not truth:
        raise ValueError('aligned trajectory and truth must not be empty')
    origin = [float(truth[0][k]) for k in ('x', 'y', 'z')]
    cuts = [0] + [i for i in range(1, len(rows))
                  if int(rows[i]['frame_index']) != int(rows[i-1]['frame_index']) + 1] + [len(rows)]
    for plane in ('xy', 'z'):
        fig, ax = plt.subplots(figsize=(9, 7) if plane == 'xy' else (11, 4),
                               constrained_layout=True)
        def coordinates(series):
            if plane == 'xy':
                return ([float(r['x']) - origin[0] for r in series],
                        [float(r['y']) - origin[1] for r in series])
            return ([int(r['frame_index']) + 1 for r in series],
                    [float(r['z']) for r in series])
        ax.plot(*coordinates(truth), color='#e69f00', linewidth=1.5, label='Ground truth', zorder=1)
        for j, (a, b) in enumerate(zip(cuts, cuts[1:])):
            ax.plot(*coordinates(rows[a:b]), color='#1769aa', linewidth=1.0,
                    label='cuVSLAM (global Sim3 aligned)' if j == 0 else None, zorder=2)
        ax.set_title('Ground truth vs cuVSLAM: ' + ('XY trajectory' if plane == 'xy' else 'Z profile'))
        if plane == 'xy':
            ax.set(xlabel='X relative to first ground-truth position (GT units)',
                   ylabel='Y relative to first ground-truth position (GT units)')
            ax.set_aspect('equal', adjustable='datalim')
        else:
            ax.set(xlabel='Source frame number in replay', ylabel='Z (GT coordinate units)')
        ax.grid(alpha=0.3)
        ax.legend()
        fig.savefig(output_dir / f'trajectory_{plane}_comparison.png', dpi=180)
        plt.close(fig)
    print(f'Comparison plots: {output_dir}')


def load_trajectory(path):
    stamps, xs, ys, zs = [], [], [], []
    with path.open(newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            stamps.append(int(row['stamp_ns']))
            xs.append(float(row['x']))
            ys.append(float(row['y']))
            zs.append(float(row['z']))
    if not stamps:
        raise ValueError(f'No odometry samples in {path}')
    return stamps, xs, ys, zs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectory_csv', type=Path)
    parser.add_argument('--output-dir', type=Path, default=None)
    parser.add_argument('--ground-truth-csv', type=Path, default=None)
    args = parser.parse_args()
    output_dir = args.output_dir or args.trajectory_csv.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    with args.trajectory_csv.open(newline='', encoding='utf-8') as stream:
        aligned = 'frame_index' in (csv.DictReader(stream).fieldnames or [])
    if aligned:
        plot_aligned(args.trajectory_csv, args.ground_truth_csv or
                     args.trajectory_csv.parent / 'ground_truth.csv', output_dir)
        return
    stamps, xs, ys, zs = load_trajectory(args.trajectory_csv)
    intervals = [b - a for a, b in zip(stamps, stamps[1:]) if b > a]
    nominal_interval = median(intervals) if intervals else 1
    breaks = [i for i in range(1, len(stamps))
              if stamps[i] - stamps[i - 1] > 1.5 * nominal_interval]
    segment_starts = [0, *breaks]
    segment_ends = [*breaks, len(stamps)]
    frames = [(stamp - stamps[0]) / nominal_interval + 1 for stamp in stamps]

    fig, ax = plt.subplots(figsize=(8, 7), constrained_layout=True)
    for start, end in zip(segment_starts, segment_ends):
        ax.plot(xs[start:end], ys[start:end], linewidth=0.8, color='#1769aa')
    ax.scatter(xs[0], ys[0], color='#2ca02c', s=36, label='Start', zorder=3)
    ax.scatter(xs[-1], ys[-1], color='#d62728', s=36, label='End', zorder=3)
    ax.set(xlabel='X (optical frame; unscaled)',
           ylabel='Y (optical frame; unscaled)',
           title=f'Mono cuVSLAM XY trajectory ({len(xs):,} poses)')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(alpha=0.3)
    ax.legend()
    xy_path = output_dir / 'trajectory_xy.png'
    fig.savefig(xy_path, dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4), constrained_layout=True)
    for start, end in zip(segment_starts, segment_ends):
        ax.plot(frames[start:end], zs[start:end], linewidth=0.8, color='#7b3294')
    ax.set(xlabel='Source frame index from first tracked pose',
           ylabel='Z (optical forward; unscaled)',
           title=f'Mono cuVSLAM Z profile ({len(zs):,} poses)')
    ax.grid(alpha=0.3)
    z_path = output_dir / 'trajectory_z.png'
    fig.savefig(z_path, dpi=180)
    plt.close(fig)
    print(f'Poses: {len(xs)}')
    print(f'Trajectory segments (timestamp gaps): {len(segment_starts)}')
    print(f'XY plot: {xy_path}')
    print(f'Z plot: {z_path}')
    print('Note: optical Z points forward, not vertically; mono translation has no absolute scale.')


if __name__ == '__main__':
    main()
