#!/usr/bin/env python3
"""Run offline cuVSLAM, then align, plot and render against labelled ground truth."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from render_trajectory_video import natural_key


def main():
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image-dir', type=Path, default=Path('/home/user/2026-06-22VTOL/firefly'))
    parser.add_argument('--ground-truth', type=Path, default=Path('/home/user/2026-06-22VTOL/EO_opk_new.csv'))
    parser.add_argument('--start-image', default='firefly_008011.png')
    parser.add_argument('--output-dir', type=Path, required=True, help='Must be a new directory')
    parser.add_argument('--max-frames', type=int, default=0)
    parser.add_argument('--source-fps', type=float, default=30.0)
    parser.add_argument('--calibration-file', type=Path,
                        default=root / 'config/firefly_20260622_assumed.yaml')
    parser.add_argument('--skip-video', action='store_true')
    args = parser.parse_args()
    if args.max_frames < 0 or args.source_fps <= 0:
        parser.error('invalid max frames or FPS')
    files = sorted(args.image_dir.glob('firefly_*.png'), key=natural_key)
    start = next((i for i, p in enumerate(files) if p.name == args.start_image), None)
    if start is None or not args.ground_truth.is_file():
        parser.error('start image or ground-truth CSV not found')
    count = min(args.max_frames or len(files), len(files) - start)
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    metrics, trajectory = output / 'metrics.json', output / 'trajectory.csv'
    command = ['ros2', 'launch', 'mono_cuvslam', 'stage4_offline.launch.py',
               f'image_dir:={args.image_dir.resolve()}', f'start_index:={start}',
               f'max_frames:={count}', f'source_fps:={args.source_fps}',
               f'calibration_file:={args.calibration_file.resolve()}',
               'use_rviz:=false', 'path_history_poses:=100',
               f'metrics_output:={metrics}', f'trajectory_output:={trajectory}']
    process = subprocess.Popen(command, start_new_session=True)
    try:
        while True:
            if metrics.exists():
                try:
                    report = json.loads(metrics.read_text())
                    break
                except json.JSONDecodeError:
                    pass
            if process.poll() is not None:
                raise RuntimeError('ROS launch exited before producing metrics')
            time.sleep(0.5)
        if not report.get('complete'):
            raise RuntimeError('tracking run is incomplete')
        time.sleep(1.0)  # Let the metrics process flush/close its trajectory CSV.
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
    comparison = output / 'comparison'
    scripts = root / 'scripts'
    subprocess.run([sys.executable, str(scripts / 'align_trajectory_sim3.py'),
                    '--image-dir', str(args.image_dir.resolve()),
                    '--start-image', args.start_image, '--source-fps', str(args.source_fps),
                    '--trajectory-csv', str(trajectory), '--metrics-json', str(metrics),
                    '--ground-truth', str(args.ground_truth.resolve()),
                    '--output-dir', str(comparison)], check=True)
    subprocess.run([sys.executable, str(scripts / 'export_run_report.py'),
                    '--run-dir', str(output)], check=True)
    subprocess.run([sys.executable, str(scripts / 'plot_trajectory.py'),
                    str(comparison / 'trajectory_aligned.csv')], check=True)
    if not args.skip_video:
        subprocess.run([sys.executable, str(scripts / 'render_trajectory_video.py'),
                        '--image-dir', str(args.image_dir.resolve()),
                        '--start-image', args.start_image, '--trajectory-csv', str(trajectory),
                        '--metrics-json', str(metrics), '--alignment-dir', str(comparison),
                        '--output', str(comparison / 'camera_trajectory_comparison.mp4')], check=True)
    print(f'Results: {comparison}', flush=True)


if __name__ == '__main__':
    main()
