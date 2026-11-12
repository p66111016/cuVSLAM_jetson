#!/usr/bin/env python3
"""Render camera frames and recorded mono cuVSLAM poses into one MP4 video."""

import argparse
import csv
import json
import math
import re
from pathlib import Path

import cv2
import numpy as np


SIZE = (1280, 720)
WHITE = (245, 245, 245)
MUTED = (175, 175, 175)
GREEN = (80, 220, 100)
RED = (90, 90, 240)
BLUE = (235, 175, 60)
TRUTH = (0, 159, 230)


def natural_key(path):
    return [int(s) if s.isdigit() else s.lower() for s in re.split(r'(\d+)', path.name)]


def put(img, value, x, y, scale=0.58, color=WHITE):
    cv2.putText(img, value, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                color, 1, cv2.LINE_AA)


def euler_deg(qx, qy, qz, qw):
    # Intrinsic XYZ / roll-pitch-yaw in the cuVSLAM optical odometry frame.
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


def load_poses(path, first_stamp, interval_ns):
    poses = {}
    with path.open(newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            index = round((int(row['stamp_ns']) - first_stamp) / interval_ns)
            if index in poses:
                raise ValueError(f'duplicate pose at source frame {index}')
            poses[index] = tuple(float(row[name]) for name in
                                 ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'))
    if not poses or min(poses) < 0:
        raise ValueError('pose timestamps do not align with the source sequence')
    return poses


def make_plot(poses, truth=None):
    panel = np.full((510, 530, 3), (28, 32, 38), np.uint8)
    pad = 42
    positions = list(poses.values()) + list((truth or {}).values())
    xvals = [v[0] for v in positions]
    yvals = [v[1] for v in positions]
    cx = (min(xvals) + max(xvals)) / 2
    cy = (min(yvals) + max(yvals)) / 2
    span = max(max(xvals) - min(xvals), max(yvals) - min(yvals), 1.0) * 1.1
    scale = min((panel.shape[1] - 2 * pad) / span,
                (panel.shape[0] - 2 * pad) / span)

    def pixel(pose):
        return (round(panel.shape[1] / 2 + (pose[0] - cx) * scale),
                round(panel.shape[0] / 2 - (pose[1] - cy) * scale))

    for i in range(1, 5):
        x = round(i * panel.shape[1] / 5)
        y = round(i * panel.shape[0] / 5)
        cv2.line(panel, (x, 0), (x, panel.shape[0]), (48, 52, 58), 1)
        cv2.line(panel, (0, y), (panel.shape[1], y), (48, 52, 58), 1)
    if truth:
        previous = None
        for index, position in sorted(truth.items()):
            if previous is not None and index == previous[0] + 1:
                cv2.line(panel, pixel(previous[1]), pixel(position), TRUTH, 2, cv2.LINE_AA)
            previous = (index, position)
    return panel, pixel


def make_z_plot(poses, frame_count, truth=None):
    panel = np.full((125, 530, 3), (28, 32, 38), np.uint8)
    zs = [p[2] for p in list(poses.values()) + list((truth or {}).values())]
    lo, hi = min(zs), max(zs)
    span = max(hi - lo, 1.0)
    lo -= 0.05 * span
    hi += 0.05 * span

    def pixel(index, pose):
        x = round(34 + index / max(frame_count - 1, 1) * 480)
        y = round(8 + (hi - pose[2]) / (hi - lo) * 105)
        return x, y

    for i in range(1, 4):
        y = round(i * panel.shape[0] / 4)
        cv2.line(panel, (0, y), (panel.shape[1], y), (48, 52, 58), 1)
    put(panel, f'{hi:.1f}', 2, 15, 0.32, MUTED)
    put(panel, f'{lo:.1f}', 2, 120, 0.32, MUTED)
    if truth:
        previous = None
        for index, position in sorted(truth.items()):
            if previous is not None and index == previous[0] + 1:
                cv2.line(panel, pixel(*previous), pixel(index, position), TRUTH, 2, cv2.LINE_AA)
            previous = (index, position)
    return panel, pixel


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image-dir', type=Path, required=True)
    parser.add_argument('--start-image', required=True, help='First filename in the replay')
    parser.add_argument('--trajectory-csv', type=Path, required=True)
    parser.add_argument('--metrics-json', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--source-fps', type=float, default=30.0)
    parser.add_argument('--video-fps', type=float, default=30.0)
    parser.add_argument('--first-pose-frame', type=int, default=None,
                        help='For old metrics without first_ack_stamp_ns; zero-based replay index')
    parser.add_argument('--limit', type=int, default=0, help='Render only this many frames')
    parser.add_argument('--alignment-dir', type=Path,
                        help='Use trajectory_aligned.csv and ground_truth.csv from Sim3 alignment')
    args = parser.parse_args()
    if args.source_fps <= 0 or args.video_fps <= 0 or args.limit < 0:
        parser.error('FPS must be positive and limit cannot be negative')
    metrics = json.loads(args.metrics_json.read_text(encoding='utf-8'))
    files = sorted((p for p in args.image_dir.iterdir() if p.suffix.lower() in
                    ('.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff')), key=natural_key)
    start = next((i for i, p in enumerate(files) if p.name == args.start_image), None)
    if start is None:
        parser.error(f'{args.start_image} is not in {args.image_dir}')
    count = int(metrics['frames_acknowledged'])
    files = files[start:start + count]
    if len(files) != count:
        raise ValueError(f'found {len(files)} input images; metrics expects {count}')
    truth = None
    if args.alignment_dir:
        from align_trajectory_sim3 import read_csv
        alignment = json.loads((args.alignment_dir / 'alignment.json').read_text())
        if alignment['start_image'] != args.start_image or alignment['source_frames'] != count:
            raise ValueError('alignment is for a different replay range')
        poses = {int(r['frame_index']): tuple(float(r[k]) for k in
                 ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'))
                 for r in read_csv(args.alignment_dir / 'trajectory_aligned.csv')}
        truth = {int(r['frame_index']): tuple(float(r[k]) for k in ('x', 'y', 'z'))
                 for r in read_csv(args.alignment_dir / 'ground_truth.csv')}
        print('Sim3 synchronization: ' + alignment['synchronization'], flush=True)
    else:
        interval_ns = round(1_000_000_000 / args.source_fps)
        first_stamp = metrics.get('first_ack_stamp_ns')
        if first_stamp is None:
            if args.first_pose_frame is None or args.first_pose_frame < 0:
                parser.error('old metrics need --first-pose-frame for image/pose alignment')
            with args.trajectory_csv.open(newline='', encoding='utf-8') as stream:
                first_row = next(csv.DictReader(stream))
            first_stamp = int(first_row['stamp_ns']) - args.first_pose_frame * interval_ns
            print(f'Using assumed first pose at source frame {args.first_pose_frame}; '
                  'old metrics did not save the first image timestamp.', flush=True)
        poses = load_poses(args.trajectory_csv, int(first_stamp), interval_ns)
    if max(poses) >= count:
        raise ValueError(f'pose at frame {max(poses)} is outside {count} input images')
    base_plot, plot_point = make_plot(poses, truth)
    base_z_plot, z_point = make_z_plot(poses, count, truth)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(args.output), cv2.VideoWriter_fourcc(*'mp4v'),
                             args.video_fps, SIZE)
    if not writer.isOpened():
        raise RuntimeError(f'cannot open MP4 writer: {args.output}')
    last_pose = None
    last_pose_frame = None
    displayed = min(count, args.limit) if args.limit else count
    try:
        for index, path in enumerate(files[:displayed]):
            gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if gray is None:
                raise RuntimeError(f'cannot decode {path}')
            image = cv2.cvtColor(cv2.resize(gray, (680, 510)), cv2.COLOR_GRAY2BGR)
            canvas = np.full((SIZE[1], SIZE[0], 3), (17, 20, 25), np.uint8)
            canvas[38:548, 20:700] = image
            pose = poses.get(index)
            if pose is not None:
                point = plot_point(pose)
                if last_pose is not None and last_pose_frame == index - 1:
                    cv2.line(base_plot, plot_point(last_pose), point, BLUE, 2, cv2.LINE_AA)
                    cv2.line(base_z_plot, z_point(last_pose_frame, last_pose),
                             z_point(index, pose), BLUE, 2, cv2.LINE_AA)
                last_pose, last_pose_frame = pose, index
            plot = base_plot.copy()
            if truth:
                cv2.line(plot, (12, 18), (35, 18), TRUTH, 2)
                put(plot, 'Ground truth', 42, 23, 0.43)
                cv2.line(plot, (12, 40), (35, 40), BLUE, 2)
                put(plot, 'cuVSLAM (Sim3 aligned)', 42, 45, 0.43)
            if pose is not None:
                cv2.circle(plot, plot_point(pose), 5, GREEN, -1, cv2.LINE_AA)
            canvas[38:548, 730:1260] = plot
            z_plot = base_z_plot.copy()
            if pose is not None:
                cv2.circle(z_plot, z_point(index, pose), 4, GREEN, -1, cv2.LINE_AA)
            canvas[580:705, 730:1260] = z_plot
            put(canvas, 'CAMERA  ' + path.name, 20, 27)
            put(canvas, 'XY TRAJECTORY  (GT coordinates)' if truth else
                'XY TRAJECTORY  (optical frame, unscaled)', 730, 27, 0.53)
            put(canvas, 'Z PROFILE  (GT coordinates)' if truth else
                'Z PROFILE  (optical forward, unscaled)', 730, 570, 0.48)
            status = 'TRACKING' if pose is not None else 'NO POSE'
            put(canvas, f'Frame {index + 1:,} / {count:,}    {status}', 20, 585,
                0.68, GREEN if pose is not None else RED)
            if last_pose is not None:
                x, y, z, qx, qy, qz, qw = last_pose
                roll, pitch, yaw = euler_deg(qx, qy, qz, qw)
                put(canvas, f'X {x:10.3f}     Y {y:10.3f}     Z {z:10.3f}', 20, 623)
                put(canvas, f'Roll {roll:8.2f} deg   Pitch {pitch:8.2f} deg   '
                    f'Yaw {yaw:8.2f} deg', 20, 653)
                put(canvas, f'Quaternion  x {qx:.4f}  y {qy:.4f}  '
                    f'z {qz:.4f}  w {qw:.4f}', 20, 683, 0.52, MUTED)
                if pose is None:
                    put(canvas, f'Last valid pose: {index - last_pose_frame} frames ago',
                        20, 710, 0.48, RED)
            writer.write(canvas)
            if (index + 1) % 1000 == 0 or index + 1 == displayed:
                print(f'Rendered {index + 1:,}/{displayed:,}', flush=True)
    finally:
        writer.release()
    print(f'MP4: {args.output}', flush=True)


if __name__ == '__main__':
    main()
