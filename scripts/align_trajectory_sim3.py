#!/usr/bin/env python3
"""Fit one global, positive-scale Sim(3) from cuVSLAM positions to labelled truth."""
import argparse
import csv
import json
import re
from pathlib import Path

import numpy as np


def fit_sim3(source, target):
    source, target = np.asarray(source, dtype=float), np.asarray(target, dtype=float)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError('source and target must be matching Nx3 arrays')
    if len(source) < 3 or not np.isfinite(source).all() or not np.isfinite(target).all():
        raise ValueError('need at least three finite paired positions')
    a, b = source - source.mean(0), target - target.mean(0)
    if min(np.linalg.matrix_rank(a), np.linalg.matrix_rank(b)) < 2:
        raise ValueError('collinear/constant trajectory cannot determine a full 3D alignment')
    u, singular, vt = np.linalg.svd(b.T @ a / len(a))
    signs = np.ones(3)
    signs[-1] = 1 if np.linalg.det(u @ vt) > 0 else -1
    rotation = u @ np.diag(signs) @ vt
    scale = float(np.dot(singular, signs) / np.mean(np.sum(a * a, axis=1)))
    if scale <= 0:
        raise ValueError('alignment has non-positive scale')
    translation = target.mean(0) - scale * rotation @ source.mean(0)
    return scale, rotation, translation


def rotate_quaternion(rotation, q):
    # Convert alignment rotation through axis-angle, then left-multiply q.
    import cv2
    vector, _ = cv2.Rodrigues(rotation)
    angle = float(np.linalg.norm(vector))
    axis = vector.ravel() / angle if angle > 1e-12 else np.zeros(3)
    v, w = axis * np.sin(angle / 2), np.cos(angle / 2)
    q = np.asarray(q, dtype=float)
    q /= np.linalg.norm(q)
    result = np.r_[w * q[:3] + q[3] * v + np.cross(v, q[:3]),
                   w * q[3] - np.dot(v, q[:3])]
    return result / np.linalg.norm(result)


def read_csv(path):
    with Path(path).open(newline='', encoding='utf-8-sig') as stream:
        return list(csv.DictReader(stream))


def write_csv(path, fields, rows):
    with Path(path).open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trajectory-csv', type=Path, required=True)
    parser.add_argument('--metrics-json', type=Path, required=True)
    parser.add_argument('--image-dir', type=Path, required=True)
    parser.add_argument('--start-image', required=True)
    parser.add_argument('--ground-truth', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--source-fps', type=float, default=30.0)
    parser.add_argument('--first-pose-frame', type=int)
    args = parser.parse_args()
    if args.source_fps <= 0:
        parser.error('source FPS must be positive')
    rows = read_csv(args.trajectory_csv)
    if not rows:
        parser.error('trajectory is empty')
    metrics = json.loads(args.metrics_json.read_text())
    interval = round(1e9 / args.source_fps)
    first_stamp = metrics.get('first_ack_stamp_ns')
    assumed = first_stamp is None
    if assumed:
        if args.first_pose_frame is None or args.first_pose_frame < 0:
            parser.error('old metrics require --first-pose-frame')
        first_stamp = int(rows[0]['stamp_ns']) - args.first_pose_frame * interval
    key = lambda p: [int(s) if s.isdigit() else s.lower()
                     for s in re.split(r'(\d+)', p.name)]
    files = sorted((p for p in args.image_dir.iterdir() if p.suffix.lower() in
                    ('.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff')), key=key)
    start = next((i for i, p in enumerate(files) if p.name == args.start_image), None)
    if start is None:
        parser.error('start image not found')
    count = int(metrics['frames_acknowledged'])
    files = files[start:start + count]
    if len(files) != count:
        parser.error('not enough source images for the recorded run')
    truth = {}
    for row in read_csv(args.ground_truth):
        label = Path(row['Label']).name
        if label in truth:
            raise ValueError(f'duplicate truth label: {label}')
        position = np.array([float(row[k]) for k in ('X', 'Y', 'Z')])
        if not np.isfinite(position).all():
            raise ValueError(f'nonfinite truth: {label}')
        truth[label] = position
    paired, seen = [], set()
    for row in rows:
        index = round((int(row['stamp_ns']) - first_stamp) / interval)
        if not 0 <= index < count or index in seen:
            raise ValueError(f'invalid/duplicate source frame index: {index}')
        seen.add(index)
        label = files[index].name
        if label in truth:
            paired.append((row, index, label))
    source = np.array([[float(r[k]) for k in ('x', 'y', 'z')] for r, _, _ in paired])
    target = np.array([truth[label] for _, _, label in paired])
    scale, rotation, translation = fit_sim3(source, target)
    errors = np.linalg.norm((scale * (rotation @ source.T)).T + translation - target, axis=1)
    output = []
    for row in rows:
        index = round((int(row['stamp_ns']) - first_stamp) / interval)
        label = files[index].name
        position = scale * rotation @ np.array([float(row[k]) for k in ('x', 'y', 'z')]) + translation
        q = rotate_quaternion(rotation, [float(row[k]) for k in ('qx', 'qy', 'qz', 'qw')])
        output.append(dict(zip(('stamp_ns', 'frame_index', 'label', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'),
                               (row['stamp_ns'], index, label, *position, *q))))
    gt_rows = [dict(frame_index=i, label=p.name, x=truth[p.name][0],
                    y=truth[p.name][1], z=truth[p.name][2])
               for i, p in enumerate(files) if p.name in truth]
    if not gt_rows:
        raise ValueError('no truth in selected image range')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / 'trajectory_aligned.csv', list(output[0]), output)
    write_csv(args.output_dir / 'ground_truth.csv', list(gt_rows[0]), gt_rows)
    report = dict(source_fps=args.source_fps, scale=scale, rotation=rotation.tolist(), translation=translation.tolist(),
                  paired_poses=len(paired), recorded_poses=len(rows), source_frames=count,
                  truth_frames=len(gt_rows), position_rmse=float(np.sqrt(np.mean(errors**2))),
                  position_median=float(np.median(errors)), position_p95=float(np.percentile(errors, 95)),
                  position_max=float(errors.max()), first_image_stamp_ns=first_stamp,
                  first_pose_frame_assumed=args.first_pose_frame if assumed else None,
                  synchronization='assumed_first_pose_frame' if assumed else 'recorded_first_image_stamp',
                  origin=[gt_rows[0][k] for k in ('x', 'y', 'z')],
                  ground_truth=str(args.ground_truth.resolve()), start_image=args.start_image,
                  note='One global Sim(3); fit and position error use the same matched camera centers. '
                       'No reflection, per-segment fitting, interpolation or OPK orientation comparison.')
    (args.output_dir / 'alignment.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
