#!/usr/bin/env python3
"""Measure PNG decoding and PyCuVSLAM Core tracking without ROS topic overhead."""

import argparse
import json
import statistics
import time
from pathlib import Path

import cv2
import cuvslam

from mono_camera_driver.node import load_camera_info
from mono_camera_driver.sequence_node import natural_key
from mono_cuvslam.node import make_camera


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image-dir', default='/home/user/2026-06-22VTOL/firefly')
    parser.add_argument('--calibration', default='/home/user/mono-cuvslam-ros2/config/firefly_20260622_assumed.yaml')
    parser.add_argument('--start-index', type=int, default=1)
    parser.add_argument('--frames', type=int, default=100)
    parser.add_argument('--source-fps', type=float, default=30.0)
    args = parser.parse_args()
    if args.start_index < 0 or args.frames <= 0 or args.source_fps <= 0:
        parser.error('start-index must be nonnegative; frames and source-fps must be positive')

    files = sorted(Path(args.image_dir).glob('firefly_*.png'), key=natural_key)
    chosen = files[args.start_index:args.start_index + args.frames]
    if len(chosen) != args.frames:
        parser.error(f'only {len(chosen)} images available for the requested range')
    info = load_camera_info(args.calibration)
    config = cuvslam.core.Odometry.Config(
        odometry_mode=cuvslam.core.Odometry.OdometryMode.Mono,
        use_gpu=True,
        async_sba=False,
    )
    cuvslam.warm_up_gpu()
    tracker = cuvslam.core.Odometry(cuvslam.Rig([make_camera(info)]), config)
    base_stamp_ns = time.time_ns()
    frame_interval_ns = round(1_000_000_000 / args.source_fps)
    decode_ms = []
    track_ms = []
    states = {}
    start = time.monotonic()
    for index, path in enumerate(chosen):
        t0 = time.perf_counter()
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        t1 = time.perf_counter()
        if image is None or image.shape != (info.height, info.width):
            raise RuntimeError(f'cannot decode calibrated image: {path}')
        estimate = tracker.track(base_stamp_ns + index * frame_interval_ns, [image])
        t2 = time.perf_counter()
        state = tracker.get_state()
        status = ('LOST' if estimate.world_from_rig is None else
                  'WARMING_UP' if state.warming_up else 'TRACKING')
        states[status] = states.get(status, 0) + 1
        decode_ms.append((t1 - t0) * 1000)
        track_ms.append((t2 - t1) * 1000)
    wall_s = time.monotonic() - start
    print(json.dumps({
        'gpu_acceleration_requested': config.use_gpu,
        'gpu_warmup': 'OK',
        'frames': len(chosen),
        'wall_seconds': round(wall_s, 3),
        'end_to_end_fps': round(len(chosen) / wall_s, 3),
        'png_decode_ms_mean': round(statistics.mean(decode_ms), 3),
        'png_decode_ms_median': round(statistics.median(decode_ms), 3),
        'cuvslam_track_ms_mean': round(statistics.mean(track_ms), 3),
        'cuvslam_track_ms_median': round(statistics.median(track_ms), 3),
        'status_counts': states,
    }, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
