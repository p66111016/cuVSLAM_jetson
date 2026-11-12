"""Measure offline replay throughput, tracking continuity, and TF consistency."""

import csv
import json
import math
import time
from pathlib import Path as FilePath

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, UInt64
from tf2_msgs.msg import TFMessage


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class Stage4Metrics(Node):
    def __init__(self):
        super().__init__('mono_cuvslam_metrics')
        self.declare_parameter('expected_frames', 0)
        self.declare_parameter('output_file', '')
        self.declare_parameter('trajectory_file', '')
        self.declare_parameter('odom_frame', 'cuvslam_odom_optical')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.expected_frames = int(self.get_parameter('expected_frames').value)
        self.output_file = str(self.get_parameter('output_file').value)
        self.trajectory_file = str(self.get_parameter('trajectory_file').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        if self.expected_frames < 0:
            raise ValueError('expected_frames cannot be negative')
        self.ack_count = 0
        self.first_ack_wall = None
        self.last_ack_wall = None
        self.first_ack_stamp_ns = None
        self.completion_wall = None
        self.done = False
        self.status_counts = {}
        self.tracking_streak = 0
        self.max_tracking_streak = 0
        self.odom_count = 0
        self.path_count = 0
        self.path_last_poses = 0
        self.tf_count = 0
        self.tf_odom_matches = 0
        self.max_tf_translation_error = 0.0
        self.max_tf_rotation_error = 0.0
        self.first_position = None
        self.last_position = None
        self.odom_samples = {}
        self.tf_samples = {}
        self.trajectory_stream = None
        self.diagnostic_stream = None
        if self.trajectory_file:
            trajectory_path = FilePath(self.trajectory_file).expanduser()
            trajectory_path.parent.mkdir(parents=True, exist_ok=True)
            self.trajectory_stream = trajectory_path.open('w', newline='', encoding='utf-8')
            self.trajectory_writer = csv.writer(self.trajectory_stream)
            self.trajectory_writer.writerow(('stamp_ns', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'))
            self.diagnostic_stream = trajectory_path.with_suffix('.frames.csv').open('w', newline='', encoding='utf-8')
            self.diagnostic_writer = csv.DictWriter(self.diagnostic_stream, fieldnames=(
                'stamp_ns', 'status', 'processing_ms', 'track_ms', 'completed_wall_ns'))
            self.diagnostic_writer.writeheader()
        self.create_subscription(String, '/mono_cuvslam/frame_diagnostics', self.on_diagnostic, 100)
        self.create_subscription(UInt64, '/mono_cuvslam/processed_stamp_ns', self.on_ack, 100)
        self.create_subscription(String, '/mono_cuvslam/tracking_status', self.on_status, 100)
        self.create_subscription(Odometry, '/mono_cuvslam/odometry', self.on_odom, 100)
        self.create_subscription(Path, '/mono_cuvslam/path', self.on_path, 10)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.create_timer(0.1, self.check_complete)

    def on_diagnostic(self, msg):
        if self.diagnostic_stream is not None:
            self.diagnostic_writer.writerow(json.loads(msg.data))

    def on_ack(self, msg):
        now = time.monotonic()
        if self.first_ack_wall is None:
            self.first_ack_wall = now
            self.first_ack_stamp_ns = int(msg.data)
        self.last_ack_wall = now
        self.ack_count += 1
        if self.expected_frames and self.ack_count >= self.expected_frames:
            self.completion_wall = now

    def on_status(self, msg):
        if msg.data == 'WAITING_FOR_INPUT':
            return
        self.status_counts[msg.data] = self.status_counts.get(msg.data, 0) + 1
        if msg.data == 'TRACKING':
            self.tracking_streak += 1
            self.max_tracking_streak = max(self.max_tracking_streak, self.tracking_streak)
        else:
            self.tracking_streak = 0

    def on_odom(self, msg):
        self.odom_count += 1
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        position = (p.x, p.y, p.z)
        if self.first_position is None:
            self.first_position = position
        self.last_position = position
        key = stamp_ns(msg.header.stamp)
        if self.trajectory_stream is not None:
            self.trajectory_writer.writerow((key, *position, q.x, q.y, q.z, q.w))
        self.odom_samples[key] = (position, (q.x, q.y, q.z, q.w))
        self.compare_transform(key)

    def on_path(self, msg):
        self.path_count += 1
        self.path_last_poses = len(msg.poses)

    def on_tf(self, msg):
        for transform in msg.transforms:
            if (transform.header.frame_id != self.odom_frame or
                    transform.child_frame_id != self.camera_frame):
                continue
            self.tf_count += 1
            p = transform.transform.translation
            q = transform.transform.rotation
            key = stamp_ns(transform.header.stamp)
            self.tf_samples[key] = ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
            self.compare_transform(key)

    def compare_transform(self, key):
        if key not in self.odom_samples or key not in self.tf_samples:
            return
        odom_p, odom_q = self.odom_samples.pop(key)
        tf_p, tf_q = self.tf_samples.pop(key)
        self.tf_odom_matches += 1
        translation_error = math.dist(odom_p, tf_p)
        rotation_error = max(abs(a - b) for a, b in zip(odom_q, tf_q))
        self.max_tf_translation_error = max(self.max_tf_translation_error, translation_error)
        self.max_tf_rotation_error = max(self.max_tf_rotation_error, rotation_error)

    def check_complete(self):
        if self.completion_wall is not None and time.monotonic() - self.completion_wall >= 1.0:
            self.done = True

    def report(self):
        duration = (self.last_ack_wall - self.first_ack_wall
                    if self.first_ack_wall is not None and self.last_ack_wall is not None
                    else None)
        status_frames = sum(self.status_counts.values())
        displacement = (math.dist(self.first_position, self.last_position)
                        if self.first_position is not None and self.last_position is not None
                        else None)
        result = {
            'complete': self.expected_frames == 0 or self.ack_count >= self.expected_frames,
            'expected_frames': self.expected_frames,
            'frames_acknowledged': self.ack_count,
            'first_ack_stamp_ns': self.first_ack_stamp_ns,
            'duration_s': round(duration, 3) if duration is not None else None,
            'processing_fps': round((self.ack_count - 1) / duration, 3)
                              if duration and self.ack_count > 1 else None,
            'status_counts': self.status_counts,
            'tracking_ratio': round(self.status_counts.get('TRACKING', 0) / status_frames, 4)
                              if status_frames else None,
            'max_tracking_streak': self.max_tracking_streak,
            'odometry_messages': self.odom_count,
            'path_messages': self.path_count,
            'last_path_poses': self.path_last_poses,
            'tf_messages': self.tf_count,
            'tf_odometry_matches': self.tf_odom_matches,
            'max_tf_translation_error': self.max_tf_translation_error,
            'max_tf_rotation_error': self.max_tf_rotation_error,
            'position_displacement_unscaled': round(displacement, 6)
                                              if displacement is not None else None,
        }
        line = json.dumps(result, sort_keys=True)
        print('STAGE4_METRICS=' + line, flush=True)
        if self.output_file:
            path = FilePath(self.output_file).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        if self.trajectory_stream is not None:
            self.trajectory_stream.flush()
        return result

    def destroy_node(self):
        if self.diagnostic_stream is not None:
            self.diagnostic_stream.close()
        if self.trajectory_stream is not None:
            self.trajectory_stream.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Stage4Metrics()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        result = node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not result['complete']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
