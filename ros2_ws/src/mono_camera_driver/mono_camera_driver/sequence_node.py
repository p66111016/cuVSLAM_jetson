"""Replay existing image files as stamped ROS camera topics."""

from array import array
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import re

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import UInt64

from .node import load_camera_info
from .sequence_data import load_image_timestamps, load_imu, ImuReplay


def natural_key(path: Path):
    return [int(part) if part.isdigit() else part.lower()
            for part in re.split(r'(\d+)', path.name)]


class ImageSequenceNode(Node):
    def __init__(self):
        super().__init__('image_sequence')
        for name, default in (
            ('image_dir', ''), ('calibration_file', ''), ('glob', '*'),
            ('fps', 30.0), ('replay_fps', 0.0), ('frame_id', 'camera_optical_frame'),
            ('start_index', 0), ('max_frames', 0), ('prefetch', True),
            ('timestamp_file', ''), ('imu_file', ''), ('imu_frame_id', 'imu_link'),
            ('imu_warmup_s', 1.0), ('max_imu_gap_s', 0.05),
            ('imu_gyro_scale', 1.0), ('imu_accel_scale', 1.0),
        ):
            self.declare_parameter(name, default)
        get = lambda name: self.get_parameter(name).value
        image_dir = Path(str(get('image_dir'))).expanduser()
        calibration_file = str(get('calibration_file'))
        if not image_dir.is_dir() or not calibration_file:
            raise ValueError('image_dir must exist and calibration_file is required')
        self.info = load_camera_info(calibration_file)
        self.frame_id = str(get('frame_id'))
        fps = float(get('fps'))
        replay_fps = float(get('replay_fps'))
        if fps <= 0 or replay_fps < 0:
            raise ValueError('fps must be positive and replay_fps cannot be negative')
        files = sorted((path for path in image_dir.glob(str(get('glob')))
                        if path.is_file() and path.suffix.lower() in
                        ('.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff')),
                       key=natural_key)
        start = int(get('start_index'))
        limit = int(get('max_frames'))
        if start < 0 or limit < 0:
            raise ValueError('start_index and max_frames cannot be negative')
        self.files = files[start:start + limit if limit else None]
        if not self.files:
            raise ValueError(f'no images found in {image_dir}')
        timestamp_file = str(get('timestamp_file'))
        imu_file = str(get('imu_file'))
        self.image_stamps = (load_image_timestamps(timestamp_file, self.files)
                             if timestamp_file else None)
        self.imu_replay = None
        self.imu_pub = None
        if imu_file:
            if self.image_stamps is None:
                raise ValueError('imu_file requires timestamp_file; synthetic FPS is unsafe for IMU')
            self.imu_replay = ImuReplay(load_imu(imu_file), self.image_stamps,
                                        float(get('imu_warmup_s')), float(get('max_imu_gap_s')))
            self.imu_frame_id = str(get('imu_frame_id'))
            self.gyro_scale = float(get('imu_gyro_scale'))
            self.accel_scale = float(get('imu_accel_scale'))
            import math
            if not all(math.isfinite(v) and v > 0 for v in (self.gyro_scale, self.accel_scale)):
                raise ValueError('IMU unit scales must be finite and positive')
            self.imu_pub = self.create_publisher(
                Imu, '/imu/data_raw', QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))
            self.get_logger().warning('Publishing synchronized raw IMU only. cuVSLAM Mono does '
                                      'NOT fuse IMU; a supported VIO backend is required.')
        self.index = 0
        self.base_stamp_ns = self.get_clock().now().nanoseconds
        self.frame_interval_ns = round(1_000_000_000 / fps)
        self.fast_replay = replay_fps == 0
        self.waiting_stamp_ns = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', qos)
        self.info_pub = self.create_publisher(type(self.info), '/camera/camera_info', qos)
        self.create_subscription(UInt64, '/mono_cuvslam/processed_stamp_ns', self.on_ack, 10)
        self.timer = self.create_timer(0.01 if self.fast_replay else 1.0 / replay_fps,
                                       self.publish_next)
        self.decoder = (ThreadPoolExecutor(max_workers=1, thread_name_prefix='mono_image_decode')
                        if bool(get('prefetch')) else None)
        self.prefetch_future = (self.decoder.submit(self.read_frame, self.files[0])
                                if self.decoder is not None else None)
        self.get_logger().info(f'Loaded {len(self.files)} images; timestamps '
                               f'{timestamp_file or str(fps) + " FPS synthetic"}; '
                               f'replay {"uncapped" if self.fast_replay else str(replay_fps) + " FPS"}')

    def read_frame(self, path: Path):
        frame = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if frame is None:
            raise RuntimeError(f'cannot decode image: {path}')
        height, width = frame.shape
        if (width, height) != (self.info.width, self.info.height):
            raise RuntimeError(f'{path}: image {width}x{height} differs from calibration '
                               f'{self.info.width}x{self.info.height}')
        return frame

    def on_ack(self, msg: UInt64):
        if msg.data != self.waiting_stamp_ns:
            return
        self.waiting_stamp_ns = None
        if self.fast_replay:
            self.publish_next()

    def publish_next(self):
        # Wait so a slow-starting cuVSLAM node does not miss the beginning.
        if self.waiting_stamp_ns is not None:
            return
        if self.image_pub.get_subscription_count() == 0 or self.info_pub.get_subscription_count() == 0:
            return
        if self.index >= len(self.files):
            self.timer.cancel()
            self.get_logger().info(f'Image sequence complete: {self.index} frames')
            return
        path = self.files[self.index]
        frame = (self.prefetch_future.result() if self.prefetch_future is not None
                 else self.read_frame(path))
        self.prefetch_future = None
        if self.decoder is not None and self.index + 1 < len(self.files):
            self.prefetch_future = self.decoder.submit(self.read_frame,
                                                       self.files[self.index + 1])
        height, width = frame.shape
        stamp_ns = (self.image_stamps[self.index] if self.image_stamps is not None
                    else self.base_stamp_ns + self.index * self.frame_interval_ns)
        if self.imu_replay is not None:
            for sample in self.imu_replay.through(stamp_ns):
                msg = Imu()
                msg.header.stamp = Time(nanoseconds=sample.stamp_ns).to_msg()
                msg.header.frame_id = self.imu_frame_id
                msg.orientation_covariance[0] = -1.0
                # Unknown covariance (all zeros); preserve raw sensor axes and gravity.
                for axis, gyro, accel in zip(('x', 'y', 'z'), sample.gyro, sample.accel):
                    setattr(msg.angular_velocity, axis, gyro * self.gyro_scale)
                    setattr(msg.linear_acceleration, axis, accel * self.accel_scale)
                self.imu_pub.publish(msg)
        stamp = Time(nanoseconds=stamp_ns).to_msg()
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = self.frame_id
        image.width, image.height = width, height
        image.encoding = 'mono8'
        image.step = width
        # ROS 2 accepts array('B') directly; bytes triggers a per-pixel Python check.
        image.data = array('B', frame.tobytes())
        self.info.header.stamp = stamp
        self.info.header.frame_id = self.frame_id
        self.waiting_stamp_ns = stamp_ns
        self.image_pub.publish(image)
        self.info_pub.publish(self.info)
        self.index += 1

    def destroy_node(self) -> bool:
        if hasattr(self, 'decoder') and self.decoder is not None:
            self.decoder.shutdown(wait=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImageSequenceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
