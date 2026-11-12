"""Publish mono camera frames and matching ROS CameraInfo messages."""

from array import array
from pathlib import Path
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
import yaml


def load_camera_info(path: str) -> CameraInfo:
    with Path(path).expanduser().open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError('calibration YAML must contain a mapping')
    info = CameraInfo()
    info.width = int(data['image_width'])
    info.height = int(data['image_height'])
    info.distortion_model = str(data.get('distortion_model', 'plumb_bob'))
    for field, key, count in (('k', 'camera_matrix', 9),
                              ('r', 'rectification_matrix', 9),
                              ('p', 'projection_matrix', 12)):
        values = data[key]['data']
        if len(values) != count:
            raise ValueError(f'{key} must contain {count} numbers')
        setattr(info, field, [float(value) for value in values])
    info.d = [float(value) for value in data['distortion_coefficients']['data']]
    if info.width <= 0 or info.height <= 0 or info.k[0] <= 0 or info.k[4] <= 0:
        raise ValueError('calibration size and focal lengths must be positive')
    return info


class MonoCameraNode(Node):
    def __init__(self) -> None:
        super().__init__('mono_camera')
        for name, default in (
            ('source_type', 'spinnaker'), ('device', '/dev/video0'), ('pipeline', ''),
            ('camera_serial', ''), ('width', 1440), ('height', 1080), ('fps', 30.0),
            ('frame_id', 'camera_optical_frame'), ('encoding', 'mono8'),
            ('calibration_file', ''),
        ):
            self.declare_parameter(name, default)
        get = lambda name: self.get_parameter(name).value
        source_type = str(get('source_type'))
        device = str(get('device'))
        pipeline = str(get('pipeline'))
        width, height, fps = int(get('width')), int(get('height')), float(get('fps'))
        self.frame_id = str(get('frame_id'))
        self.encoding = str(get('encoding'))
        if width <= 0 or height <= 0 or fps <= 0:
            raise ValueError('width, height and fps must be positive')
        if self.encoding not in ('mono8', 'bgr8'):
            raise ValueError('encoding must be mono8 or bgr8')

        calibration_file = str(get('calibration_file'))
        self.camera_info = load_camera_info(calibration_file) if calibration_file else CameraInfo()
        if not calibration_file:
            self.get_logger().warning('No calibration_file: CameraInfo is uncalibrated (K[0] = 0)')

        if source_type == 'v4l2':
            self.capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.capture.set(cv2.CAP_PROP_FPS, fps)
        elif source_type == 'gstreamer' and pipeline:
            self.capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        elif source_type == 'spinnaker':
            from .spinnaker_capture import SpinnakerCapture
            self.capture = SpinnakerCapture(serial=str(get('camera_serial')))
        else:
            raise ValueError('source_type must be spinnaker, v4l2 or gstreamer with a pipeline')
        if not self.capture.isOpened():
            raise RuntimeError(f'could not open {source_type} camera source')

        self.image_pub = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', qos_profile_sensor_data)
        self.last_warning = 0.0
        self.timer = self.create_timer(1.0 / fps, self.publish_frame)
        self.get_logger().info(f'Camera ready: {source_type}, requested {width}x{height} @ {fps:g} FPS')

    def publish_frame(self) -> None:
        ok, frame = self.capture.read()
        if not ok or frame is None:
            now = time.monotonic()
            if now - self.last_warning >= 5.0:
                self.get_logger().warning('Camera frame read failed')
                self.last_warning = now
            return
        if frame.ndim == 2:
            mono, bgr = frame, None
        elif frame.ndim == 3 and frame.shape[2] == 3:
            mono, bgr = None, frame
        else:
            self.get_logger().error(f'Unsupported frame shape: {frame.shape}')
            return
        if self.encoding == 'mono8':
            image = mono if mono is not None else cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        else:
            image = bgr if bgr is not None else cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
        image = image.copy(order='C')
        image_height, image_width = image.shape[:2]
        if self.camera_info.k[0] != 0 and (
            self.camera_info.width != image_width or self.camera_info.height != image_height
        ):
            self.get_logger().error(
                f'Frame {image_width}x{image_height} differs from calibration '
                f'{self.camera_info.width}x{self.camera_info.height}; dropping frame')
            return

        stamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = image_height, image_width
        msg.encoding = self.encoding
        msg.is_bigendian = False
        msg.step = int(image.strides[0])
        # Avoid the generated ROS setter's per-pixel validation of bytes input.
        msg.data = array('B', image.tobytes())
        self.camera_info.header.stamp = stamp
        self.camera_info.header.frame_id = self.frame_id
        self.camera_info.width, self.camera_info.height = image_width, image_height
        self.image_pub.publish(msg)
        self.info_pub.publish(self.camera_info)

    def destroy_node(self) -> bool:
        if hasattr(self, 'capture'):
            self.capture.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MonoCameraNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
