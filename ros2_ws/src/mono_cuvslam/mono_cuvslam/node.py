"""ROS 2 mono odometry using the cuVSLAM Core Python API (no Isaac ROS node)."""

import time
import json

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String, UInt64
from tf2_ros import TransformBroadcaster

import cuvslam


def make_camera(info: CameraInfo):
    """Map ROS camera calibration to cuVSLAM's distortion coefficient order."""
    if info.k[0] <= 0 or info.k[4] <= 0:
        raise ValueError('CameraInfo is uncalibrated (K[0] or K[4] is zero)')
    d = list(info.d)
    if not d or all(value == 0 for value in d):
        distortion = cuvslam.Distortion(cuvslam.Distortion.Model.Pinhole)
    elif info.distortion_model == 'plumb_bob' and len(d) >= 5:
        # ROS: k1,k2,p1,p2,k3. cuVSLAM Brown: k1,k2,k3,p1,p2.
        distortion = cuvslam.Distortion(cuvslam.Distortion.Model.Brown,
                                         [d[0], d[1], d[4], d[2], d[3]])
    elif info.distortion_model == 'equidistant' and len(d) == 4:
        distortion = cuvslam.Distortion(cuvslam.Distortion.Model.Fisheye, d)
    elif info.distortion_model == 'rational_polynomial' and len(d) >= 8:
        distortion = cuvslam.Distortion(cuvslam.Distortion.Model.Polynomial, d[:8])
    else:
        raise ValueError(f'unsupported distortion model/coefficient count: {info.distortion_model}, {len(d)}')
    return cuvslam.Camera(
        size=[info.width, info.height],
        principal=[info.k[2], info.k[5]],
        focal=[info.k[0], info.k[4]],
        distortion=distortion,
    )


def mono_array(msg: Image) -> np.ndarray:
    if msg.is_bigendian:
        raise ValueError('big-endian Image is unsupported')
    if msg.encoding not in ('mono8', 'bgr8'):
        raise ValueError(f'unsupported image encoding: {msg.encoding}')
    channels = 1 if msg.encoding == 'mono8' else 3
    if msg.step < msg.width * channels or len(msg.data) < msg.step * msg.height:
        raise ValueError('Image data/step is inconsistent with dimensions')
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    if channels == 1:
        return np.ascontiguousarray(raw[:, :msg.width])
    import cv2
    bgr = raw[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)


class MonoCuVslamNode(Node):
    def __init__(self):
        super().__init__('mono_cuvslam')
        self.declare_parameter('odom_frame', 'cuvslam_odom_optical')
        self.declare_parameter('max_path_poses', 2000)
        self.declare_parameter('input_reliability', 'best_effort')
        self.declare_parameter('publish_tf', True)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.max_path_poses = int(self.get_parameter('max_path_poses').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        reliability = str(self.get_parameter('input_reliability').value)
        if self.max_path_poses <= 0:
            raise ValueError('max_path_poses must be positive')
        if reliability not in ('best_effort', 'reliable'):
            raise ValueError('input_reliability must be best_effort or reliable')
        input_qos = (QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
                     if reliability == 'reliable' else qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/mono_cuvslam/odometry', 10)
        self.path_pub = self.create_publisher(Path, '/mono_cuvslam/path', 10)
        self.status_pub = self.create_publisher(String, '/mono_cuvslam/tracking_status', 10)
        self.ack_pub = self.create_publisher(UInt64, '/mono_cuvslam/processed_stamp_ns', 10)
        self.diagnostic_pub = self.create_publisher(String, '/mono_cuvslam/frame_diagnostics', 100)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.path = Path()
        self.path.header.frame_id = self.odom_frame
        self.tracker = None
        self.calibration = None
        self.pending_images = {}
        self.pending_info = {}
        self.last_timestamp_ns = -1
        self.last_status = None
        self.last_log = 0.0
        self.create_subscription(Image, '/camera/image_raw', self.on_image, input_qos)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, input_qos)
        self.publish_status('WAITING_FOR_INPUT')

    def publish_status(self, status: str):
        self.status_pub.publish(String(data=status))
        if status != self.last_status:
            self.get_logger().info(f'tracking_status={status}')
            self.last_status = status

    @staticmethod
    def stamp_ns(msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def on_image(self, msg: Image):
        key = self.stamp_ns(msg)
        self.pending_images[key] = msg
        self.flush_pending()

    def on_info(self, msg: CameraInfo):
        key = self.stamp_ns(msg)
        self.pending_info[key] = msg
        self.flush_pending()

    def flush_pending(self):
        for key in sorted(self.pending_images.keys() & self.pending_info.keys()):
            image = self.pending_images.pop(key)
            info = self.pending_info.pop(key)
            started = time.perf_counter()
            self.frame_track_ms = None
            try:
                self.process_pair(key, image, info)
            finally:
                self.diagnostic_pub.publish(String(data=json.dumps({
                    'stamp_ns': key, 'status': self.last_status,
                    'processing_ms': (time.perf_counter() - started) * 1000,
                    'track_ms': self.frame_track_ms,
                    'completed_wall_ns': time.time_ns(),
                })))
                self.ack_pub.publish(UInt64(data=key))
        # Bound memory if a source sends only one of the two topics.
        for cache in (self.pending_images, self.pending_info):
            while len(cache) > 20:
                cache.pop(min(cache))

    def process_pair(self, timestamp_ns: int, image: Image, info: CameraInfo):
        if timestamp_ns <= self.last_timestamp_ns:
            self.publish_status('OUT_OF_ORDER')
            return
        if image.width != info.width or image.height != info.height:
            self.publish_status('CALIBRATION_MISMATCH')
            return
        if image.header.frame_id != info.header.frame_id:
            self.publish_status('FRAME_ID_MISMATCH')
            return
        try:
            frame = mono_array(image)
            calibration = (info.width, info.height, tuple(info.k), tuple(info.d), info.distortion_model)
            if self.tracker is None:
                camera = make_camera(info)
                config = cuvslam.core.Odometry.Config(
                    odometry_mode=cuvslam.core.Odometry.OdometryMode.Mono,
                    use_gpu=True,
                    async_sba=False,
                )
                self.tracker = cuvslam.core.Odometry(cuvslam.Rig([camera]), config)
                self.calibration = calibration
                self.get_logger().info(f'cuVSLAM Mono initialized for {image.width}x{image.height}')
            elif calibration != self.calibration:
                self.publish_status('CALIBRATION_CHANGED_RESTART_REQUIRED')
                return
            track_started = time.perf_counter()
            estimate = self.tracker.track(timestamp_ns, [frame])
            self.frame_track_ms = (time.perf_counter() - track_started) * 1000
            self.last_timestamp_ns = timestamp_ns
        except (ValueError, RuntimeError) as exc:
            now = time.monotonic()
            if now - self.last_log > 2.0:
                self.get_logger().error(f'cuVSLAM input/tracking error: {exc}')
                self.last_log = now
            self.publish_status('ERROR')
            return

        if estimate.world_from_rig is None:
            self.publish_status('LOST')
            return
        state = self.tracker.get_state()
        if state.warming_up:
            # Initialization poses can be identities and must not enter a trajectory.
            self.publish_status('WARMING_UP')
            return
        self.publish_status('TRACKING')
        pose = estimate.world_from_rig.pose
        odom = Odometry()
        odom.header.stamp = image.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = image.header.frame_id
        odom.pose.pose.position.x = float(pose.translation[0])
        odom.pose.pose.position.y = float(pose.translation[1])
        odom.pose.pose.position.z = float(pose.translation[2])
        odom.pose.pose.orientation.x = float(pose.rotation[0])
        odom.pose.pose.orientation.y = float(pose.rotation[1])
        odom.pose.pose.orientation.z = float(pose.rotation[2])
        odom.pose.pose.orientation.w = float(pose.rotation[3])
        covariance = list(estimate.world_from_rig.covariance_xyz_rpy)
        if len(covariance) == 36:
            odom.pose.covariance = covariance
        else:
            for index in (0, 7, 14, 21, 28, 35):
                odom.pose.covariance[index] = 1e6
        # Velocity is not estimated here. Mark twist covariance as very large.
        for index in (0, 7, 14, 21, 28, 35):
            odom.twist.covariance[index] = 1e6
        self.odom_pub.publish(odom)
        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = odom.child_frame_id
            transform.transform.translation.x = odom.pose.pose.position.x
            transform.transform.translation.y = odom.pose.pose.position.y
            transform.transform.translation.z = odom.pose.pose.position.z
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

        stamped = PoseStamped()
        stamped.header = odom.header
        stamped.pose = odom.pose.pose
        self.path.poses.append(stamped)
        self.path.poses = self.path.poses[-self.max_path_poses:]
        self.path.header.stamp = odom.header.stamp
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = MonoCuVslamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
