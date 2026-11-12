"""Opt-in ROS replay integration check using the local VTOL dataset.

Run after building both packages and sourcing scripts/ros_env.sh:
ROS_LOG_DIR=/tmp/mono-imu-roslog ROS_DOMAIN_ID=91 /usr/bin/python3 tests/verify_imu_ros_replay.py
No CUDA or tracker is required: the observer acknowledges received images.
"""
import time
from pathlib import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import UInt64
from mono_camera_driver.sequence_node import ImageSequenceNode, natural_key
from mono_camera_driver.sequence_data import load_image_timestamps, load_imu, ImuReplay

root = Path('/home/user/2026-06-22VTOL')
files = sorted((root / 'firefly').glob('firefly_*.png'), key=natural_key)
start = next(i for i, p in enumerate(files) if p.name == 'firefly_008011.png')
selected = files[start:start+3]
stamps = load_image_timestamps(root/'IMU/EOP_firefly_unix_aligned_imu.csv', selected)
expected = ImuReplay(load_imu(root/'IMU/imu_200hz.csv'), stamps)
expected_samples = [s for stamp in stamps for s in expected.through(stamp)]
rclpy.init(args=['--ros-args', '-p', f'image_dir:={root}/firefly', '-p',
                'calibration_file:=/home/user/mono-cuvslam-ros2/config/firefly_20260622_metashape.yaml',
                '-p', f'start_index:={start}', '-p', 'max_frames:=3', '-p',
                f'timestamp_file:={root}/IMU/EOP_firefly_unix_aligned_imu.csv', '-p',
                f'imu_file:={root}/IMU/imu_200hz.csv'])
source = ImageSequenceNode()
observer = Node('replay_observer', use_global_arguments=False)
qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
images, infos, samples = [], [], []
stamp_ns = lambda msg: msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
ack = observer.create_publisher(UInt64, '/mono_cuvslam/processed_stamp_ns', 10)
def on_image(msg):
    images.append(stamp_ns(msg))
    ack.publish(UInt64(data=images[-1]))
observer.create_subscription(Image, '/camera/image_raw', on_image, qos)
observer.create_subscription(CameraInfo, '/camera/camera_info', lambda m: infos.append(stamp_ns(m)), qos)
observer.create_subscription(Imu, '/imu/data_raw', samples.append, qos)
executor = SingleThreadedExecutor()
executor.add_node(source)
executor.add_node(observer)
try:
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline and (len(images) < 3 or len(infos) < 3 or len(samples) < len(expected_samples)):
        executor.spin_once(timeout_sec=0.1)
    assert images == stamps, (images, stamps)
    assert infos == stamps
    assert [stamp_ns(m) for m in samples] == [s.stamp_ns for s in expected_samples]
    assert samples[0].angular_velocity.x == expected_samples[0].gyro[0]
    assert samples[0].linear_acceleration.z == expected_samples[0].accel[2]
    assert samples[0].orientation_covariance[0] == -1
    print(f'ROS replay passed: {len(images)} images, {len(samples)} exact IMU samples; start_index={start}')
finally:
    executor.shutdown()
    source.destroy_node()
    observer.destroy_node()
    rclpy.shutdown()
