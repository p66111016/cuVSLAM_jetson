from setuptools import setup
from glob import glob

name = 'mono_cuvslam'
setup(
    name=name,
    version='0.1.0',
    packages=[name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml']),
        ('share/' + name + '/launch', glob('launch/*.launch.py')),
        ('share/' + name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mono cuVSLAM project',
    maintainer_email='maintainer@example.com',
    description='Custom ROS 2 wrapper around PyCuVSLAM Core mono odometry',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'mono_cuvslam_node = mono_cuvslam.node:main',
        'stage4_metrics = mono_cuvslam.metrics_node:main',
    ]},
)
