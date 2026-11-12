from setuptools import setup

package_name = 'mono_camera_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mono cuVSLAM project',
    maintainer_email='maintainer@example.com',
    description='Monocular camera publisher for ROS 2 Humble',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'mono_camera_node = mono_camera_driver.node:main',
        'image_sequence_node = mono_camera_driver.sequence_node:main',
    ]},
)
