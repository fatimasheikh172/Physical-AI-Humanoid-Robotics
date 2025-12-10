from setuptools import setup
import os
from glob import glob

package_name = 'digital_twin_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AI Native Book Team',
    maintainer_email='info@example.com',
    description='Perception package for digital twin system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_analysis_node = digital_twin_perception.imu_analysis.imu_analysis_node:main',
            'perception_demo = digital_twin_perception.perception_demo:main',
            'sensor_fidelity_analysis = digital_twin_perception.sensor_fidelity_analysis:main',
            'lidar_obstacle_detector = digital_twin_perception.lidar_processing.lidar_pointcloud_processor:main',
            'depth_camera_processor = digital_twin_perception.depth_camera.depth_camera_processor:main',
        ],
    },
)