from setuptools import setup, find_packages

package_name = 'ai_robot_brain'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        ('share/' + package_name + '/launch', ['launch/isaac_sim_bridge.launch.py']),
        # Include config files
        ('share/' + package_name + '/config', ['config/simulation_config.yaml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'sensor_msgs',
        'geometry_msgs', 
        'nav_msgs',
        'tf2_ros',
        'cv_bridge',
        'numpy',
        'opencv-python',
        'torch',
        'torchvision',
        'tensorflow',
        'gymnasium',
        'stable_baselines3',
        'omni.isaac.gym',
        'omni.isaac.core'
    ],
    zip_safe=True,
    author='AI-Native Book Team',
    author_email='contact@ainativebook.com',
    maintainer='AI-Native Book Team',
    maintainer_email='contact@ainativebook.com',
    keywords=['ROS', 'Isaac Sim', 'Robotics', 'Simulation', 'AI'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python',
        'Topic :: Software Development :: Libraries :: Python Modules',
    ],
    description='AI-Robot Brain Package for Isaac Sim Integration',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'isaac_sim_bridge = ai_robot_brain.isaac_sim_bridge:main',
            'perception_node = ai_robot_brain.perception.perception_node:main',
            'vslam_node = ai_robot_brain.vslam.vslam_node:main',
            'synthetic_generator = ai_robot_brain.synthetic_data.synthetic_generator:main',
            'training_node = ai_robot_brain.perception.training_node:main',
        ],
    },
)