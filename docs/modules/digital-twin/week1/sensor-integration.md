# Sensor Integration

## Overview

This section covers the integration of various sensor models into your robot simulation. You'll learn how to add LiDAR, depth camera, and IMU sensors to your robot model and configure them to generate realistic sensor data for your digital twin system.

## Learning Objectives

By the end of this section, you will be able to:

- Add sensor models to URDF robot descriptions
- Configure sensor parameters to match real-world specifications
- Validate sensor data accuracy and quality
- Troubleshoot common sensor integration issues

## LiDAR Sensor Integration

### Adding LiDAR to URDF

To add a LiDAR sensor to your robot model, you'll need to define it in your URDF file. Here's an example:

```xml
<!-- LiDAR Mount -->
<link name="lidar_mount">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.01"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.01"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
  </inertial>
</link>

<!-- LiDAR Sensor -->
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.05"/>
    </geometry>
    <material name="dark_gray">
      <color rgba="0.3 0.3 0.3 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.2"/>
    <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
  </inertial>
</link>

<!-- Joint connecting LiDAR to mount -->
<joint name="lidar_mount_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_mount"/>
  <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
</joint>

<!-- Joint connecting mount to LiDAR -->
<joint name="lidar_joint" type="fixed">
  <parent link="lidar_mount"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.03" rpy="0 0 0"/>
</joint>

<!-- Gazebo plugin for LiDAR -->
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar_sensor">
    <pose>0 0 0 0 0 0</pose>
    <visualize>false</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <argument>~/out:=scan</argument>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

## Depth Camera Integration

### Adding Depth Camera to URDF

```xml
<!-- Depth Camera Mount -->
<link name="camera_mount">
  <visual>
    <geometry>
      <box size="0.02 0.02 0.02"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.02 0.02 0.02"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.05"/>
    <inertia ixx="1e-5" ixy="0.0" ixz="0.0" iyy="1e-5" iyz="0.0" izz="1e-5"/>
  </inertial>
</link>

<!-- Depth Camera Link -->
<link name="camera_link">
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="1e-6" ixy="0.0" ixz="0.0" iyy="1e-6" iyz="0.0" izz="1e-6"/>
  </inertial>
</link>

<!-- Joint connecting camera to robot -->
<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
</joint>

<!-- Gazebo plugin for depth camera -->
<gazebo reference="camera_link">
  <sensor type="depth" name="depth_camera">
    <always_on>true</always_on>
    <visualize>true</visualize>
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <format>R8G8B8</format>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>10</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <baseline>0.2</baseline>
      <always_on>true</always_on>
      <update_rate>30.0</update_rate>
      <camera_name>camera</camera_name>
      <image_topic_name>rgb/image_raw</image_topic_name>
      <depth_image_topic_name>depth/image_raw</depth_image_topic_name>
      <point_cloud_topic_name>depth/points</point_cloud_topic_name>
      <camera_info_topic_name>rgb/camera_info</camera_info_topic_name>
      <depth_image_camera_info_topic_name>depth/camera_info</depth_image_camera_info_topic_name>
      <point_cloud_cutoff>0.1</point_cloud_cutoff>
      <frame_name>camera_depth_frame</frame_name>
      <baseline>0.1</baseline>
      <distortion_k1>0.0</distortion_k1>
      <distortion_k2>0.0</distortion_k2>
      <distortion_k3>0.0</distortion_k3>
      <distortion_t1>0.0</distortion_t1>
      <distortion_t2>0.0</distortion_t2>
      <point_cloud_cutoff_max>3.0</point_cloud_cutoff_max>
    </plugin>
  </sensor>
</gazebo>
```

## IMU Integration

### Adding IMU to URDF

```xml
<!-- IMU Sensor -->
<link name="imu_link">
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="1e-7" ixy="0.0" ixz="0.0" iyy="1e-7" iyz="0.0" izz="1e-7"/>
  </inertial>
</link>

<!-- Joint connecting IMU to robot -->
<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>

<!-- Gazebo plugin for IMU -->
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>false</visualize>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
      <topicName>imu</topicName>
      <bodyName>imu_link</bodyName>
      <frameName>imu_link</frameName>
      <updateRateHZ>100.0</updateRateHZ>
      <gaussianNoise>0.0</gaussianNoise>
      <ros>
        <remapping>~/out:=imu</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

## Configuration Guidelines

### LiDAR Parameters
- Sample count: Higher counts provide more detailed scans but require more processing
- Range: Set based on the sensor's real-world capabilities
- Update rate: Balance between real-time performance and data frequency

### Depth Camera Parameters
- Image resolution: Higher resolution provides more detail but increases processing requirements
- FOV (Field of View): Should match the real sensor's specifications
- Update rate: Typically 15-30 Hz for real-time applications

### IMU Parameters
- Update rate: Usually higher (100-500 Hz) for accurate motion tracking
- Noise parameters: Match these to real-world sensor characteristics
- Frame alignment: Ensure proper coordinate system alignment with the robot

## Validation Steps

1. Launch your Gazebo world with the robot
2. Check that all sensors are publishing data:
   ```bash
   ros2 topic list | grep -E "(scan|camera|imu)"
   ```
3. Verify the quality of sensor data:
   ```bash
   ros2 topic echo /scan
   ros2 topic echo /camera/depth/image_raw
   ros2 topic echo /imu
   ```
4. Use RViz2 to visualize sensor data

## Troubleshooting

- If sensors don't appear in Gazebo, check that the plugin filenames are correct
- If topics aren't publishing, verify ROS 2 parameter mappings in the plugins
- If sensor data looks incorrect, review the coordinate frames and transformations