#!/usr/bin/env python3
"""
UI telemetry display test for the Digital Twin project.

This test verifies that telemetry data is displayed correctly in the Unity UI.
While this test primarily runs on the ROS side, it verifies that the data
expected by the Unity UI is being published correctly.
"""
import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Imu, BatteryState
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
import time


class TelemetryDisplayTest(Node):
    def __init__(self):
        super().__init__('telemetry_display_test')
        
        # Initialize flags to track telemetry data reception
        self.odom_received = False
        self.imu_received = False
        self.battery_received = False
        self.velocity_received = False
        self.position_received = False
        
        # Message counters
        self.odom_count = 0
        self.imu_count = 0
        self.battery_count = 0
        self.velocity_count = 0
        self.position_count = 0
        
        # Store latest values
        self.latest_odom = None
        self.latest_imu = None
        self.latest_battery = None
        self.latest_velocity = None
        
        # Create subscribers for telemetry data
        self.odom_sub = self.create_subscription(
            Odometry,
            '/robot/odom',
            self.odom_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/robot/imu/data',
            self.imu_callback,
            10
        )
        
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/robot/battery',
            self.battery_callback,
            10
        )
        
        # For velocity, we'll listen to cmd_vel as well
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/robot/cmd_vel',
            self.velocity_callback,
            10
        )
        
        # Timeout for tests
        self.test_timeout = 10.0  # seconds

    def odom_callback(self, msg):
        """Callback for odometry data."""
        self.odom_received = True
        self.odom_count += 1
        self.latest_odom = msg
        self.get_logger().debug(f"Odometry message received, count: {self.odom_count}")

    def imu_callback(self, msg):
        """Callback for IMU data."""
        self.imu_received = True
        self.imu_count += 1
        self.latest_imu = msg
        self.get_logger().debug(f"IMU message received, count: {self.imu_count}")

    def battery_callback(self, msg):
        """Callback for battery data."""
        self.battery_received = True
        self.battery_count += 1
        self.latest_battery = msg
        self.get_logger().debug(f"Battery message received, count: {self.battery_count}")

    def velocity_callback(self, msg):
        """Callback for velocity commands."""
        self.velocity_received = True
        self.velocity_count += 1
        self.latest_velocity = msg
        self.get_logger().debug(f"Velocity command received, count: {self.velocity_count}")

    def test_ui_telemetry_display(self):
        """
        Test that telemetry data required for UI display is available.
        
        Checks:
        1. Odometry data is published (for position/pose display)
        2. IMU data is published (for orientation display)
        3. Battery data is published (for battery level indicator)
        4. Velocity data is published (for speed display)
        """
        self.get_logger().info("Starting UI telemetry display test...")
        self.get_logger().info("Checking for telemetry data availability...")
        
        # Wait for 10 seconds to collect telemetry data
        start_time = time.time()
        while time.time() - start_time < self.test_timeout:
            time.sleep(0.1)
            
            # Check if we've received all required telemetry data
            if (self.odom_received and self.imu_received and 
                self.battery_received and self.velocity_received):
                self.get_logger().info("All telemetry data received, stopping early...")
                break
        
        # Print summary
        self.get_logger().info(f"Telemetry message counts:")
        self.get_logger().info(f"  Odometry: {self.odom_count}")
        self.get_logger().info(f"  IMU: {self.imu_count}")
        self.get_logger().info(f"  Battery: {self.battery_count}")
        self.get_logger().info(f"  Velocity: {self.velocity_count}")
        
        # Verify data quality
        data_quality_issues = []
        
        # Check if odometry has valid pose
        if self.latest_odom and self.latest_odom.pose:
            pose = self.latest_odom.pose.pose
            # Check if position values are reasonable
            if not (-1000 < pose.position.x < 1000 and 
                    -1000 < pose.position.y < 1000 and 
                    -1000 < pose.position.z < 1000):
                data_quality_issues.append("Odometry position values seem unreasonable")
        
        # Check if IMU has valid orientation
        if self.latest_imu:
            orientation = self.latest_imu.orientation
            # Check if quaternion values are normalized (or close to it)
            norm = orientation.x**2 + orientation.y**2 + orientation.z**2 + orientation.w**2
            if abs(norm - 1.0) > 0.1:  # Allow some tolerance
                data_quality_issues.append("IMU orientation quaternion not normalized")
        
        # Check if battery data seems reasonable
        if self.latest_battery:
            battery = self.latest_battery
            if battery.voltage < 0 or battery.percentage < 0 or battery.percentage > 100:
                data_quality_issues.append("Battery data values seem unreasonable")
        
        # Check if we have at least basic data for UI
        all_data_present = (
            self.odom_received and 
            self.imu_received and 
            self.battery_received and 
            self.velocity_received
        )
        
        if all_data_present and not data_quality_issues:
            self.get_logger().info("✓ UI telemetry display test PASSED")
            return True
        else:
            if not all_data_present:
                missing_data = []
                if not self.odom_received: missing_data.append("Odometry")
                if not self.imu_received: missing_data.append("IMU")
                if not self.battery_received: missing_data.append("Battery")
                if not self.velocity_received: missing_data.append("Velocity")
                
                self.get_logger().error(f"Missing telemetry data: {', '.join(missing_data)}")
            
            if data_quality_issues:
                for issue in data_quality_issues:
                    self.get_logger().error(f"Data quality issue: {issue}")
            
            self.get_logger().error("✗ UI telemetry display test FAILED")
            return False


def main():
    rclpy.init()
    
    try:
        test_node = TelemetryDisplayTest()
        
        # Run the UI telemetry display test
        result = test_node.test_ui_telemetry_display()
        
        if result:
            print("\n✓ UI telemetry display test completed successfully!")
            exit_code = 0
        else:
            print("\n✗ UI telemetry display test failed!")
            exit_code = 1
            
    except KeyboardInterrupt:
        print("Test interrupted by user")
        exit_code = 1
    finally:
        rclpy.shutdown()
        
    return exit_code


if __name__ == '__main__':
    exit(main())