#!/usr/bin/env python3
"""
Transform synchronization test for the Digital Twin project.

This test verifies that transforms are synchronized correctly between Gazebo and Unity.
"""
import unittest
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, Point
from std_msgs.msg import Header
import time
import math


class TransformSyncTest(Node):
    def __init__(self):
        super().__init__('transform_sync_test')
        
        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Store transform data for comparison
        self.gazebo_transforms = {}
        self.unity_transforms = {}
        
        # Timeout for tests
        self.test_timeout = 15.0  # seconds

    def get_transform(self, target_frame, source_frame, timeout=5.0):
        """
        Get transform from source_frame to target_frame.
        Returns transform if found, None otherwise.
        """
        try:
            # Wait for transform to become available
            start_time = time.time()
            while time.time() - start_time < timeout:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        rclpy.time.Time()
                    )
                    return transform
                except Exception as e:
                    time.sleep(0.1)  # Wait a bit before retrying
            return None
        except Exception as e:
            self.get_logger().error(f"Error getting transform: {e}")
            return None

    def test_transform_synchronization(self):
        """
        Test that transforms are synchronized between Gazebo and Unity.
        
        Checks:
        1. Robot transforms in Gazebo match those in Unity
        2. Link positions and orientations are consistent
        3. Coordinate frame transformations are correct
        """
        self.get_logger().info("Starting transform synchronization test...")
        
        # Define expected robot links to check
        robot_links = ['base_link', 'wheel_left_link', 'wheel_right_link', 'lidar_link']
        
        # Wait for transforms to be available
        self.get_logger().info("Waiting for transforms to become available...")
        time.sleep(5.0)
        
        sync_errors = []
        
        for link in robot_links:
            # Get transform from robot base to link in Gazebo frame
            gazebo_tf = self.get_transform('world', f'robot1/{link}', timeout=2.0)
            
            # For Unity, we check the same transform but it might be published with a different prefix
            # Unity typically uses a different frame naming convention
            unity_tf = self.get_transform('world', f'unity_robot1/{link}', timeout=2.0)
            
            # If Unity uses a different naming convention, try alternative
            if unity_tf is None:
                unity_tf = self.get_transform('world', f'unity/{link}', timeout=2.0)
            
            if gazebo_tf is None:
                sync_errors.append(f"Could not get transform for {link} in Gazebo")
                continue
                
            if unity_tf is None:
                # It's possible Unity doesn't publish the same transform names
                # Check if there are Unity-specific transforms related to the robot
                unity_tf = self.get_transform('world', f'UnityRobot/{link}', timeout=2.0)
                
            if unity_tf is None:
                sync_errors.append(f"Could not get transform for {link} in Unity")
                continue
                
            # Extract positions
            gz_pos = gazebo_tf.transform.translation
            unity_pos = unity_tf.transform.translation
            
            # Check if positions are approximately equal (within tolerance)
            pos_diff = math.sqrt(
                (gz_pos.x - unity_pos.x)**2 + 
                (gz_pos.y - unity_pos.y)**2 + 
                (gz_pos.z - unity_pos.z)**2
            )
            
            # Also check orientations (quaternions)
            gz_quat = gazebo_tf.transform.rotation
            unity_quat = unity_tf.transform.rotation
            
            # Calculate quaternion difference (dot product)
            quat_dot = abs(
                gz_quat.x * unity_quat.x + 
                gz_quat.y * unity_quat.y + 
                gz_quat.z * unity_quat.z + 
                gz_quat.w * unity_quat.w
            )
            
            # Check if transforms are similar (position within 5cm, orientation within ~3 degrees)
            pos_threshold = 0.05  # 5cm
            quat_threshold = 0.998  # ~3 degree difference in quaternions
            
            if pos_diff > pos_threshold:
                sync_errors.append(
                    f"Position mismatch for {link}: "
                    f"Gazebo=({gz_pos.x:.3f}, {gz_pos.y:.3f}, {gz_pos.z:.3f}), "
                    f"Unity=({unity_pos.x:.3f}, {unity_pos.y:.3f}, {unity_pos.z:.3f}), "
                    f"diff={pos_diff:.3f}m"
                )
                
            if quat_dot < quat_threshold:
                sync_errors.append(
                    f"Orientation mismatch for {link}: "
                    f"dot product = {quat_dot:.3f} (threshold {quat_threshold})"
                )
        
        # Check if we found any transforms at all
        if not sync_errors and (len(robot_links) > 0):
            self.get_logger().info("✓ Transform synchronization test PASSED")
            return True
        elif len(sync_errors) == 0:
            # No errors, but also no transforms found - this might indicate a different naming scheme
            self.get_logger().warn(
                "No synchronization errors found, but no transforms were checked. "
                "This may indicate different frame naming conventions between Gazebo and Unity."
            )
            return True
        else:
            for error in sync_errors:
                self.get_logger().error(error)
            self.get_logger().error("✗ Transform synchronization test FAILED")
            return False


def main():
    rclpy.init()
    
    try:
        test_node = TransformSyncTest()
        
        # Run the transform synchronization test
        result = test_node.test_transform_synchronization()
        
        if result:
            print("\n✓ Transform synchronization test completed successfully!")
            exit_code = 0
        else:
            print("\n✗ Transform synchronization test failed!")
            exit_code = 1
            
    except KeyboardInterrupt:
        print("Test interrupted by user")
        exit_code = 1
    finally:
        rclpy.shutdown()
        
    return exit_code


if __name__ == '__main__':
    exit(main())