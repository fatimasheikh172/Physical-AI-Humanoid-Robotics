#!/usr/bin/env python3
"""
Perception node validation test for the Digital Twin project.

This test validates that perception nodes are processing sensor data correctly.
"""
import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, PointCloud2, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Time
import time
import math


class PerceptionValidationTest(Node):
    def __init__(self):
        super().__init__('perception_validation_test')
        
        # Initialize flags and data storage
        self.obstacle_detected = False
        self.obstacle_count = 0
        self.perception_output_received = False
        self.perception_output_count = 0
        
        # Store sample outputs for analysis
        self.marker_samples = []
        
        # Create subscribers for perception outputs
        self.obstacle_sub = self.create_subscription(
            MarkerArray,
            '/perception/obstacles',  # Typical topic for obstacle detection output
            self.obstacle_callback,
            10
        )
        
        # Also subscribe to other common perception outputs
        self.perception_sub = self.create_subscription(
            MarkerArray,
            '/perception/markers',  # General perception output
            self.perception_callback,
            10
        )
        
        # Timeout for tests
        self.test_timeout = 15.0  # seconds

    def obstacle_callback(self, msg):
        """Callback for obstacle detection output."""
        if len(msg.markers) > 0:
            self.obstacle_detected = True
            self.obstacle_count += len(msg.markers)
            
            # Store the markers to analyze later
            for marker in msg.markers:
                self.marker_samples.append(marker)
                
        self.get_logger().debug(f"Obstacle detection message received, {len(msg.markers)} obstacles")

    def perception_callback(self, msg):
        """Callback for general perception output."""
        if len(msg.markers) > 0:
            self.perception_output_received = True
            self.perception_output_count += len(msg.markers)
            
            # Store the markers to analyze later
            for marker in msg.markers:
                if marker not in self.marker_samples:  # Avoid duplicates
                    self.marker_samples.append(marker)
        
        self.get_logger().debug(f"Perception message received, {len(msg.markers)} markers")

    def validate_perception_output(self):
        """Validate perception node outputs."""
        # Check that we received some perception data
        if self.perception_output_count == 0 and self.obstacle_count == 0:
            self.get_logger().error("No perception output received")
            return False
        
        self.get_logger().info(f"Perception Analysis:")
        self.get_logger().info(f"  Obstacle detections: {self.obstacle_count}")
        self.get_logger().info(f"  General perception outputs: {self.perception_output_count}")
        self.get_logger().info(f"  Total markers analyzed: {len(self.marker_samples)}")
        
        # Validate marker properties if any were received
        valid_markers = 0
        invalid_markers = 0
        
        for marker in self.marker_samples:
            # Check if marker has valid properties
            marker_valid = True
            
            # Check header timestamp
            if marker.header.stamp.sec == 0 and marker.header.stamp.nanosec == 0:
                self.get_logger().warn(f"Marker {marker.id} has zero timestamp")
                # This may be expected in some cases, so don't count as invalid
            
            # Check position
            if (math.isnan(marker.pose.position.x) or 
                math.isnan(marker.pose.position.y) or 
                math.isnan(marker.pose.position.z)):
                self.get_logger().error(f"Marker {marker.id} has invalid position")
                marker_valid = False
            
            # Check orientation (quaternion should be normalized)
            norm = math.sqrt(
                marker.pose.orientation.x**2 + 
                marker.pose.orientation.y**2 + 
                marker.pose.orientation.z**2 + 
                marker.pose.orientation.w**2
            )
            if not (0.9 < norm < 1.1):  # Allow some tolerance
                self.get_logger().warn(f"Marker {marker.id} has non-normalized quaternion")
                # This may be OK depending on usage, so just warn
            
            # Check scale
            if (marker.scale.x <= 0 or marker.scale.y <= 0 or marker.scale.z <= 0):
                self.get_logger().error(f"Marker {marker.id} has non-positive scale")
                marker_valid = False
            
            # Check color
            if (marker.color.r < 0 or marker.color.r > 1 or
                marker.color.g < 0 or marker.color.g > 1 or
                marker.color.b < 0 or marker.color.b > 1 or
                marker.color.a < 0 or marker.color.a > 1):
                self.get_logger().warn(f"Marker {marker.id} has color values out of range [0,1]")
                # Just warn, as this doesn't necessarily invalidate the detection
            
            if marker_valid:
                valid_markers += 1
            else:
                invalid_markers += 1
        
        self.get_logger().info(f"  Valid markers: {valid_markers}")
        self.get_logger().info(f"  Invalid markers: {invalid_markers}")
        
        # For perception to be working, we need at least some valid outputs
        if valid_markers > 0:
            self.get_logger().info("  ✓ Perception validation passed")
            return True
        else:
            self.get_logger().info("  ✗ Perception validation failed - no valid markers")
            return False

    def test_perception_node_validation(self):
        """
        Test that perception nodes are processing sensor data correctly.
        
        Checks:
        1. Perception nodes publish output based on sensor inputs
        2. Output messages have valid content and format
        3. Perceived objects have reasonable properties
        """
        self.get_logger().info("Starting perception node validation test...")
        self.get_logger().info(f"Waiting for perception outputs for {self.test_timeout} seconds...")
        
        # Wait for perception outputs
        start_time = time.time()
        while time.time() - start_time < self.test_timeout:
            time.sleep(0.1)
        
        # Validate the perception outputs we received
        result = self.validate_perception_output()
        
        if result:
            self.get_logger().info("✓ Perception node validation test PASSED")
            return True
        else:
            self.get_logger().info("✗ Perception node validation test FAILED")
            return False


def main():
    rclpy.init()
    
    try:
        test_node = PerceptionValidationTest()
        
        # Run the perception node validation test
        result = test_node.test_perception_node_validation()
        
        if result:
            print("\n✓ Perception node validation test completed successfully!")
            exit_code = 0
        else:
            print("\n✗ Perception node validation test failed!")
            exit_code = 1
            
    except KeyboardInterrupt:
        print("Test interrupted by user")
        exit_code = 1
    finally:
        rclpy.shutdown()
        
    return exit_code


if __name__ == '__main__':
    exit(main())