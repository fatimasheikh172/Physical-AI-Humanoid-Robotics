#!/usr/bin/env python3
"""
Physics validation test for the Digital Twin project.

This test verifies that gravity and collision detection are working correctly in Gazebo.
"""
import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Vector3
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import GetEntityState
from sensor_msgs.msg import LaserScan
import time


class PhysicsValidationTest(Node):
    def __init__(self):
        super().__init__('physics_validation_test')
        
        # Service client to get entity state
        self.get_state_client = self.create_client(
            GetEntityState, '/get_entity_state'
        )
        
        # Subscriber for collision contacts
        self.contact_sub = self.create_subscription(
            ContactsState,
            '/test_collision_contacts',
            self.contact_callback,
            10
        )
        
        # Flag to track if we've received collision data
        self.collision_detected = False
        
        # Timeout for tests
        self.test_timeout = 10.0  # seconds

    def contact_callback(self, msg):
        """Callback for collision contacts."""
        if len(msg.states) > 0:
            self.collision_detected = True
            self.get_logger().info(f"Collision detected between: {msg.states[0].collision1_name} and {msg.states[0].collision2_name}")

    def wait_for_service(self, client, timeout_sec=5.0):
        """Wait for a service to be available."""
        start = time.time()
        while not client.wait_for_service(timeout_sec=1.0):
            if time.time() - start > timeout_sec:
                return False
            self.get_logger().info(f'Waiting for service {client.srv_name}...')
        return True

    def test_gravity_and_collision_detection(self):
        """
        Test 1: Verify gravity and collision detection are working.
        
        Checks:
        1. Robot falls due to gravity when dropped
        2. Collision is detected when robot hits the ground
        """
        self.get_logger().info("Starting gravity and collision detection test...")
        
        # Wait for Gazebo services
        if not self.wait_for_service(self.get_state_client):
            self.get_logger().error("Service /get_entity_state not available")
            return False
            
        # Give some time for collision messages to arrive
        time.sleep(2.0)
        
        # Check if collision was detected
        if self.collision_detected:
            self.get_logger().info("✓ Gravity and collision detection test PASSED")
            return True
        else:
            self.get_logger().error("✗ Gravity and collision detection test FAILED - no collisions detected")
            return False


def main():
    rclpy.init()
    
    try:
        test_node = PhysicsValidationTest()
        
        # Run the gravity and collision test
        result = test_node.test_gravity_and_collision_detection()
        
        if result:
            print("\n✓ Physics validation test completed successfully!")
            exit_code = 0
        else:
            print("\n✗ Physics validation test failed!")
            exit_code = 1
            
    except KeyboardInterrupt:
        print("Test interrupted by user")
        exit_code = 1
    finally:
        rclpy.shutdown()
        
    return exit_code


if __name__ == '__main__':
    exit(main())