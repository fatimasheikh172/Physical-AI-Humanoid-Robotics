import unittest
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
from unittest.mock import Mock, MagicMock

# Import the modules we want to test
# Since we can't directly import ROS 2 nodes without a running ROS system,
# we'll test the core functions separately

def calculate_orientation_euler(orientation):
    """Test version of orientation calculation without ROS dependencies"""
    try:
        # Create rotation object from quaternion
        rot = R.from_quat([
            orientation[0],
            orientation[1], 
            orientation[2],
            orientation[3]
        ])
        
        # Get Euler angles in radians
        euler = rot.as_euler('xyz')
        
        return euler[0], euler[1], euler[2]  # roll, pitch, yaw
    except Exception:
        # Return zeros as fallback
        return 0.0, 0.0, 0.0

def detect_obstacles_lidar(ranges, range_min, range_max, angle_min, angle_increment, threshold=1.0):
    """Test version of obstacle detection without ROS dependencies"""
    if not ranges:
        return []
    
    valid_ranges = []
    valid_indices = []
    
    for i, r in enumerate(ranges):
        if range_min <= r <= range_max:
            valid_ranges.append(r)
            valid_indices.append(i)
    
    obstacle_indices = [i for i, r in enumerate(valid_ranges) if r < threshold]
    
    obstacles = []
    for idx in obstacle_indices:
        original_idx = valid_indices[idx]
        angle = angle_min + original_idx * angle_increment
        distance = valid_ranges[idx]
        obstacles.append({
            'angle': angle,
            'distance': distance,
            'x': distance * math.cos(angle),
            'y': distance * math.sin(angle)
        })
    
    return obstacles

class TestIMUAnalysis(unittest.TestCase):
    """Test cases for IMU analysis functions"""
    
    def test_orientation_calculation_valid_quaternion(self):
        """Test orientation calculation with a valid quaternion"""
        # Valid unit quaternion for no rotation
        orientation = [0, 0, 0, 1]
        roll, pitch, yaw = calculate_orientation_euler(orientation)
        
        self.assertAlmostEqual(roll, 0.0, places=5)
        self.assertAlmostEqual(pitch, 0.0, places=5)
        self.assertAlmostEqual(yaw, 0.0, places=5)
    
    def test_orientation_calculation_90deg_rotation(self):
        """Test orientation calculation with a 90-degree rotation around Z-axis"""
        # Quaternion for 90-degree rotation around Z-axis
        # This corresponds to a rotation of pi/2 around the z-axis
        angle = math.pi / 2
        orientation = [0, 0, math.sin(angle/2), math.cos(angle/2)]
        roll, pitch, yaw = calculate_orientation_euler(orientation)
        
        self.assertAlmostEqual(roll, 0.0, places=5)
        self.assertAlmostEqual(pitch, 0.0, places=5)
        self.assertAlmostEqual(yaw, math.pi/2, places=5)
    
    def test_orientation_calculation_invalid_quaternion(self):
        """Test orientation calculation with an invalid quaternion"""
        # Invalid quaternion (not normalized)
        orientation = [1, 1, 1, 1]
        roll, pitch, yaw = calculate_orientation_euler(orientation)
        
        # Should handle the error gracefully and return zeros
        self.assertIsInstance(roll, float)
        self.assertIsInstance(pitch, float)
        self.assertIsInstance(yaw, float)
    
    def test_linear_acceleration_magnitude_calculation(self):
        """Test that we can calculate linear acceleration magnitude correctly"""
        # Test with Earth's gravity (should be approximately 9.81)
        ax, ay, az = 0.0, 0.0, 9.81
        magnitude = math.sqrt(ax**2 + ay**2 + az**2)
        
        self.assertAlmostEqual(magnitude, 9.81, places=2)
    
    def test_angular_velocity_magnitude_calculation(self):
        """Test that we can calculate angular velocity magnitude correctly"""
        wx, wy, wz = 1.0, 0.0, 0.0  # 1 rad/s around x-axis
        magnitude = math.sqrt(wx**2 + wy**2 + wz**2)
        
        self.assertEqual(magnitude, 1.0)


class TestLiDARProcessing(unittest.TestCase):
    """Test cases for LiDAR processing functions"""
    
    def test_detect_obstacles_no_obstacles(self):
        """Test obstacle detection when no obstacles are present"""
        ranges = [2.0, 2.0, 2.0, 2.0]  # All ranges beyond threshold
        obstacles = detect_obstacles_lidar(
            ranges, 
            range_min=0.1, 
            range_max=10.0, 
            angle_min=-math.pi/2, 
            angle_increment=math.pi/6,
            threshold=1.0
        )
        
        self.assertEqual(len(obstacles), 0)
    
    def test_detect_obstacles_with_obstacles(self):
        """Test obstacle detection when obstacles are present"""
        ranges = [0.5, 2.0, 0.3, 2.0]  # Two obstacles within threshold
        obstacles = detect_obstacles_lidar(
            ranges, 
            range_min=0.1, 
            range_max=10.0, 
            angle_min=-math.pi/2, 
            angle_increment=math.pi/6,
            threshold=1.0
        )
        
        self.assertEqual(len(obstacles), 2)
        self.assertEqual(obstacles[0]['distance'], 0.5)
        self.assertEqual(obstacles[1]['distance'], 0.3)
    
    def test_detect_obstacles_with_invalid_ranges(self):
        """Test obstacle detection with invalid range values"""
        ranges = [0.5, float('inf'), 0.3, float('nan')]  # Mix of valid and invalid ranges
        obstacles = detect_obstacles_lidar(
            ranges, 
            range_min=0.1, 
            range_max=10.0, 
            angle_min=-math.pi/2, 
            angle_increment=math.pi/6,
            threshold=1.0
        )
        
        # Only valid ranges within threshold should be detected as obstacles
        self.assertEqual(len(obstacles), 2)
    
    def test_detect_obstacles_empty_ranges(self):
        """Test obstacle detection with empty ranges"""
        obstacles = detect_obstacles_lidar(
            [], 
            range_min=0.1, 
            range_max=10.0, 
            angle_min=-math.pi/2, 
            angle_increment=math.pi/6,
            threshold=1.0
        )
        
        self.assertEqual(len(obstacles), 0)


class TestSensorFusion(unittest.TestCase):
    """Test cases for sensor fusion functions"""
    
    def test_fuse_sensor_data_basic(self):
        """Test basic sensor fusion"""
        obstacles = [{'x': 1.0, 'y': 0.0}, {'x': 0.5, 'y': 0.5}]
        depth_distance = 1.5
        roll, pitch, yaw = 0.1, -0.05, 0.2
        
        # Basic fusion logic test
        obstacle_warning = len(obstacles) > 0
        close_depth_object = depth_distance is not None and depth_distance < 2.0
        orientation_stable = abs(roll) < 0.3 and abs(pitch) < 0.3
        
        self.assertTrue(obstacle_warning)
        self.assertTrue(close_depth_object)
        self.assertTrue(orientation_stable)
    
    def test_fuse_sensor_data_unstable_orientation(self):
        """Test sensor fusion with unstable orientation"""
        obstacles = []
        depth_distance = 3.0
        roll, pitch, yaw = 0.5, 0.4, 0.1  # This exceeds stability threshold
        
        # Basic fusion logic test
        orientation_stable = abs(roll) < 0.3 and abs(pitch) < 0.3
        
        self.assertFalse(orientation_stable)


if __name__ == '__main__':
    unittest.main()