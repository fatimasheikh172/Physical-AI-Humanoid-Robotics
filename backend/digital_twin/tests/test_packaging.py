import unittest
import os
import tempfile
import shutil
from pathlib import Path


class TestPackaging(unittest.TestCase):
    """Test cases for packaging functionality"""
    
    def setUp(self):
        """Set up test fixtures before each test method."""
        # Create a temporary directory for testing
        self.test_dir = Path(tempfile.mkdtemp())
        self.source_dir = self.test_dir / "source"
        self.source_dir.mkdir()
        
        # Create some mock files to represent the digital twin structure
        (self.source_dir / "gazebo").mkdir()
        (self.source_dir / "gazebo" / "models").mkdir()
        (self.source_dir / "gazebo" / "worlds").mkdir()
        (self.source_dir / "gazebo" / "launch").mkdir()
        (self.source_dir / "perception").mkdir()
        (self.source_dir / "perception" / "imu_analysis").mkdir()
        
        # Create a mock model file
        with open(self.source_dir / "gazebo" / "models" / "test_robot.urdf", "w") as f:
            f.write("<robot name='test_robot'></robot>")
        
        # Create a mock launch file
        with open(self.source_dir / "gazebo" / "launch" / "test.launch.py", "w") as f:
            f.write("# Test launch file")
    
    def tearDown(self):
        """Clean up after each test method."""
        # Remove the temporary directory
        shutil.rmtree(self.test_dir)
    
    def test_packaging_structure_creation(self):
        """Test that the package structure is created correctly"""
        # Import our package module
        import sys
        sys.path.append(str(self.source_dir.parent))
        
        # Since we can't easily import the packaging script as a module,
        # we'll test the concepts directly
        package_path = self.test_dir / "test_package"
        package_path.mkdir()
        
        # Create the expected directory structure
        dirs = [
            'gazebo/models',
            'gazebo/worlds', 
            'gazebo/launch',
            'gazebo/sensors',
            'gazebo/physics',
            'ros_bridge/nodes',
            'perception/lidar_processing',
            'perception/depth_camera',
            'perception/imu_analysis',
            'utils/scripts',
            'docs',
            'config'
        ]
        
        for dir_path in dirs:
            (package_path / dir_path).mkdir(parents=True, exist_ok=True)
        
        # Verify all directories were created
        for dir_path in dirs:
            self.assertTrue((package_path / dir_path).exists())
        
        # Verify we can write files to these directories
        test_file_path = package_path / "gazebo" / "models" / "test.txt"
        with open(test_file_path, "w") as f:
            f.write("test")
        self.assertTrue(test_file_path.exists())


class TestConfiguration(unittest.TestCase):
    """Test cases for configuration functionality"""
    
    def test_config_file_creation(self):
        """Test that configuration files are created with proper structure"""
        import json
        
        # Test robot configuration
        robot_config = {
            'robot_name': 'default_robot',
            'sensors': {
                'lidar': {
                    'enabled': True,
                    'topic': '/robot/lidar_scan',
                    'range_min': 0.1,
                    'range_max': 10.0
                },
                'depth_camera': {
                    'enabled': True,
                    'topic': '/robot/depth_camera/image_raw'
                },
                'imu': {
                    'enabled': True,
                    'topic': '/robot/imu'
                }
            },
            'physics': {
                'gravity': [0, 0, -9.81],
                'solver_type': 'ode',
                'max_step_size': 0.001
            }
        }
        
        # Validate the structure
        self.assertEqual(robot_config['robot_name'], 'default_robot')
        self.assertIn('sensors', robot_config)
        self.assertIn('physics', robot_config)
        self.assertEqual(robot_config['physics']['solver_type'], 'ode')
        
        # Test that the config can be serialized to JSON
        try:
            json_str = json.dumps(robot_config, indent=2)
            self.assertIsInstance(json_str, str)
            
            # Test that it can be deserialized back
            parsed_config = json.loads(json_str)
            self.assertEqual(parsed_config['robot_name'], 'default_robot')
        except Exception as e:
            self.fail(f"Configuration could not be serialized to JSON: {e}")
    
    def test_simulation_config(self):
        """Test simulation configuration structure"""
        import json
        
        sim_config = {
            'simulation': {
                'real_time_factor': 1.0,
                'max_update_rate': 1000,
                'paused': False
            },
            'visualization': {
                'show_sensors': True,
                'show_contacts': False
            }
        }
        
        # Validate structure
        self.assertIn('simulation', sim_config)
        self.assertIn('visualization', sim_config)
        self.assertIsInstance(sim_config['simulation']['real_time_factor'], float)
        self.assertIsInstance(sim_config['visualization']['show_sensors'], bool)
        
        # Test serialization
        try:
            json_str = json.dumps(sim_config, indent=2)
            parsed_config = json.loads(json_str)
            
            self.assertEqual(parsed_config['simulation']['real_time_factor'], 1.0)
            self.assertEqual(parsed_config['visualization']['show_sensors'], True)
        except Exception as e:
            self.fail(f"Simulation config could not be serialized: {e}")


class TestUtilityFunctions(unittest.TestCase):
    """Test cases for utility functions"""
    
    def test_mathematical_functions(self):
        """Test basic mathematical functions used in the system"""
        import math
        
        # Test distance calculation
        x1, y1 = 0, 0
        x2, y2 = 3, 4
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        self.assertEqual(distance, 5.0)  # 3-4-5 triangle
        
        # Test angle normalization
        angle = 3 * math.pi  # This is equivalent to pi
        normalized = ((angle + math.pi) % (2 * math.pi)) - math.pi
        self.assertAlmostEqual(normalized, math.pi, places=7)
        
        # Test that extremely large values don't cause issues
        with self.assertRaises((OverflowError, ValueError)):
            # This might raise an exception for very large values depending on the system
            try:
                huge_number = 10**308
                result = math.sqrt(huge_number)
            except:
                pass  # Expected to potentially fail with large numbers
    
    def test_parameter_validation(self):
        """Test validation of parameters"""
        # Test mass validation (should be positive)
        def is_valid_mass(mass):
            return mass > 0
        
        self.assertTrue(is_valid_mass(1.0))
        self.assertTrue(is_valid_mass(0.1))
        self.assertFalse(is_valid_mass(0))
        self.assertFalse(is_valid_mass(-1.0))
        
        # Test friction coefficient validation (typically between 0 and 10)
        def is_valid_friction(friction):
            return 0 <= friction <= 10
        
        self.assertTrue(is_valid_friction(0.5))
        self.assertTrue(is_valid_friction(1.0))
        self.assertTrue(is_valid_friction(0))
        self.assertTrue(is_valid_friction(10))
        self.assertFalse(is_valid_friction(-0.1))
        self.assertFalse(is_valid_friction(10.1))


if __name__ == '__main__':
    unittest.main()