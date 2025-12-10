"""
AI-Robot Brain Package for Isaac Sim Integration

This package implements the AI-Robot Brain using NVIDIA Isaac ecosystem for advanced perception and training.
It includes Isaac Sim for photorealistic simulation and synthetic data generation, 
Isaac ROS for hardware-accelerated VSLAM and navigation, and Nav2 adapted for bipedal humanoid movement.
"""

from .isaac_sim_bridge import IsaacSimBridgeNode
from .perception.perception_node import IsaacPerceptionNode
from .vslam.vslam_node import IsaacVSLAMNode
from .synthetic_data.synthetic_generator import SyntheticDatasetGenerator
from .perception.training.perception_training_node import PerceptionTrainingNode

__version__ = "0.1.0"
__author__ = "AI-Native Book Team"

# Package initialization
def initialize_nodes():
    """Initialize core nodes for the AI-Robot Brain package"""
    print("AI-Robot Brain package initialized successfully")
    print("Features available: Isaac Sim Bridge, Perception, VSLAM, Synthetic Data Generation, Training")