#!/usr/bin/env python3

"""
Domain Randomization Features for Digital Twin System

This module implements domain randomization techniques to reduce the sim-to-real gap
by introducing controlled variations in simulation parameters during training.
"""

import numpy as np
import random
import yaml
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional, Any
import argparse
import os


@dataclass
class DomainRandomizationParams:
    """Parameters for domain randomization"""
    # Visual properties
    light_intensity_range: Tuple[float, float] = (0.5, 1.5)
    light_position_variance: float = 0.5
    texture_randomization: bool = True
    color_randomization: bool = True
    
    # Physical properties
    friction_range: Tuple[float, float] = (0.3, 0.8)
    mass_variance: float = 0.2  # ±20%
    damping_variance: float = 0.1  # ±10%
    
    # Sensor properties
    sensor_noise_range: Tuple[float, float] = (0.01, 0.05)
    sensor_bias_range: Tuple[float, float] = (-0.02, 0.02)
    sensor_dropout_rate: float = 0.01  # 1% chance of sensor dropout
    
    # Environmental properties
    gravity_variance: float = 0.1  # ±10%
    wind_force_range: Tuple[float, float] = (-0.5, 0.5)
    object_position_variance: float = 0.1  # meters


class DomainRandomizer:
    def __init__(self, robot_name: str = "default_robot"):
        self.robot_name = robot_name
        self.params = DomainRandomizationParams()
        self.randomization_step = 0
        self.applied_parameters = {}
        
    def randomize_lighting(self) -> Dict[str, Any]:
        """Randomize lighting conditions in the simulation"""
        intensity = np.random.uniform(*self.params.light_intensity_range)
        position_variance = np.random.normal(0, self.params.light_position_variance, 3)
        
        lighting_params = {
            'intensity': intensity,
            'position_offset': position_variance.tolist(),
            'color_temperature': np.random.uniform(5000, 8000)  # Kelvin
        }
        
        self.applied_parameters['lighting'] = lighting_params
        return lighting_params
    
    def randomize_visual_textures(self) -> Dict[str, Any]:
        """Randomize visual textures and appearance"""
        if not self.params.texture_randomization:
            return {}
        
        texture_params = {
            'roughness_range': np.random.uniform(0.1, 0.9, 2).tolist(),
            'metallic_range': np.random.uniform(0.0, 0.5, 2).tolist(),
            'normal_map_strength': np.random.uniform(0.5, 1.5)
        }
        
        self.applied_parameters['textures'] = texture_params
        return texture_params
    
    def randomize_colors(self) -> Dict[str, Any]:
        """Randomize colors of objects in the scene"""
        if not self.params.color_randomization:
            return {}
        
        color_params = {
            'object_colors': [
                np.random.uniform(0, 1, 3).tolist()  # RGB for each object
                for _ in range(5)  # Randomize up to 5 objects
            ],
            'background_color': np.random.uniform(0, 1, 3).tolist()
        }
        
        self.applied_parameters['colors'] = color_params
        return color_params
    
    def randomize_physics_properties(self, base_params: Dict[str, float]) -> Dict[str, float]:
        """Randomize physics properties of objects"""
        randomized_params = base_params.copy()
        
        # Randomize friction
        base_friction = base_params.get('friction', 0.5)
        friction = np.random.uniform(*self.params.friction_range)
        randomized_params['friction'] = friction
        
        # Randomize mass with variance
        base_mass = base_params.get('mass', 1.0)
        mass_variance = np.random.uniform(1 - self.params.mass_variance, 
                                          1 + self.params.mass_variance)
        randomized_params['mass'] = base_mass * mass_variance
        
        # Randomize damping
        base_damping = base_params.get('damping', 0.1)
        damping_variance = np.random.uniform(1 - self.params.damping_variance, 
                                             1 + self.params.damping_variance)
        randomized_params['damping'] = base_damping * damping_variance
        
        self.applied_parameters['physics'] = randomized_params
        return randomized_params
    
    def randomize_sensor_properties(self, base_sensor_params: Dict[str, Any]) -> Dict[str, Any]:
        """Randomize sensor properties to simulate real sensor variations"""
        randomized_params = base_sensor_params.copy()
        
        # Add noise to sensor readings
        noise_level = np.random.uniform(*self.params.sensor_noise_range)
        randomized_params['noise_level'] = noise_level
        
        # Add bias to sensor readings
        bias = np.random.uniform(*self.params.sensor_bias_range)
        randomized_params['bias'] = bias
        
        # Randomize dropout rate
        dropout_rate = self.params.sensor_dropout_rate
        randomized_params['dropout_rate'] = dropout_rate
        
        self.applied_parameters['sensors'] = randomized_params
        return randomized_params
    
    def randomize_environment(self) -> Dict[str, Any]:
        """Randomize environmental conditions"""
        # Randomize gravity
        base_gravity = 9.81  # m/s^2
        gravity_variance = np.random.uniform(1 - self.params.gravity_variance, 
                                             1 + self.params.gravity_variance)
        gravity = base_gravity * gravity_variance
        
        # Randomize wind force
        wind_force = np.random.uniform(*self.params.wind_force_range, 3)
        
        env_params = {
            'gravity': gravity,
            'wind_force': wind_force.tolist(),
            'floor_friction': np.random.uniform(*self.params.friction_range)
        }
        
        self.applied_parameters['environment'] = env_params
        return env_params
    
    def randomize_object_positions(self, base_positions: List[Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
        """Randomize positions of objects in the scene"""
        randomized_positions = []
        
        for pos in base_positions:
            offset = np.random.normal(0, self.params.object_position_variance, 3)
            new_pos = (pos[0] + offset[0], pos[1] + offset[1], pos[2] + offset[2])
            randomized_positions.append(new_pos)
        
        self.applied_parameters['object_positions'] = randomized_positions
        return randomized_positions
    
    def apply_randomization(self, simulation_interface) -> Dict[str, Any]:
        """
        Apply all randomization techniques to the simulation.
        This would interface with Gazebo/Unity in a real implementation.
        """
        print(f"Applying domain randomization step {self.randomization_step}")
        
        # Collect all randomization changes
        all_changes = {}
        
        # Visual randomization
        all_changes['lighting'] = self.randomize_lighting()
        all_changes['textures'] = self.randomize_visual_textures()
        all_changes['colors'] = self.randomize_colors()
        
        # Physics randomization
        base_physics = {
            'friction': 0.5,
            'mass': 10.0,
            'damping': 0.1
        }
        all_changes['physics'] = self.randomize_physics_properties(base_physics)
        
        # Sensor randomization
        base_sensors = {
            'noise_level': 0.01,
            'bias': 0.0,
            'dropout_rate': 0.01
        }
        all_changes['sensors'] = self.randomize_sensor_properties(base_sensors)
        
        # Environment randomization
        all_changes['environment'] = self.randomize_environment()
        
        # Object position randomization
        base_positions = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0)]
        all_changes['object_positions'] = self.randomize_object_positions(base_positions)
        
        # In a real implementation, we would apply these changes to the simulation
        # For now, we'll just print them
        self.randomization_step += 1
        
        return all_changes
    
    def save_randomization_config(self, filepath: str):
        """Save current randomization configuration to file"""
        config = {
            'robot_name': self.robot_name,
            'parameters': self.params.__dict__,
            'applied_parameters': self.applied_parameters,
            'randomization_step': self.randomization_step
        }
        
        with open(filepath, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
        
        print(f"Randomization config saved to {filepath}")
    
    def load_randomization_config(self, filepath: str):
        """Load randomization configuration from file"""
        with open(filepath, 'r') as f:
            config = yaml.safe_load(f)
        
        self.robot_name = config['robot_name']
        self.applied_parameters = config['applied_parameters']
        self.randomization_step = config['randomization_step']
        
        # Update parameters
        for key, value in config['parameters'].items():
            if hasattr(self.params, key):
                setattr(self.params, key, value)


class DomainRandomizationManager:
    """Manager class to handle multiple domain randomization strategies"""
    def __init__(self, robot_name: str = "default_robot"):
        self.randomizer = DomainRandomizer(robot_name)
        self.strategies = []
        
    def add_strategy(self, strategy_func, name: str, frequency: int = 1):
        """Add a randomization strategy with a frequency (every N steps)"""
        self.strategies.append({
            'func': strategy_func,
            'name': name,
            'frequency': frequency,
            'step_count': 0
        })
    
    def apply_strategies(self, simulation_interface):
        """Apply all registered strategies based on their frequencies"""
        results = {}
        
        for strategy in self.strategies:
            strategy['step_count'] += 1
            
            if strategy['step_count'] >= strategy['frequency']:
                # Apply the strategy
                result = strategy['func'](self.randomizer, simulation_interface)
                results[strategy['name']] = result
                strategy['step_count'] = 0  # Reset counter
        
        return results


def main():
    parser = argparse.ArgumentParser(description='Domain Randomization for Digital Twin')
    parser.add_argument('--robot-name', type=str, default='turtlebot3',
                        help='Name of the robot model')
    parser.add_argument('--output-config', type=str,
                        help='File to save randomization configuration')
    parser.add_argument('--num-steps', type=int, default=10,
                        help='Number of randomization steps to perform')
    
    args = parser.parse_args()
    
    # Create domain randomizer
    randomizer = DomainRandomizer(robot_name=args.robot_name)
    
    print(f"Starting domain randomization for {args.robot_name}")
    print(f"Parameters: {randomizer.params}")
    
    # Simulate applying randomization for several steps
    for step in range(args.num_steps):
        print(f"\nStep {step + 1}:")
        changes = randomizer.apply_randomization(simulation_interface=None)
        
        # Print some key changes
        print(f"  Physics - Friction: {changes['physics']['friction']:.3f}, "
              f"Mass: {changes['physics']['mass']:.3f} kg")
        print(f"  Sensor - Noise: {changes['sensors']['noise_level']:.4f}, "
              f"Bias: {changes['sensors']['bias']:.4f}")
        print(f"  Environment - Gravity: {changes['environment']['gravity']:.3f} m/s²")
    
    # Save config if specified
    if args.output_config:
        randomizer.save_randomization_config(args.output_config)


if __name__ == '__main__':
    main()