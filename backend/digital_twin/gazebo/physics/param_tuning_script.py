#!/usr/bin/env python3

"""
Parameter Tuning Script for Physics Optimization in Digital Twin System

This script implements optimization algorithms to tune physics parameters
for better simulation fidelity, such as friction coefficients, mass properties,
and damping parameters.
"""

import numpy as np
import matplotlib.pyplot as plt
import yaml
import os
from scipy.optimize import minimize
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
import argparse
import json


@dataclass
class PhysicsParameters:
    """Class to represent physics parameters for a robot model"""
    robot_name: str
    mass: float  # kg
    friction_coefficient: float  # unitless
    damping_linear: float  # Ns/m
    damping_angular: float  # Nms/rad
    gear_ratio: float  # unitless
    max_torque: float  # Nm
    wheel_radius: float  # m
    wheel_base: float  # m (distance between wheels)
    tread_width: float  # m (width of robot)


class PhysicsOptimizer:
    def __init__(self, robot_name: str = "default_robot"):
        self.robot_name = robot_name
        self.initial_params = PhysicsParameters(
            robot_name=robot_name,
            mass=10.0,  # kg
            friction_coefficient=0.5,  # unitless
            damping_linear=0.1,  # Ns/m
            damping_angular=0.1,  # Nms/rad
            gear_ratio=1.0,  # unitless
            max_torque=5.0,  # Nm
            wheel_radius=0.1,  # m
            wheel_base=0.3,  # m
            tread_width=0.25  # m
        )
        
        # Bounds for parameters (min, max)
        self.bounds = [
            (0.1, 50.0),      # mass
            (0.01, 10.0),     # friction coefficient  
            (0.001, 5.0),     # linear damping
            (0.001, 5.0),     # angular damping
            (0.1, 100.0),     # gear ratio
            (0.1, 50.0),      # max torque
            (0.01, 1.0),      # wheel radius
            (0.05, 2.0),      # wheel base
            (0.05, 2.0)       # tread width
        ]
        
        # Store optimization history
        self.optimization_history = []

    def simulate_robot_behavior(self, params: PhysicsParameters) -> Dict[str, float]:
        """
        Simulate robot behavior with given parameters.
        In a real implementation, this would interface with Gazebo simulation.
        For this example, we'll simulate a simplified model.
        """
        # This is a placeholder simulation - in real scenario, this would call Gazebo
        # or another physics simulator to get actual robot behavior
        
        # Simplified model of how parameters affect robot behavior
        # For example, simulating a movement task where we try to reach target position
        
        # Time to reach target (simpler is faster, more friction is slower)
        time_to_target = 5.0 * (params.mass ** 0.5) / params.friction_coefficient
        
        # Energy consumption (higher mass, friction, damping increases consumption)
        energy = (params.mass * 10 + 
                  params.friction_coefficient * 5 + 
                  params.damping_linear * 3 + 
                  params.damping_angular * 3)
        
        # Stability measure (damping affects stability)
        stability = 1.0 / (params.damping_linear * params.damping_angular * 0.1 + 0.2)
        
        # Accuracy measure (how well the robot follows the commanded trajectory)
        accuracy = 1.0 / (abs(params.friction_coefficient - 0.7) * 2 + 0.5)
        
        return {
            'time_to_target': time_to_target,
            'energy_consumption': energy,
            'stability': stability,
            'accuracy': accuracy
        }

    def cost_function(self, param_values: List[float], target_behavior: Dict[str, float] = None) -> float:
        """
        Cost function to minimize during optimization.
        """
        # Create PhysicsParameters object from the list of values
        params = PhysicsParameters(
            robot_name=self.robot_name,
            mass=param_values[0],
            friction_coefficient=param_values[1],
            damping_linear=param_values[2],
            damping_angular=param_values[3],
            gear_ratio=param_values[4],
            max_torque=param_values[5],
            wheel_radius=param_values[6],
            wheel_base=param_values[7],
            tread_width=param_values[8]
        )
        
        # Simulate behavior with these parameters
        simulated_behavior = self.simulate_robot_behavior(params)
        
        # Define default target behavior if not provided
        if target_behavior is None:
            target_behavior = {
                'time_to_target': 5.0,
                'energy_consumption': 50.0,
                'stability': 0.8,
                'accuracy': 0.9
            }
        
        # Calculate cost as sum of squared differences from target behavior
        cost = 0.0
        for key in target_behavior:
            if key in simulated_behavior:
                # Weight different metrics appropriately
                if key == 'time_to_target':
                    weight = 1.0
                elif key == 'energy_consumption':
                    weight = 0.01
                else:
                    weight = 1.0
                
                diff = simulated_behavior[key] - target_behavior[key]
                cost += weight * (diff ** 2)
        
        # Add regularization to prevent extreme parameter values
        # Mass should be reasonable (not too light or too heavy)
        if params.mass < 0.5 or params.mass > 30.0:
            cost += 100.0
            
        # Friction should be reasonable
        if params.friction_coefficient < 0.01 or params.friction_coefficient > 5.0:
            cost += 100.0
        
        # Store history for analysis
        history_entry = {
            'params': param_values,
            'behavior': simulated_behavior,
            'cost': cost
        }
        self.optimization_history.append(history_entry)
        
        return cost

    def optimize_parameters(self, target_behavior: Dict[str, float] = None, 
                           method: str = 'L-BFGS-B', max_iter: int = 100) -> Tuple[PhysicsParameters, float]:
        """
        Optimize physics parameters to achieve target behavior.
        """
        print("Starting physics parameter optimization...")
        
        # Initial parameter values
        initial_values = [
            self.initial_params.mass,
            self.initial_params.friction_coefficient,
            self.initial_params.damping_linear,
            self.initial_params.damping_angular,
            self.initial_params.gear_ratio,
            self.initial_params.max_torque,
            self.initial_params.wheel_radius,
            self.initial_params.wheel_base,
            self.initial_params.tread_width
        ]
        
        # Optimize
        result = minimize(
            fun=lambda x: self.cost_function(x, target_behavior),
            x0=initial_values,
            method=method,
            bounds=self.bounds,
            options={'maxiter': max_iter}
        )
        
        if result.success:
            optimized_params = PhysicsParameters(
                robot_name=self.robot_name,
                mass=result.x[0],
                friction_coefficient=result.x[1],
                damping_linear=result.x[2],
                damping_angular=result.x[3],
                gear_ratio=result.x[4],
                max_torque=result.x[5],
                wheel_radius=result.x[6],
                wheel_base=result.x[7],
                tread_width=result.x[8]
            )
            
            print(f"Optimization successful! Final cost: {result.fun:.4f}")
            print(f"Optimized parameters: {result.x}")
            
            return optimized_params, result.fun
        else:
            print(f"Optimization failed: {result.message}")
            return self.initial_params, float('inf')

    def plot_optimization_history(self):
        """Plot the optimization history to visualize convergence."""
        if not self.optimization_history:
            print("No optimization history to plot")
            return
        
        costs = [entry['cost'] for entry in self.optimization_history]
        
        plt.figure(figsize=(10, 6))
        plt.plot(costs)
        plt.title('Optimization History: Cost vs Iteration')
        plt.xlabel('Iteration')
        plt.ylabel('Cost')
        plt.grid(True)
        plt.show()

    def save_parameters_to_file(self, params: PhysicsParameters, filename: str):
        """Save optimized parameters to a YAML file."""
        param_dict = {
            'robot_name': params.robot_name,
            'mass': params.mass,
            'friction_coefficient': params.friction_coefficient,
            'damping_linear': params.damping_linear,
            'damping_angular': params.damping_angular,
            'gear_ratio': params.gear_ratio,
            'max_torque': params.max_torque,
            'wheel_radius': params.wheel_radius,
            'wheel_base': params.wheel_base,
            'tread_width': params.tread_width
        }
        
        with open(filename, 'w') as f:
            yaml.dump(param_dict, f, default_flow_style=False)
        
        print(f"Parameters saved to {filename}")

    def load_parameters_from_file(self, filename: str) -> PhysicsParameters:
        """Load parameters from a YAML file."""
        with open(filename, 'r') as f:
            param_dict = yaml.safe_load(f)
        
        return PhysicsParameters(
            robot_name=param_dict['robot_name'],
            mass=param_dict['mass'],
            friction_coefficient=param_dict['friction_coefficient'],
            damping_linear=param_dict['damping_linear'],
            damping_angular=param_dict['damping_angular'],
            gear_ratio=param_dict['gear_ratio'],
            max_torque=param_dict['max_torque'],
            wheel_radius=param_dict['wheel_radius'],
            wheel_base=param_dict['wheel_base'],
            tread_width=param_dict['tread_width']
        )


def main():
    parser = argparse.ArgumentParser(description='Physics Parameter Tuning for Digital Twin')
    parser.add_argument('--robot-name', type=str, default='turtlebot3',
                        help='Name of the robot model to tune')
    parser.add_argument('--target-behavior', type=str, 
                        help='JSON string of target behavior to achieve')
    parser.add_argument('--output-file', type=str, 
                        help='File to save optimized parameters to')
    parser.add_argument('--max-iterations', type=int, default=100,
                        help='Maximum number of optimization iterations')
    
    args = parser.parse_args()
    
    # Parse target behavior if provided
    target_behavior = None
    if args.target_behavior:
        try:
            target_behavior = json.loads(args.target_behavior)
        except json.JSONDecodeError:
            print(f"Error: Invalid JSON in target behavior: {args.target_behavior}")
            return
    
    # Create optimizer
    optimizer = PhysicsOptimizer(robot_name=args.robot_name)
    
    # Run optimization
    optimized_params, final_cost = optimizer.optimize_parameters(
        target_behavior=target_behavior,
        max_iter=args.max_iterations
    )
    
    # Print results
    print(f"\nOptimization Results:")
    print(f"Robot: {optimized_params.robot_name}")
    print(f"Mass: {optimized_params.mass:.3f} kg")
    print(f"Friction Coefficient: {optimized_params.friction_coefficient:.3f}")
    print(f"Linear Damping: {optimized_params.damping_linear:.3f}")
    print(f"Angular Damping: {optimized_params.damping_angular:.3f}")
    print(f"Gear Ratio: {optimized_params.gear_ratio:.3f}")
    print(f"Max Torque: {optimized_params.max_torque:.3f} Nm")
    print(f"Wheel Radius: {optimized_params.wheel_radius:.3f} m")
    print(f"Wheel Base: {optimized_params.wheel_base:.3f} m")
    print(f"Tread Width: {optimized_params.tread_width:.3f} m")
    print(f"Final Cost: {final_cost:.4f}")
    
    # Save parameters if output file specified
    if args.output_file:
        optimizer.save_parameters_to_file(optimized_params, args.output_file)
    
    # Plot optimization history
    optimizer.plot_optimization_history()


if __name__ == '__main__':
    main()