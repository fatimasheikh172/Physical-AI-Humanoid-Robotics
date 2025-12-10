# Week 3: Simulation Fidelity

## Overview

Simulation fidelity refers to how accurately your digital twin replicates the behavior of a real-world system. In this section, you'll learn how to measure and improve the accuracy of your simulation, ensuring that the digital twin provides meaningful insights for real-world applications.

## Learning Objectives

By the end of this section, you will be able to:

- Analyze the fidelity of your simulation compared to expected real-world behavior
- Implement parameter tuning techniques to improve simulation accuracy
- Apply domain randomization to reduce the sim-to-real gap
- Validate that your perception algorithms work effectively with simulated data

## Understanding Simulation Fidelity

### Definition and Importance

Simulation fidelity measures how closely a simulation matches the real-world system it represents. High fidelity is crucial for:

- **Training**: Ensuring that skills learned in simulation transfer to real robots
- **Testing**: Validating algorithms before deployment on physical systems
- **Design**: Making informed decisions about robot design and capabilities

### Types of Fidelity

1. **Visual Fidelity**: How closely the visual representation matches reality
2. **Physical Fidelity**: How accurately physics are simulated
3. **Sensor Fidelity**: How closely sensor data matches real sensors
4. **Behavioral Fidelity**: How closely robot behaviors match real systems

## Measuring Simulation Fidelity

### Quantitative Metrics

#### Physical Accuracy Metrics
- **Position Error**: Difference between simulated and expected positions
- **Velocity Error**: Difference between simulated and expected velocities
- **Timing Accuracy**: How precisely simulation timing matches real time
- **Force/Interaction Accuracy**: How well collisions and interactions behave

#### Sensor Data Quality
- **Range Accuracy**: How closely measured distances match ground truth
- **Noise Characteristics**: Whether sensor noise models match real sensors
- **Update Rates**: Whether sensor data is published at correct frequencies
- **Field of View**: Whether sensor coverage matches specifications

### Comparison Tools

Create a script to compare simulated vs expected behavior:

```python
#!/usr/bin/env python3
"""
Simulation Fidelity Analysis Tool
Compares simulated behavior with expected real-world behavior
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import math
import yaml
import argparse
import os


class FidelityAnalyzer:
    def __init__(self, config_file=None):
        self.config = self.load_config(config_file) if config_file else self.get_default_config()
        self.results = {}
    
    def get_default_config(self):
        """Default configuration for fidelity analysis"""
        return {
            'position_error_threshold': 0.05,  # meters
            'orientation_error_threshold': 0.1,  # radians
            'velocity_error_threshold': 0.1,  # m/s
            'sensor_range_accuracy': 0.02,  # meters
            'timing_tolerance': 0.01,  # seconds
        }
    
    def load_config(self, config_file):
        """Load configuration from YAML file"""
        with open(config_file, 'r') as f:
            return yaml.safe_load(f)
    
    def analyze_position_fidelity(self, sim_positions, expected_positions):
        """Analyze position accuracy between simulation and expected values"""
        if len(sim_positions) != len(expected_positions):
            raise ValueError("Position arrays must have same length")
        
        errors = []
        for sim_pos, exp_pos in zip(sim_positions, expected_positions):
            error = np.linalg.norm(np.array(sim_pos) - np.array(exp_pos))
            errors.append(error)
        
        mean_error = np.mean(errors)
        max_error = np.max(errors)
        std_error = np.std(errors)
        
        self.results['position_fidelity'] = {
            'mean_error': mean_error,
            'max_error': max_error,
            'std_error': std_error,
            'threshold': self.config['position_error_threshold'],
            'pass': mean_error <= self.config['position_error_threshold']
        }
        
        return self.results['position_fidelity']
    
    def analyze_orientation_fidelity(self, sim_orientations, expected_orientations):
        """Analyze orientation accuracy using quaternions"""
        if len(sim_orientations) != len(expected_orientations):
            raise ValueError("Orientation arrays must have same length")
        
        errors = []
        for sim_quat, exp_quat in zip(sim_orientations, expected_orientations):
            # Convert to rotations and find difference
            sim_rot = R.from_quat(sim_quat)
            exp_rot = R.from_quat(exp_quat)
            
            # Find relative rotation
            rel_rot = exp_rot.inv() * sim_rot
            angle_error = rel_rot.magnitude()  # Radians
            
            errors.append(angle_error)
        
        mean_error = np.mean(errors)
        max_error = np.max(errors)
        std_error = np.std(errors)
        
        self.results['orientation_fidelity'] = {
            'mean_error': mean_error,
            'max_error': max_error,
            'std_error': std_error,
            'threshold': self.config['orientation_error_threshold'],
            'pass': mean_error <= self.config['orientation_error_threshold']
        }
        
        return self.results['orientation_fidelity']
    
    def analyze_sensor_fidelity(self, sim_sensor_data, expected_sensor_data):
        """Analyze sensor data quality"""
        # Example for range sensor (LiDAR, sonar, etc.)
        range_errors = []
        
        for sim_range, exp_range in zip(sim_sensor_data, expected_sensor_data):
            if exp_range > 0:  # Valid expected range
                error = abs(sim_range - exp_range)
                range_errors.append(error)
        
        mean_error = np.mean(range_errors)
        max_error = np.max(range_errors)
        std_error = np.std(range_errors)
        
        self.results['sensor_fidelity'] = {
            'mean_error': mean_error,
            'max_error': max_error,
            'std_error': std_error,
            'threshold': self.config['sensor_range_accuracy'],
            'pass': mean_error <= self.config['sensor_range_accuracy']
        }
        
        return self.results['sensor_fidelity']
    
    def generate_report(self):
        """Generate a comprehensive fidelity report"""
        report_lines = [
            "SIMULATION FIDELITY REPORT",
            "=" * 50,
            ""
        ]
        
        # Overall summary
        total_tests = len(self.results)
        passed_tests = sum(1 for result in self.results.values() if result.get('pass', False))
        
        report_lines.extend([
            f"Overall Results: {passed_tests}/{total_tests} tests passed",
            f"Success Rate: {passed_tests/total_tests*100:.1f}%" if total_tests > 0 else "Success Rate: 0%",
            ""
        ])
        
        # Detailed results for each test
        for test_name, result in self.results.items():
            status = "PASS" if result.get('pass', False) else "FAIL"
            report_lines.extend([
                f"Test: {test_name.replace('_', ' ').title()}",
                f"  Status: {status}",
                f"  Mean Error: {result.get('mean_error', 0):.4f}",
                f"  Max Error: {result.get('max_error', 0):.4f}",
                f"  Std Dev: {result.get('std_error', 0):.4f}",
                f"  Threshold: {result.get('threshold', 0):.4f}",
                ""
            ])
        
        # Recommendations
        report_lines.extend([
            "RECOMMENDATIONS:",
            ""
        ])
        
        for test_name, result in self.results.items():
            if not result.get('pass', False):
                test_title = test_name.replace('_', ' ').title()
                report_lines.append(f"- {test_title} failed - consider adjusting simulation parameters")
        
        if passed_tests == total_tests:
            report_lines.append("- All tests passed! Simulation fidelity is good.")
        
        return "\n".join(report_lines)


def main():
    parser = argparse.ArgumentParser(description='Analyze Simulation Fidelity')
    parser.add_argument('--config', type=str, help='Configuration file for fidelity thresholds')
    parser.add_argument('--output', type=str, default='fidelity_report.txt', 
                        help='Output file for the report')
    
    args = parser.parse_args()
    
    # Create analyzer
    analyzer = FidelityAnalyzer(args.config)
    
    # Example: Generate some test data
    num_samples = 100
    time_steps = np.linspace(0, 10, num_samples)
    
    # Simulated positions (with some error relative to expected)
    expected_positions = [(t, 0.1*t**2, 0) for t in time_steps]  # Expected: accelerating in X
    simulated_positions = [
        (pos[0] + np.random.normal(0, 0.02),  # Add small position error
         pos[1] + np.random.normal(0, 0.02),
         pos[2])
        for pos in expected_positions
    ]
    
    # Run position fidelity analysis
    pos_result = analyzer.analyze_position_fidelity(simulated_positions, expected_positions)
    print(f"Position fidelity test: {'PASS' if pos_result['pass'] else 'FAIL'}")
    
    # Example orientations (quaternions)
    expected_orientations = [[0, 0, 0, 1] for _ in range(num_samples)]  # No rotation
    simulated_orientations = [
        [np.random.normal(0, 0.001),  # Add small orientation error
         np.random.normal(0, 0.001),
         np.random.normal(0, 0.001),
         1] 
        for _ in range(num_samples)
    ]
    
    # Run orientation fidelity analysis
    orient_result = analyzer.analyze_orientation_fidelity(simulated_orientations, expected_orientations)
    print(f"Orientation fidelity test: {'PASS' if orient_result['pass'] else 'FAIL'}")
    
    # Example sensor data
    expected_ranges = [5.0 + 0.1*np.sin(t) for t in time_steps]  # Expected ranges
    simulated_ranges = [r + np.random.normal(0, 0.01) for r in expected_ranges]  # With noise
    
    # Run sensor fidelity analysis
    sensor_result = analyzer.analyze_sensor_fidelity(simulated_ranges, expected_ranges)
    print(f"Sensor fidelity test: {'PASS' if sensor_result['pass'] else 'FAIL'}")
    
    # Generate and save report
    report = analyzer.generate_report()
    with open(args.output, 'w') as f:
        f.write(report)
    
    print(f"Fidelity report saved to {args.output}")
    print("\n" + report)


if __name__ == '__main__':
    main()
```

### Visualization Tools

Create a visualization script to help analyze fidelity:

```python
#!/usr/bin/env python3
"""
Fidelity Visualization Tool
Creates plots to visualize the differences between simulated and expected behavior
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def plot_position_comparison(sim_positions, expected_positions, title="Position Comparison"):
    """Plot simulated vs expected positions"""
    fig = plt.figure(figsize=(12, 8))
    
    # 3D plot
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax1.plot3D(*zip(*expected_positions), label='Expected', alpha=0.7)
    ax1.plot3D(*zip(*sim_positions), label='Simulated', alpha=0.7)
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title('3D Position Comparison')
    ax1.legend()
    
    # X component over time
    ax2 = fig.add_subplot(2, 2, 2)
    time = range(len(sim_positions))
    ax2.plot(time, [p[0] for p in expected_positions], label='Expected X', alpha=0.7)
    ax2.plot(time, [p[0] for p in sim_positions], label='Simulated X', alpha=0.7)
    ax2.set_xlabel('Time Step')
    ax2.set_ylabel('X Position')
    ax2.set_title('X Position Over Time')
    ax2.legend()
    
    # Y component over time
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(time, [p[1] for p in expected_positions], label='Expected Y', alpha=0.7)
    ax3.plot(time, [p[1] for p in sim_positions], label='Simulated Y', alpha=0.7)
    ax3.set_xlabel('Time Step')
    ax3.set_ylabel('Y Position')
    ax3.set_title('Y Position Over Time')
    ax3.legend()
    
    # Error magnitude over time
    ax4 = fig.add_subplot(2, 2, 4)
    errors = [np.linalg.norm(np.array(sp) - np.array(ep)) 
              for sp, ep in zip(sim_positions, expected_positions)]
    ax4.plot(time, errors)
    ax4.set_xlabel('Time Step')
    ax4.set_ylabel('Position Error (m)')
    ax4.set_title('Position Error Over Time')
    ax4.grid(True)
    
    plt.suptitle(title)
    plt.tight_layout()
    plt.show()


def plot_sensor_comparison(sim_sensor_data, expected_sensor_data, title="Sensor Comparison"):
    """Plot simulated vs expected sensor data"""
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))
    
    time = range(len(sim_sensor_data))
    
    # Raw data comparison
    ax1.plot(time, expected_sensor_data, label='Expected', alpha=0.7)
    ax1.plot(time, sim_sensor_data, label='Simulated', alpha=0.7)
    ax1.set_ylabel('Sensor Value')
    ax1.set_title(f'{title} - Raw Data')
    ax1.legend()
    ax1.grid(True)
    
    # Error over time
    errors = [abs(s - e) for s, e in zip(sim_sensor_data, expected_sensor_data)]
    ax2.plot(time, errors)
    ax2.set_ylabel('Absolute Error')
    ax2.set_title('Absolute Error Over Time')
    ax2.grid(True)
    
    # Error histogram
    ax3.hist(errors, bins=30, edgecolor='black')
    ax3.set_xlabel('Error Magnitude')
    ax3.set_ylabel('Frequency')
    ax3.set_title('Error Distribution')
    ax3.grid(True)
    
    plt.suptitle(title)
    plt.tight_layout()
    plt.show()


def plot_orientation_comparison(sim_orientations, expected_orientations, title="Orientation Comparison"):
    """Plot simulated vs expected orientations (as roll, pitch, yaw)"""
    from scipy.spatial.transform import Rotation as R
    
    # Convert quaternions to Euler angles
    sim_rotations = R.from_quat(sim_orientations)
    exp_rotations = R.from_quat(expected_orientations)
    
    sim_euler = sim_rotations.as_euler('xyz')
    exp_euler = exp_rotations.as_euler('xyz')
    
    fig, axes = plt.subplots(3, 1, figsize=(12, 8))
    
    time = range(len(sim_euler))
    
    # Roll comparison
    axes[0].plot(time, exp_euler[:, 0], label='Expected Roll', alpha=0.7)
    axes[0].plot(time, sim_euler[:, 0], label='Simulated Roll', alpha=0.7)
    axes[0].set_ylabel('Roll (rad)')
    axes[0].legend()
    axes[0].grid(True)
    
    # Pitch comparison
    axes[1].plot(time, exp_euler[:, 1], label='Expected Pitch', alpha=0.7)
    axes[1].plot(time, sim_euler[:, 1], label='Simulated Pitch', alpha=0.7)
    axes[1].set_ylabel('Pitch (rad)')
    axes[1].legend()
    axes[1].grid(True)
    
    # Yaw comparison
    axes[2].plot(time, exp_euler[:, 2], label='Expected Yaw', alpha=0.7)
    axes[2].plot(time, sim_euler[:, 2], label='Simulated Yaw', alpha=0.7)
    axes[2].set_ylabel('Yaw (rad)')
    axes[2].set_xlabel('Time Step')
    axes[2].legend()
    axes[2].grid(True)
    
    plt.suptitle(title)
    plt.tight_layout()
    plt.show()
```

## Parameter Tuning for Fidelity Improvement

### Physics Parameter Optimization

Create a parameter tuning script to optimize physics parameters for better fidelity:

```python
#!/usr/bin/env python3
"""
Physics Parameter Tuning for Simulation Fidelity
Uses optimization algorithms to find physics parameters that best match real-world behavior
"""

import numpy as np
from scipy.optimize import minimize
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


class PhysicsTuner:
    def __init__(self, real_data, target_function=None):
        """
        Initialize the physics tuner
        
        Parameters:
        - real_data: Real-world data to match against
        - target_function: Function that runs the simulation and returns trajectory
        """
        self.real_data = real_data
        self.target_function = target_function or self.default_simulation
        
    def default_simulation(self, time_points, params):
        """
        Default simulation function modeling a simple system
        
        Parameters:
        - time_points: Array of time points to evaluate
        - params: [mass, friction_coeff, damping_coeff]
        """
        mass, friction_coeff, damping_coeff = params
        
        # Simple physics model: ball with friction and damping
        # For this example, we'll model 1D motion with friction
        def dynamics(t, y):
            position, velocity = y
            # F = ma = -friction*sign(v) - damping*v
            acceleration = -(friction_coeff * 9.8 * np.sign(velocity) + damping_coeff * velocity) / mass
            return [velocity, acceleration]
        
        # Initial conditions
        initial_conditions = [0, 5.0]  # Start at 0 with velocity 5
        
        # Solve ODE
        sol = solve_ivp(
            dynamics, 
            [time_points[0], time_points[-1]], 
            initial_conditions,
            t_eval=time_points,
            method='RK45'
        )
        
        return sol.y[0]  # Return position over time

    def cost_function(self, params):
        """
        Cost function to minimize - difference between simulation and real data
        """
        # Ensure parameters are physically reasonable
        if params[0] <= 0 or params[1] < 0 or params[2] < 0:  # mass, friction, damping must be positive
            return 1e6  # Large penalty for invalid parameters
        
        # Run simulation
        sim_trajectory = self.target_function(self.real_data['time'], params)
        
        # Handle case where simulation fails
        if len(sim_trajectory) != len(self.real_data['values']):
            return 1e6
        
        # Calculate cost (mean squared error)
        squared_diff = (sim_trajectory - self.real_data['values']) ** 2
        cost = np.mean(squared_diff)
        
        return cost
    
    def tune_parameters(self, initial_guess, bounds):
        """
        Tune physics parameters to match real data
        
        Parameters:
        - initial_guess: Initial parameter values [mass, friction_coeff, damping_coeff]
        - bounds: List of (min, max) tuples for each parameter
        """
        print("Starting parameter tuning...")
        print(f"Initial guess: {initial_guess}")
        print(f"Bounds: {bounds}")
        
        result = minimize(
            self.cost_function,
            initial_guess,
            method='L-BFGS-B',
            bounds=bounds
        )
        
        if result.success:
            print(f"Optimization successful! Final cost: {result.fun:.6f}")
            print(f"Optimized parameters: {result.x}")
            return result.x
        else:
            print(f"Optimization failed: {result.message}")
            return initial_guess
    
    def validate_tuning(self, params):
        """
        Validate the tuned parameters against the real data
        """
        sim_trajectory = self.target_function(self.real_data['time'], params)
        
        # Calculate validation metrics
        mae = np.mean(np.abs(sim_trajectory - self.real_data['values']))
        rmse = np.sqrt(np.mean((sim_trajectory - self.real_data['values']) ** 2))
        max_error = np.max(np.abs(sim_trajectory - self.real_data['values']))
        
        print(f"\nValidation Results:")
        print(f"Mean Absolute Error: {mae:.4f}")
        print(f"RMSE: {rmse:.4f}")
        print(f"Max Error: {max_error:.4f}")
        
        # Plot comparison
        plt.figure(figsize=(10, 6))
        plt.plot(self.real_data['time'], self.real_data['values'], label='Real Data', linewidth=2)
        plt.plot(self.real_data['time'], sim_trajectory, label='Tuned Simulation', linestyle='--', linewidth=2)
        plt.xlabel('Time')
        plt.ylabel('Value')
        plt.title('Real Data vs Tuned Simulation')
        plt.legend()
        plt.grid(True)
        plt.show()
        
        return {
            'mae': mae,
            'rmse': rmse,
            'max_error': max_error,
            'sim_trajectory': sim_trajectory
        }


def main():
    # Create example real data (e.g., from physical experiment)
    time_points = np.linspace(0, 5, 50)
    # Simulate some real-world data with noise
    real_values = 5 * np.exp(-0.5 * time_points) * np.cos(2 * time_points) + np.random.normal(0, 0.02, len(time_points))
    
    real_data = {
        'time': time_points,
        'values': real_values
    }
    
    # Define initial guess and bounds for parameters
    initial_guess = [1.0, 0.1, 0.05]  # [mass, friction_coeff, damping_coeff]
    bounds = [
        (0.1, 10.0),    # mass bounds
        (0.01, 1.0),   # friction coefficient bounds  
        (0.01, 1.0)    # damping coefficient bounds
    ]
    
    # Create tuner and run optimization
    tuner = PhysicsTuner(real_data)
    optimized_params = tuner.tune_parameters(initial_guess, bounds)
    
    # Validate results
    validation_results = tuner.validate_tuning(optimized_params)
    
    print(f"\nOptimized Parameters: {optimized_params}")


if __name__ == '__main__':
    main()
```

## Domain Randomization

### Reducing Sim-to-Real Gap

Domain randomization is a technique to reduce the sim-to-real gap by training or testing with randomized simulation parameters:

```python
#!/usr/bin/env python3
"""
Domain Randomization for Sim-to-Real Gap Reduction
"""

import numpy as np
import random
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
import copy


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
    
    def randomize_lighting(self) -> Dict[str, float]:
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
    
    def randomize_visual_textures(self) -> Dict[str, float]:
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
    
    def randomize_colors(self) -> Dict[str, List[float]]:
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
        randomized_params = copy.deepcopy(base_params)
        
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
    
    def randomize_sensor_properties(self, base_sensor_params: Dict[str, float]) -> Dict[str, float]:
        """Randomize sensor properties to simulate real sensor variations"""
        randomized_params = copy.deepcopy(base_sensor_params)
        
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
    
    def randomize_environment(self) -> Dict[str, float]:
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
    
    def apply_randomization(self) -> Dict[str, any]:
        """Apply all randomization techniques to the simulation"""
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
            'mass': 1.0,
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
        
        self.randomization_step += 1
        
        return all_changes


def main():
    print("Domain Randomization Example")
    print("-" * 30)
    
    # Create randomizer
    randomizer = DomainRandomizer()
    
    # Apply randomization
    changes = randomizer.apply_randomization()
    
    # Print some key changes
    print(f"Physics - Friction: {changes['physics']['friction']:.3f}, "
          f"Mass: {changes['physics']['mass']:.3f} kg")
    print(f"Sensor - Noise: {changes['sensors']['noise_level']:.4f}, "
          f"Bias: {changes['sensors']['bias']:.4f}")
    print(f"Environment - Gravity: {changes['environment']['gravity']:.3f} m/s²")
    
    # Multiple randomization steps for training
    print("\nNext 5 randomization steps for training...")
    for i in range(5):
        changes = randomizer.apply_randomization()
        print(f"Step {i+2}: Mass={changes['physics']['mass']:.2f}, "
              f"Friction={changes['physics']['friction']:.2f}, "
              f"Gravity={changes['environment']['gravity']:.2f}")


if __name__ == '__main__':
    main()
```

## Perception Validation

### Validating Perception with Simulated Data

Create a validation script for perception algorithms:

```python
#!/usr/bin/env python3
"""
Perception Validation with Simulated Data
Tests perception algorithms using synthetic but realistic sensor data
"""

import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist


class PerceptionValidator:
    def __init__(self):
        self.ground_truth = {}
        self.results = {}
    
    def generate_synthetic_lidar_data(self, num_points=360, max_range=10.0, obstacles=None):
        """Generate synthetic LiDAR data with ground truth"""
        angles = np.linspace(0, 2*np.pi, num_points, endpoint=False)
        ranges = np.full(num_points, max_range)  # Start with max range (no obstacles)
        
        # Add obstacles
        if obstacles is None:
            obstacles = [
                {'pos': (3.0, 0.0), 'size': 0.5},  # Circle at (3,0) with radius 0.5
                {'pos': (5.0, 2.0), 'size': 0.8},
            ]
        
        for i, angle in enumerate(angles):
            ray_x, ray_y = np.cos(angle), np.sin(angle)
            
            for obs in obstacles:
                obs_x, obs_y = obs['pos']
                obs_radius = obs['size']
                
                # Find distance from ray to obstacle
                # For a ray starting at origin, direction (ray_x, ray_y), 
                # distance to circle at (obs_x, obs_y) with radius obs_radius
                a = np.dot([ray_x, ray_y], [ray_x, ray_y])  # Should be 1
                b = 2 * np.dot([ray_x, ray_y], [0, 0] - [obs_x, obs_y])
                c = np.dot([0, 0] - [obs_x, obs_y], [0, 0] - [obs_x, obs_y]) - obs_radius**2
                
                discriminant = b**2 - 4*a*c
                if discriminant >= 0:
                    t = (-b - np.sqrt(discriminant)) / (2*a)  # Nearest intersection
                    if t > 0 and t < ranges[i]:  # Ray goes outward and closer than current range
                        ranges[i] = t
        
        # Add realistic noise
        noise_std = 0.02  # meters
        noisy_ranges = ranges + np.random.normal(0, noise_std, ranges.shape)
        
        # Clip to valid range
        noisy_ranges = np.clip(noisy_ranges, 0.1, max_range)
        
        self.ground_truth['lidar'] = {
            'angles': angles,
            'ranges': ranges,  # Noise-free for ground truth
            'obstacles': obstacles
        }
        
        return angles, noisy_ranges
    
    def detect_obstacles_lidar(self, angles, ranges, threshold=2.0):
        """Simple obstacle detection algorithm"""
        # Find ranges less than threshold
        obstacle_indices = np.where(ranges < threshold)[0]
        
        # Convert to Cartesian coordinates
        obstacle_points = []
        for idx in obstacle_indices:
            angle = angles[idx]
            dist = ranges[idx]
            x = dist * np.cos(angle)
            y = dist * np.sin(angle)
            obstacle_points.append((x, y))
        
        return np.array(obstacle_points) if len(obstacle_points) > 0 else np.array([]).reshape(0, 2)
    
    def validate_lidar_perception(self, min_recall=0.8, min_precision=0.7):
        """Validate LiDAR perception against ground truth"""
        # Generate synthetic data
        angles, ranges = self.generate_synthetic_lidar_data()
        
        # Run perception algorithm
        detected_obstacles = self.detect_obstacles_lidar(angles, ranges)
        
        # Get ground truth obstacles
        gt_obstacles = []
        for obs in self.ground_truth['lidar']['obstacles']:
            gt_obstacles.append(obs['pos'])
        gt_obstacles = np.array(gt_obstacles)
        
        if len(gt_obstacles) == 0 or len(detected_obstacles) == 0:
            recall = 0.0 if len(gt_obstacles) > 0 else 1.0
            precision = 0.0 if len(detected_obstacles) > 0 else 1.0
        else:
            # Calculate distances between detected and ground truth
            distances = cdist(detected_obstacles, gt_obstacles)
            
            # For each ground truth, find closest detection
            gt_matches = np.min(distances, axis=0) < 0.5  # Match if within 0.5m
            recall = np.sum(gt_matches) / len(gt_obstacles)
            
            # For each detection, find closest ground truth
            det_matches = np.min(distances, axis=1) < 0.5  # Match if within 0.5m
            precision = np.sum(det_matches) / len(detected_obstacles)
        
        f1_score = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0.0
        
        self.results['lidar'] = {
            'recall': recall,
            'precision': precision,
            'f1_score': f1_score,
            'threshold_recall': recall >= min_recall,
            'threshold_precision': precision >= min_precision,
            'passed': recall >= min_recall and precision >= min_precision
        }
        
        return self.results['lidar']
    
    def generate_synthetic_camera_data(self, width=640, height=480, fov=90):
        """Generate synthetic camera data with ground truth"""
        # Create a simple scene with known objects
        img = np.ones((height, width, 3), dtype=np.uint8) * 240  # Light gray background
        
        # Add some objects with known positions
        objects = [
            {'type': 'circle', 'pos': (150, 200), 'radius': 25, 'color': (255, 0, 0)},      # Red circle
            {'type': 'rectangle', 'pos': (400, 300), 'size': (40, 30), 'color': (0, 255, 0)},  # Green rectangle
            {'type': 'triangle', 'pos': (300, 150), 'size': 30, 'color': (0, 0, 255)},         # Blue triangle
        ]
        
        for obj in objects:
            if obj['type'] == 'circle':
                cv2.circle(img, obj['pos'], obj['radius'], obj['color'], -1)
            elif obj['type'] == 'rectangle':
                pt1 = (obj['pos'][0] - obj['size'][0]//2, obj['pos'][1] - obj['size'][1]//2)
                pt2 = (obj['pos'][0] + obj['size'][0]//2, obj['pos'][1] + obj['size'][1]//2)
                cv2.rectangle(img, pt1, pt2, obj['color'], -1)
            elif obj['type'] == 'triangle':
                # Create triangle points
                center = obj['pos']
                size = obj['size']
                
                pts = np.array([
                    [center[0], center[1] - size],
                    [center[0] - size, center[1] + size],
                    [center[0] + size, center[1] + size]
                ], np.int32)
                
                cv2.fillPoly(img, [pts], obj['color'])
        
        # Add some noise
        noise = np.random.normal(0, 10, img.shape).astype(np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        
        self.ground_truth['camera'] = {
            'image': img,
            'objects': objects,
            'width': width,
            'height': height
        }
        
        return img
    
    def detect_objects_camera(self, img):
        """Simple object detection algorithm"""
        # Convert to HSV for color-based detection
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        detected_objects = []
        
        # Define color ranges
        color_ranges = {
            'red': (np.array([0, 50, 50]), np.array([10, 255, 255])),
            'red2': (np.array([170, 50, 50]), np.array([180, 255, 255])),  # Red wraps around in HSV
            'green': (np.array([40, 50, 50]), np.array([80, 255, 255])),
            'blue': (np.array([100, 50, 50]), np.array([130, 255, 255]))
        }
        
        for color_name, (lower, upper) in color_ranges.items():
            if color_name == 'red2':  # Combine the two red ranges
                mask1 = cv2.inRange(hsv, color_ranges['red'][0], color_ranges['red'][1])
                mask2 = cv2.inRange(hsv, lower, upper)
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(hsv, lower, upper)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 100:  # Filter out small areas
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)
                    detected_objects.append({
                        'color': color_name,
                        'bbox': (x, y, w, h),
                        'center': (x + w//2, y + h//2),
                        'area': area
                    })
        
        return detected_objects
    
    def validate_camera_perception(self, min_iou_threshold=0.5):
        """Validate camera perception against ground truth"""
        # Generate synthetic data
        img = self.generate_synthetic_camera_data()
        
        # Run perception algorithm
        detected_objects = self.detect_objects_camera(img)
        
        # For camera validation, we'll check if we detect the right number of objects
        # and they're in approximately the right positions
        
        expected_positions = [obj['pos'] for obj in self.ground_truth['camera']['objects']]
        detected_positions = [obj['center'] for obj in detected_objects]
        
        if len(expected_positions) == 0:
            accuracy = 1.0 if len(detected_positions) == 0 else 0.0
        elif len(detected_positions) == 0:
            accuracy = 0.0
        else:
            # Calculate distances between expected and detected positions
            distances = cdist([expected_positions[0]], [detected_positions[0]])  # Simplified for demo
            # A more sophisticated validation would match detections to ground truth objects
            accuracy = len(detected_objects) / len(expected_positions)
        
        self.results['camera'] = {
            'accuracy': accuracy,
            'detected_objects': len(detected_objects),
            'expected_objects': len(expected_positions),
            'passed': accuracy >= min_iou_threshold
        }
        
        return self.results['camera']
    
    def run_complete_validation(self):
        """Run validation for all perception systems"""
        lidar_result = self.validate_lidar_perception()
        camera_result = self.validate_camera_perception()
        
        overall_pass = lidar_result['passed'] and camera_result['passed']
        
        print("PERCEPTION VALIDATION RESULTS")
        print("=" * 40)
        print(f"LIDAR Perception: {'PASS' if lidar_result['passed'] else 'FAIL'}")
        print(f"  Recall: {lidar_result['recall']:.3f}")
        print(f"  Precision: {lidar_result['precision']:.3f}")
        print(f"  F1 Score: {lidar_result['f1_score']:.3f}")
        
        print(f"\nCamera Perception: {'PASS' if camera_result['passed'] else 'FAIL'}")
        print(f"  Accuracy: {camera_result['accuracy']:.3f}")
        print(f"  Detected: {camera_result['detected_objects']}, Expected: {camera_result['expected_objects']}")
        
        print(f"\nOverall Validation: {'PASS' if overall_pass else 'FAIL'}")
        
        return overall_pass


def main():
    validator = PerceptionValidator()
    validator.run_complete_validation()


if __name__ == '__main__':
    main()
```

## Next Steps

After implementing simulation fidelity analysis and validation techniques, your digital twin system will be more reliable and accurate. The key elements you've added include:

1. Methods to measure how closely your simulation matches expected behavior
2. Tools to tune simulation parameters for better accuracy
3. Techniques to reduce the sim-to-real gap using domain randomization
4. Validation of perception algorithms with simulated data

This completes the core content for Week 3 and the digital twin module.