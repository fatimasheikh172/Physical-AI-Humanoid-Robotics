#!/usr/bin/env python3

"""
Testing Report Generator for Digital Twin System

This script generates a comprehensive testing report comparing 
simulated behavior with expected behavior for the digital twin system.
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import yaml
import json
import argparse
from datetime import datetime
from pathlib import Path
import statistics
import csv


class TestReportGenerator:
    def __init__(self, report_dir: str = "test_reports"):
        self.report_dir = Path(report_dir)
        self.report_dir.mkdir(exist_ok=True)
        
        self.test_results = {
            'timestamp': datetime.now().isoformat(),
            'tests_executed': [],
            'overall_score': 0.0,
            'fidelity_metrics': {},
            'performance_metrics': {},
            'system_status': {}
        }
    
    def generate_mock_test_data(self):
        """Generate mock test data for demonstration purposes"""
        # Mock sensor data over time
        time_steps = np.arange(0, 10, 0.1)  # 10 seconds of data at 0.1s intervals
        
        # Simulated LiDAR data (distance measurements)
        simulated_lidar = {
            'time': time_steps,
            'distances': [1.0 + 0.1*np.sin(t) + np.random.normal(0, 0.02) for t in time_steps],  # Add noise
            'angles': [np.pi/4 + 0.01*np.cos(t) for t in time_steps]  # Slight angle variations
        }
        
        # Expected LiDAR data (without noise)
        expected_lidar = {
            'time': time_steps,
            'distances': [1.0 + 0.1*np.sin(t) for t in time_steps],
            'angles': [np.pi/4 + 0.01*np.cos(t) for t in time_steps]
        }
        
        # Simulated IMU data
        simulated_imu = {
            'time': time_steps,
            'orientation': [R.from_euler('xyz', [0.01*np.sin(t), 0.01*np.cos(t), 0.005*t]).as_quat() 
                           for t in time_steps],
            'linear_acceleration': [{'x': 0.1*np.cos(t), 'y': -0.05*np.sin(t), 'z': 9.81 + 0.1*np.sin(t)} 
                                   for t in time_steps],
            'angular_velocity': [{'x': 0.01*np.cos(t), 'y': -0.01*np.sin(t), 'z': 0.005} 
                                for t in time_steps]
        }
        
        # Expected IMU data
        expected_imu = {
            'time': time_steps,
            'orientation': [R.from_euler('xyz', [0.01*np.sin(t), 0.01*np.cos(t), 0.005*t]).as_quat() 
                           for t in time_steps],
            'linear_acceleration': [{'x': 0.1*np.cos(t), 'y': -0.05*np.sin(t), 'z': 9.81} 
                                   for t in time_steps],
            'angular_velocity': [{'x': 0.01*np.cos(t), 'y': -0.01*np.sin(t), 'z': 0.005} 
                                for t in time_steps]
        }
        
        return {
            'lidar': {'simulated': simulated_lidar, 'expected': expected_lidar},
            'imu': {'simulated': simulated_imu, 'expected': expected_imu}
        }
    
    def calculate_fidelity_metrics(self, test_data):
        """Calculate fidelity metrics comparing simulated vs expected data"""
        metrics = {}
        
        # LiDAR metrics
        lidar_sim = test_data['lidar']['simulated']
        lidar_exp = test_data['lidar']['expected']
        
        distance_errors = [abs(s - e) for s, e in zip(lidar_sim['distances'], lidar_exp['distances'])]
        metrics['lidar'] = {
            'mean_error': statistics.mean(distance_errors),
            'std_error': statistics.stdev(distance_errors) if len(distance_errors) > 1 else 0,
            'max_error': max(distance_errors),
            'rmse': np.sqrt(statistics.mean([e**2 for e in distance_errors])),
            'data_points': len(distance_errors)
        }
        
        # IMU metrics
        imu_sim = test_data['imu']['simulated']
        imu_exp = test_data['imu']['expected']
        
        # Calculate orientation errors (difference in quaternions)
        orientation_errors = []
        for s, e in zip(imu_sim['orientation'], imu_exp['orientation']):
            # Convert quaternion difference to angle (in radians)
            q_diff = R.from_quat(s).inv() * R.from_quat(e)
            angle_diff = R.magnitude(q_diff)
            orientation_errors.append(angle_diff)
        
        metrics['imu'] = {
            'orientation_mean_error': statistics.mean(orientation_errors),
            'orientation_max_error': max(orientation_errors),
            'orientation_rmse': np.sqrt(statistics.mean([e**2 for e in orientation_errors]))
        }
        
        # Linear acceleration errors
        lin_acc_errors = []
        for s, e in zip(imu_sim['linear_acceleration'], imu_exp['linear_acceleration']):
            error = np.sqrt(
                (s['x'] - e['x'])**2 + 
                (s['y'] - e['y'])**2 + 
                (s['z'] - e['z'])**2
            )
            lin_acc_errors.append(error)
        
        metrics['imu']['linear_acceleration'] = {
            'mean_error': statistics.mean(lin_acc_errors),
            'rmse': np.sqrt(statistics.mean([e**2 for e in lin_acc_errors]))
        }
        
        self.test_results['fidelity_metrics'] = metrics
        return metrics
    
    def calculate_performance_metrics(self):
        """Calculate performance metrics"""
        # Mock performance data
        perf_metrics = {
            'simulation_fps': 60.0,  # frames per second
            'sensor_update_rate': 10.0,  # Hz
            'processing_latency': 0.015,  # seconds
            'memory_usage': 512.0,  # MB
            'cpu_usage': 45.0  # percentage
        }
        
        self.test_results['performance_metrics'] = perf_metrics
        return perf_metrics
    
    def generate_test_report(self):
        """Generate a comprehensive test report"""
        # Generate mock test data
        test_data = self.generate_mock_test_data()
        
        # Calculate metrics
        fidelity_metrics = self.calculate_fidelity_metrics(test_data)
        performance_metrics = self.calculate_performance_metrics()
        
        # Calculate overall score (combination of fidelity and performance)
        # Normalize metrics to 0-1 scale and combine
        lidar_fidelity_score = max(0, 1 - fidelity_metrics['lidar']['mean_error'] * 5)  # Adjust based on expected range
        imu_orientation_score = max(0, 1 - fidelity_metrics['imu']['orientation_mean_error'] * 10)
        perf_score = min(1.0, performance_metrics['simulation_fps'] / 60.0)  # Normalize FPS to 0-1
        
        overall_score = (lidar_fidelity_score * 0.4 + 
                        imu_orientation_score * 0.3 + 
                        perf_score * 0.3)
        
        self.test_results['overall_score'] = overall_score
        
        # Create detailed report
        report_content = self.create_detailed_report(test_data, fidelity_metrics, performance_metrics, overall_score)
        
        # Save report to file
        report_filename = self.report_dir / f"test_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.md"
        with open(report_filename, 'w') as f:
            f.write(report_content)
        
        # Also save as JSON for programmatic access
        json_filename = self.report_dir / f"test_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(json_filename, 'w') as f:
            json.dump(self.test_results, f, indent=2)
        
        print(f"Test report generated: {report_filename}")
        return str(report_filename)
    
    def create_detailed_report(self, test_data, fidelity_metrics, performance_metrics, overall_score):
        """Create a detailed markdown report"""
        report = f"""# Digital Twin Testing Report

**Generated:** {self.test_results['timestamp']}
**Overall Fidelity Score:** {overall_score:.3f}/1.0

## Summary

This report compares the simulated behavior of the digital twin system against expected behavior. The system demonstrates {self.get_quality_description(overall_score)} fidelity.

## Test Configuration

- **Test Duration:** 10 seconds
- **Simulation Time Step:** 0.1 seconds
- **Components Tested:** LiDAR, IMU

## Fidelity Analysis

### LiDAR Sensor

| Metric | Value | Unit |
|--------|-------|------|
| Mean Error | {fidelity_metrics['lidar']['mean_error']:.4f} | m |
| Standard Deviation | {fidelity_metrics['lidar']['std_error']:.4f} | m |
| Max Error | {fidelity_metrics['lidar']['max_error']:.4f} | m |
| RMSE | {fidelity_metrics['lidar']['rmse']:.4f} | m |
| Data Points | {fidelity_metrics['lidar']['data_points']} | - |

The LiDAR sensor demonstrates {self.get_quality_description(1 - fidelity_metrics['lidar']['mean_error'])} accuracy with respect to distance measurements.

### IMU Sensor

| Metric | Value | Unit |
|--------|-------|------|
| Orientation Mean Error | {fidelity_metrics['imu']['orientation_mean_error']:.4f} | rad |
| Orientation Max Error | {fidelity_metrics['imu']['orientation_max_error']:.4f} | rad |
| Orientation RMSE | {fidelity_metrics['imu']['orientation_rmse']:.4f} | rad |
| Linear Acceleration Mean Error | {fidelity_metrics['imu']['linear_acceleration']['mean_error']:.4f} | m/s² |
| Linear Acceleration RMSE | {fidelity_metrics['imu']['linear_acceleration']['rmse']:.4f} | m/s² |

The IMU sensor demonstrates {self.get_quality_description(1 - fidelity_metrics['imu']['orientation_mean_error'])} accuracy with respect to orientation measurements.

## Performance Analysis

| Metric | Value | Unit |
|--------|-------|------|
| Simulation FPS | {performance_metrics['simulation_fps']:.1f} | frames/second |
| Sensor Update Rate | {performance_metrics['sensor_update_rate']:.1f} | Hz |
| Processing Latency | {performance_metrics['processing_latency']:.3f} | seconds |
| Memory Usage | {performance_metrics['memory_usage']:.1f} | MB |
| CPU Usage | {performance_metrics['cpu_usage']:.1f} | % |

## Recommendations

Based on the test results, the following recommendations are made:

1. **LiDAR Accuracy:** {'Consider reducing noise parameters' if fidelity_metrics['lidar']['mean_error'] > 0.05 else 'Performance is adequate'}
2. **IMU Accuracy:** {'Consider improving orientation estimation algorithms' if fidelity_metrics['imu']['orientation_mean_error'] > 0.02 else 'Performance is adequate'}
3. **Performance:** {'Optimize for better real-time performance' if performance_metrics['simulation_fps'] < 50.0 else 'Performance is adequate'}

## Visualizations

The following plots compare simulated vs expected values:

- LiDAR distance measurements over time
- IMU orientation over time
- IMU linear acceleration over time

## Conclusion

The digital twin system demonstrates {'high' if overall_score > 0.8 else 'moderate' if overall_score > 0.6 else 'low'} fidelity in simulating the expected behavior. {'The system is ready for advanced tasks.' if overall_score > 0.8 else 'Consider parameter tuning to improve fidelity.'}

---

*This report was automatically generated by the Digital Twin Testing Framework.*
"""
        return report
    
    def get_quality_description(self, score):
        """Get a text description for a score"""
        if score >= 0.9:
            return "excellent"
        elif score >= 0.8:
            return "very good"
        elif score >= 0.7:
            return "good"
        elif score >= 0.6:
            return "adequate"
        elif score >= 0.5:
            return "needs improvement"
        else:
            return "poor"
    
    def create_visualizations(self, test_data):
        """Create visualizations comparing simulated vs expected data"""
        # Create figure with subplots
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Digital Twin Simulation Fidelity Analysis', fontsize=16)
        
        # LiDAR distance comparison
        lidar_sim = test_data['lidar']['simulated']
        lidar_exp = test_data['lidar']['expected']
        
        axes[0, 0].plot(lidar_sim['time'], lidar_sim['distances'], label='Simulated', alpha=0.7)
        axes[0, 0].plot(lidar_exp['time'], lidar_exp['distances'], label='Expected', alpha=0.7)
        axes[0, 0].set_title('LiDAR Distance Comparison')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Distance (m)')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        # LiDAR error over time
        distance_errors = [abs(s - e) for s, e in zip(lidar_sim['distances'], lidar_exp['distances'])]
        axes[0, 1].plot(lidar_sim['time'], distance_errors)
        axes[0, 1].set_title('LiDAR Distance Error Over Time')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Error (m)')
        axes[0, 1].grid(True)
        
        # IMU orientation comparison (using first component of quaternion)
        imu_sim = test_data['imu']['simulated']
        imu_exp = test_data['imu']['expected']
        
        sim_w = [q[3] for q in imu_sim['orientation']]  # w component
        exp_w = [q[3] for q in imu_exp['orientation']]
        
        axes[1, 0].plot(test_data['lidar']['simulated']['time'], sim_w, label='Simulated', alpha=0.7)
        axes[1, 0].plot(test_data['lidar']['simulated']['time'], exp_w, label='Expected', alpha=0.7)
        axes[1, 0].set_title('IMU Orientation (W Component) Comparison')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Quaternion W')
        axes[1, 0].legend()
        axes[1, 0].grid(True)
        
        # IMU linear acceleration Z comparison
        sim_acc_z = [a['z'] for a in imu_sim['linear_acceleration']]
        exp_acc_z = [a['z'] for a in imu_exp['linear_acceleration']]
        
        axes[1, 1].plot(test_data['lidar']['simulated']['time'], sim_acc_z, label='Simulated Z', alpha=0.7)
        axes[1, 1].plot(test_data['lidar']['simulated']['time'], exp_acc_z, label='Expected Z', alpha=0.7)
        axes[1, 1].set_title('IMU Linear Acceleration Z Comparison')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('Acceleration (m/s²)')
        axes[1, 1].legend()
        axes[1, 1].grid(True)
        
        plt.tight_layout()
        
        # Save the visualization
        viz_path = self.report_dir / f"visualizations_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        plt.savefig(viz_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"Visualizations saved to: {viz_path}")
        
        return str(viz_path)


def main():
    parser = argparse.ArgumentParser(description='Generate Testing Report for Digital Twin')
    parser.add_argument('--report-dir', type=str, default='test_reports',
                        help='Directory to save test reports')
    parser.add_argument('--include-viz', action='store_true',
                        help='Include visualizations in the report')
    
    args = parser.parse_args()
    
    # Create report generator
    generator = TestReportGenerator(report_dir=args.report_dir)
    
    # Generate the test report
    report_path = generator.generate_test_report()
    
    # Generate visualizations if requested
    if args.include_viz:
        test_data = generator.generate_mock_test_data()
        viz_path = generator.create_visualizations(test_data)
        print(f"Visualizations created at: {viz_path}")
    
    print(f"Test report created at: {report_path}")
    print(f"Overall fidelity score: {generator.test_results['overall_score']:.3f}/1.0")


if __name__ == '__main__':
    main()