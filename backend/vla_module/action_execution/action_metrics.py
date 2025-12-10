"""
Action execution metrics and monitoring for the Vision-Language-Action (VLA) module.

This module collects, processes, and reports metrics for action execution performance,
reliability, and efficiency.
"""

import asyncio
import logging
import time
from typing import Dict, List, Any, Optional, Callable
from dataclasses import dataclass
from datetime import datetime, timedelta
import json
import threading
from collections import defaultdict, deque
import statistics

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import Action, ActionResponse, ExecutionStatus
from ..core.data_models import ActionModel, ActionResponseModel


@dataclass
class ExecutionMetrics:
    """Data class for action execution metrics."""
    action_type: str
    execution_time: float
    success_rate: float
    resource_usage: Dict[str, float]
    timestamp: float
    error_count: int = 0
    total_executions: int = 0


@dataclass
class PerformanceSummary:
    """Data class for performance summary."""
    start_time: float
    end_time: float
    total_actions: int
    completed_actions: int
    failed_actions: int
    avg_execution_time: float
    success_rate: float
    total_execution_time: float
    actions_by_type: Dict[str, int]
    error_distribution: Dict[str, int]
    resource_usage: Dict[str, float]


class ActionMetricsCollector:
    """
    Collects metrics for action executions including performance, reliability, and resource usage.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Store metrics
        self.metrics_storage = deque(maxlen=self.config.get('metrics_history_size', 1000))
        
        # Statistics tracking
        self.execution_times_by_type: Dict[str, List[float]] = defaultdict(list)
        self.success_counts_by_type: Dict[str, int] = defaultdict(int)
        self.failure_counts_by_type: Dict[str, int] = defaultdict(int)
        self.total_counts_by_type: Dict[str, int] = defaultdict(int)
        
        # Performance counters
        self.total_executions = 0
        self.total_completed = 0
        self.total_failed = 0
        self.start_time = time.time()
        
        # Monitoring callbacks
        self.monitoring_callbacks: List[Callable[[ExecutionMetrics], None]] = []
        
        # Threading for async monitoring
        self.monitoring_active = False
        self.monitoring_thread = None
        
        # Real-time metrics
        self.active_actions: Dict[str, float] = {}  # action_id -> start_time
        
        self.logger.info("ActionMetricsCollector initialized")
    
    @log_exception()
    def record_action_start(self, action: ActionModel):
        """
        Record the start of an action execution.
        
        Args:
            action: ActionModel that is starting to execute
        """
        try:
            self.active_actions[action.action_id] = time.time()
            self.logger.debug(f"Started tracking action: {action.action_id}")
        except Exception as e:
            self.logger.error(f"Error recording action start: {e}")
    
    @log_exception()
    def record_action_completion(
        self, 
        action: ActionModel, 
        response: ActionResponseModel, 
        execution_time: float
    ) -> ExecutionMetrics:
        """
        Record the completion of an action execution.
        
        Args:
            action: ActionModel that completed
            response: ActionResponseModel with results
            execution_time: Time taken for execution in seconds
            
        Returns:
            ExecutionMetrics object with recorded metrics
        """
        try:
            # Remove from active actions
            if action.action_id in self.active_actions:
                del self.active_actions[action.action_id]
            
            # Update success/failure counts
            is_successful = response.status == ExecutionStatus.COMPLETED
            action_type = action.action_type
            
            if is_successful:
                self.success_counts_by_type[action_type] += 1
                self.total_completed += 1
            else:
                self.failure_counts_by_type[action_type] += 1
                self.total_failed += 1
            
            self.total_counts_by_type[action_type] += 1
            self.total_executions += 1
            
            # Store execution time
            self.execution_times_by_type[action_type].append(execution_time)
            
            # Create metrics object
            success_rate = (
                self.success_counts_by_type[action_type] / self.total_counts_by_type[action_type] 
                if self.total_counts_by_type[action_type] > 0 
                else 0.0
            )
            
            metrics = ExecutionMetrics(
                action_type=action_type,
                execution_time=execution_time,
                success_rate=success_rate,
                resource_usage=self._collect_resource_usage(action),  # Simplified resource collection
                timestamp=time.time(),
                error_count=0 if is_successful else 1,
                total_executions=self.total_counts_by_type[action_type]
            )
            
            # Store in history
            self.metrics_storage.append(metrics)
            
            # Notify callbacks
            self._notify_callbacks(metrics)
            
            self.logger.debug(f"Recorded completion for action {action.action_id}: {response.status.name} in {execution_time:.3f}s")
            
            return metrics
            
        except Exception as e:
            self.logger.error(f"Error recording action completion: {e}")
            raise VLAException(
                f"Action metrics recording error: {str(e)}", 
                VLAErrorType.MONITORING_ERROR,
                e
            )
    
    def _collect_resource_usage(self, action: ActionModel) -> Dict[str, float]:
        """
        Collect resource usage for an action.
        This is a simplified implementation - in a real system this would interface 
        with system monitoring tools to collect actual resource data.
        
        Args:
            action: ActionModel to collect resource usage for
            
        Returns:
            Dictionary of resource usage metrics
        """
        # For now, return a simulated resource usage
        # In a real implementation, this would collect actual data from the system
        return {
            'cpu_usage_percent': 15.0,      # Simulated CPU usage during action
            'memory_usage_mb': 25.0,        # Simulated memory usage
            'network_io_mb': 0.1,           # Simulated network I/O
            'disk_io_mb': 0.05,             # Simulated disk I/O
            'power_consumption_w': 45.0     # Simulated power draw
        }
    
    def add_monitoring_callback(self, callback: Callable[[ExecutionMetrics], None]):
        """
        Add a callback to be notified when metrics are collected.
        
        Args:
            callback: Function to call when metrics are collected
        """
        self.monitoring_callbacks.append(callback)
        self.logger.debug(f"Added monitoring callback, total: {len(self.monitoring_callbacks)}")
    
    def _notify_callbacks(self, metrics: ExecutionMetrics):
        """
        Notify all registered monitoring callbacks.
        
        Args:
            metrics: ExecutionMetrics to notify callbacks about
        """
        for callback in self.monitoring_callbacks:
            try:
                callback(metrics)
            except Exception as e:
                self.logger.error(f"Error in monitoring callback: {e}")
    
    @log_exception()
    def get_performance_summary(
        self, 
        time_window: Optional[timedelta] = None
    ) -> PerformanceSummary:
        """
        Get performance summary for action executions.
        
        Args:
            time_window: Optional time window to calculate summary for.
                        If None, calculates for all time.
            
        Returns:
            PerformanceSummary with execution statistics
        """
        try:
            if time_window:
                # Filter metrics based on time window
                cutoff_time = time.time() - time_window.total_seconds()
                recent_metrics = [m for m in self.metrics_storage if m.timestamp >= cutoff_time]
            else:
                recent_metrics = list(self.metrics_storage)
            
            if not recent_metrics:
                return PerformanceSummary(
                    start_time=self.start_time,
                    end_time=time.time(),
                    total_actions=0,
                    completed_actions=0,
                    failed_actions=0,
                    avg_execution_time=0.0,
                    success_rate=0.0,
                    total_execution_time=0.0,
                    actions_by_type={},
                    error_distribution={},
                    resource_usage={}
                )
            
            # Calculate aggregate metrics
            total_time = sum(m.execution_time for m in recent_metrics)
            avg_time = total_time / len(recent_metrics) if recent_metrics else 0.0
            
            # Count by action type
            actions_by_type = defaultdict(int)
            for metrics in recent_metrics:
                actions_by_type[metrics.action_type] += 1
            
            # Calculate success rate
            completed_actions = sum(1 for m in recent_metrics if m.success_rate == 1.0)
            success_rate = completed_actions / len(recent_metrics) if recent_metrics else 0.0
            
            # Error distribution (for recent metrics only)
            error_distribution = defaultdict(int)
            for metrics in recent_metrics:
                if metrics.error_count > 0:
                    error_distribution[metrics.action_type] += metrics.error_count
            
            # Aggregate resource usage
            resource_totals = defaultdict(float)
            for metrics in recent_metrics:
                for resource, usage in metrics.resource_usage.items():
                    resource_totals[resource] += usage
            
            # Average resource usage
            if recent_metrics:
                for resource in resource_totals:
                    resource_totals[resource] /= len(recent_metrics)
            
            summary = PerformanceSummary(
                start_time=min(m.timestamp for m in recent_metrics),
                end_time=max(m.timestamp for m in recent_metrics),
                total_actions=len(recent_metrics),
                completed_actions=completed_actions,
                failed_actions=len(recent_metrics) - completed_actions,
                avg_execution_time=avg_time,
                success_rate=success_rate,
                total_execution_time=total_time,
                actions_by_type=dict(actions_by_type),
                error_distribution=dict(error_distribution),
                resource_usage=dict(resource_totals)
            )
            
            return summary
            
        except Exception as e:
            self.logger.error(f"Error generating performance summary: {e}")
            raise VLAException(
                f"Performance summary error: {str(e)}", 
                VLAErrorType.MONITORING_ERROR,
                e
            )
    
    @log_exception()
    async def get_real_time_metrics(self) -> Dict[str, Any]:
        """
        Get real-time metrics for currently executing actions.
        
        Returns:
            Dictionary with real-time execution metrics
        """
        try:
            # Calculate metrics for active actions
            current_time = time.time()
            active_execution_times = {
                action_id: current_time - start_time
                for action_id, start_time in self.active_actions.items()
            }
            
            # Calculate system-wide metrics
            total_executions = self.total_executions
            current_success_rate = (
                self.total_completed / total_executions 
                if total_executions > 0 else 0.0
            )
            
            # Get average execution times by type
            avg_execution_times = {}
            for action_type, times in self.execution_times_by_type.items():
                if times:
                    avg_execution_times[action_type] = statistics.mean(times)
            
            # Calculate recent metrics (last 100 executions)
            recent_metrics = list(self.metrics_storage)[-100:] if self.metrics_storage else []
            recent_success_rate = 0.0
            recent_avg_time = 0.0
            
            if recent_metrics:
                successful_recent = sum(1 for m in recent_metrics if m.success_rate == 1.0)
                recent_success_rate = successful_recent / len(recent_metrics)
                
                recent_total_time = sum(m.execution_time for m in recent_metrics)
                recent_avg_time = recent_total_time / len(recent_metrics)
            
            return {
                'active_actions_count': len(self.active_actions),
                'active_execution_times': active_execution_times,
                'system_total_executions': total_executions,
                'system_total_completed': self.total_completed,
                'system_total_failed': self.total_failed,
                'system_success_rate': current_success_rate,
                'average_execution_times_by_type': avg_execution_times,
                'recent_success_rate': recent_success_rate,
                'recent_average_time': recent_avg_time,
                'execution_rate_per_minute': self._calculate_execution_rate(),
                'resource_usage_summary': self._get_resource_usage_summary()
            }
            
        except Exception as e:
            self.logger.error(f"Error getting real-time metrics: {e}")
            raise VLAException(
                f"Real-time metrics error: {str(e)}", 
                VLAErrorType.MONITORING_ERROR,
                e
            )
    
    def _calculate_execution_rate(self) -> float:
        """
        Calculate the execution rate (actions per minute) over the last 5 minutes.
        
        Returns:
            Execution rate in actions per minute
        """
        time_window = 300.0  # 5 minutes in seconds
        current_time = time.time()
        
        recent_metrics = [
            m for m in self.metrics_storage 
            if current_time - m.timestamp <= time_window
        ]
        
        if not recent_metrics:
            return 0.0
        
        # Calculate actual rate based on time window
        min_timestamp = min(m.timestamp for m in recent_metrics)
        actual_window = current_time - min_timestamp
        
        if actual_window <= 0:
            return 0.0
        
        rate_per_minute = len(recent_metrics) / (actual_window / 60.0)
        return rate_per_minute
    
    def _get_resource_usage_summary(self) -> Dict[str, float]:
        """
        Get a summary of resource usage.
        
        Returns:
            Dictionary with resource usage summary
        """
        if not self.metrics_storage:
            return {}
        
        # Calculate average resource usage across all metrics
        resource_totals = defaultdict(float)
        resource_counts = defaultdict(int)
        
        for metrics in self.metrics_storage:
            for resource, usage in metrics.resource_usage.items():
                resource_totals[resource] += usage
                resource_counts[resource] += 1
        
        # Calculate averages
        avg_resources = {
            resource: total / resource_counts[resource]
            for resource, total in resource_totals.items()
        }
        
        return avg_resources
    
    @log_exception()
    async def export_metrics(self, output_format: str = 'json') -> str:
        """
        Export metrics in the specified format.
        
        Args:
            output_format: Format to export in ('json', 'csv', 'prometheus')
            
        Returns:
            String with exported metrics
        """
        try:
            if output_format.lower() == 'json':
                # Export metrics as JSON
                metrics_list = [
                    {
                        'action_type': m.action_type,
                        'execution_time': m.execution_time,
                        'success_rate': m.success_rate,
                        'resource_usage': m.resource_usage,
                        'timestamp': m.timestamp,
                        'error_count': m.error_count
                    }
                    for m in self.metrics_storage
                ]
                
                summary = self.get_performance_summary()
                
                export_data = {
                    'export_timestamp': time.time(),
                    'performance_summary': {
                        'start_time': summary.start_time,
                        'end_time': summary.end_time,
                        'total_actions': summary.total_actions,
                        'completed_actions': summary.completed_actions,
                        'failed_actions': summary.failed_actions,
                        'avg_execution_time': summary.avg_execution_time,
                        'success_rate': summary.success_rate,
                        'total_execution_time': summary.total_execution_time,
                        'actions_by_type': summary.actions_by_type,
                        'error_distribution': summary.error_distribution,
                        'resource_usage': summary.resource_usage
                    },
                    'individual_metrics': metrics_list
                }
                
                return json.dumps(export_data, indent=2)
            
            elif output_format.lower() == 'csv':
                # Export metrics as CSV
                import csv
                from io import StringIO
                
                output = StringIO()
                fieldnames = [
                    'timestamp', 'action_type', 'execution_time', 'success_rate', 
                    'cpu_usage', 'memory_usage', 'error_count'
                ]
                
                writer = csv.DictWriter(output, fieldnames=fieldnames)
                writer.writeheader()
                
                for metrics in self.metrics_storage:
                    row = {
                        'timestamp': metrics.timestamp,
                        'action_type': metrics.action_type,
                        'execution_time': metrics.execution_time,
                        'success_rate': metrics.success_rate,
                        'cpu_usage': metrics.resource_usage.get('cpu_usage_percent', 0.0),
                        'memory_usage': metrics.resource_usage.get('memory_usage_mb', 0.0),
                        'error_count': metrics.error_count
                    }
                    writer.writerow(row)
                
                return output.getvalue()
            
            elif output_format.lower() == 'prometheus':
                # Export in Prometheus format
                output_lines = []
                
                # Total executions
                output_lines.append(f"# HELP vla_total_executions Total number of action executions")
                output_lines.append(f"# TYPE vla_total_executions counter")
                output_lines.append(f"vla_total_executions {self.total_executions}")
                
                # Total completed
                output_lines.append(f"# HELP vla_total_completed Total number of completed actions")
                output_lines.append(f"# TYPE vla_total_completed counter")
                output_lines.append(f"vla_total_completed {self.total_completed}")
                
                # Total failed
                output_lines.append(f"# HELP vla_total_failed Total number of failed actions")
                output_lines.append(f"# TYPE vla_total_failed counter")
                output_lines.append(f"vla_total_failed {self.total_failed}")
                
                # Success rate
                success_rate = self.total_completed / self.total_executions if self.total_executions > 0 else 0.0
                output_lines.append(f"# HELP vla_success_rate Success rate of action executions")
                output_lines.append(f"# TYPE vla_success_rate gauge")
                output_lines.append(f"vla_success_rate {success_rate}")
                
                # Average execution time by type
                output_lines.append(f"# HELP vla_avg_execution_time_seconds Average execution time by action type")
                output_lines.append(f"# TYPE vla_avg_execution_time_seconds gauge")
                
                for action_type, times in self.execution_times_by_type.items():
                    if times:
                        avg_time = statistics.mean(times)
                        output_lines.append(f'vla_avg_execution_time_seconds{{action_type="{action_type}"}} {avg_time}')
                
                return "\n".join(output_lines)
            
            else:
                raise VLAException(
                    f"Unsupported export format: {output_format}", 
                    VLAErrorType.VALIDATION_ERROR
                )
                
        except Exception as e:
            self.logger.error(f"Error exporting metrics: {e}")
            raise VLAException(
                f"Metrics export error: {str(e)}", 
                VLAErrorType.MONITORING_ERROR,
                e
            )
    
    def get_success_rate_by_type(self, action_type: str) -> float:
        """
        Get the success rate for a specific action type.
        
        Args:
            action_type: Type of action to get success rate for
            
        Returns:
            Success rate as a percentage (0.0-1.0)
        """
        total = self.total_counts_by_type.get(action_type, 0)
        if total == 0:
            return 0.0
        
        successful = self.success_counts_by_type.get(action_type, 0)
        return successful / total
    
    def get_avg_execution_time_by_type(self, action_type: str) -> float:
        """
        Get the average execution time for a specific action type.
        
        Args:
            action_type: Type of action to get average time for
            
        Returns:
            Average execution time in seconds
        """
        times = self.execution_times_by_type.get(action_type, [])
        if not times:
            return 0.0
        
        return statistics.mean(times)
    
    def get_execution_stats_by_type(self, action_type: str) -> Dict[str, Any]:
        """
        Get comprehensive execution statistics for a specific action type.
        
        Args:
            action_type: Type of action to get statistics for
            
        Returns:
            Dictionary with execution statistics
        """
        total = self.total_counts_by_type.get(action_type, 0)
        successful = self.success_counts_by_type.get(action_type, 0)
        failed = self.failure_counts_by_type.get(action_type, 0)
        
        success_rate = successful / total if total > 0 else 0.0
        
        times = self.execution_times_by_type.get(action_type, [])
        avg_time = statistics.mean(times) if times else 0.0
        min_time = min(times) if times else 0.0
        max_time = max(times) if times else 0.0
        
        return {
            'total_executions': total,
            'successful_executions': successful,
            'failed_executions': failed,
            'success_rate': success_rate,
            'average_execution_time': avg_time,
            'min_execution_time': min_time,
            'max_execution_time': max_time,
            'execution_time_stddev': statistics.stdev(times) if len(times) > 1 else 0.0
        }


class ActionMonitor:
    """
    Monitors action execution and provides real-time insights and alerts.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Metrics collector
        self.metrics_collector = ActionMetricsCollector()
        
        # Alert thresholds
        self.alert_thresholds = {
            'success_rate_low': 0.8,  # Alert if success rate drops below 80%
            'execution_time_high': 10.0,  # Alert if execution time exceeds 10s (default)
            'error_rate_high': 0.1  # Alert if error rate exceeds 10%
        }
        
        # Active alerts
        self.active_alerts = []
        
        # Monitoring state
        self.monitoring_active = False
        
        # Alert callbacks
        self.alert_callbacks = []
        
        self.logger.info("ActionMonitor initialized")
    
    def start_monitoring(self):
        """Start the monitoring process."""
        if self.monitoring_active:
            self.logger.warning("Monitoring already active")
            return
        
        self.monitoring_active = True
        self.logger.info("Action monitoring started")
    
    def stop_monitoring(self):
        """Stop the monitoring process."""
        self.monitoring_active = False
        self.logger.info("Action monitoring stopped")
    
    def add_alert_callback(self, callback: Callable[[str, Dict[str, Any]], None]):
        """
        Add a callback to be notified when alerts are generated.
        
        Args:
            callback: Function to call when alerts are generated
        """
        self.alert_callbacks.append(callback)
        self.logger.debug(f"Added alert callback, total: {len(self.alert_callbacks)}")
    
    @log_exception()
    def check_for_anomalies(self, metrics: ExecutionMetrics) -> List[Dict[str, Any]]:
        """
        Check for anomalies in the execution metrics.
        
        Args:
            metrics: Latest ExecutionMetrics to analyze
            
        Returns:
            List of anomaly alerts or empty list if none
        """
        anomalies = []
        
        # Check if success rate is below threshold for this action type
        if metrics.success_rate < self.alert_thresholds['success_rate_low']:
            anomaly = {
                'alert_id': f"low_success_{int(metrics.timestamp)}_{metrics.action_type}",
                'type': 'success_rate_low',
                'severity': 'warning',
                'message': f"Low success rate for {metrics.action_type}: {metrics.success_rate:.2%} < threshold {self.alert_thresholds['success_rate_low']:.2%}",
                'timestamp': metrics.timestamp,
                'details': {
                    'action_type': metrics.action_type,
                    'success_rate': metrics.success_rate,
                    'threshold': self.alert_thresholds['success_rate_low']
                }
            }
            anomalies.append(anomaly)
        
        # Check if execution time is above threshold for this action type
        # We'll use a dynamic threshold based on historical data
        historical_avg = self.metrics_collector.get_avg_execution_time_by_type(metrics.action_type)
        if historical_avg > 0:
            time_ratio = metrics.execution_time / historical_avg
            if time_ratio > 2.0:  # Execution time is more than 2x the average
                anomaly = {
                    'alert_id': f"slow_execution_{int(metrics.timestamp)}_{metrics.action_type}",
                    'type': 'execution_time_high',
                    'severity': 'warning',
                    'message': f"Slow execution for {metrics.action_type}: {metrics.execution_time:.2f}s ({time_ratio:.1f}x avg)",
                    'timestamp': metrics.timestamp,
                    'details': {
                        'action_type': metrics.action_type,
                        'execution_time': metrics.execution_time,
                        'historical_average': historical_avg,
                        'ratio': time_ratio
                    }
                }
                anomalies.append(anomaly)
        
        # Check for repeated errors
        if metrics.error_count > 0:
            error_count_for_type = self.metrics_collector.failure_counts_by_type.get(metrics.action_type, 0)
            total_for_type = self.metrics_collector.total_counts_by_type.get(metrics.action_type, 0)
            
            if total_for_type > 0:
                error_rate = error_count_for_type / total_for_type
                if error_rate > self.alert_thresholds['error_rate_high']:
                    anomaly = {
                        'alert_id': f"high_error_rate_{int(metrics.timestamp)}_{metrics.action_type}",
                        'type': 'error_rate_high',
                        'severity': 'warning',
                        'message': f"High error rate for {metrics.action_type}: {error_rate:.2%} > threshold {self.alert_thresholds['error_rate_high']:.2%}",
                        'timestamp': metrics.timestamp,
                        'details': {
                            'action_type': metrics.action_type,
                            'error_rate': error_rate,
                            'threshold': self.alert_thresholds['error_rate_high']
                        }
                    }
                    anomalies.append(anomaly)
        
        # Trigger alert callbacks for each anomaly
        for anomaly in anomalies:
            self._trigger_alert_callbacks(anomaly)
        
        return anomalies
    
    def _trigger_alert_callbacks(self, alert: Dict[str, Any]):
        """
        Trigger all registered alert callbacks.
        
        Args:
            alert: Alert information to pass to callbacks
        """
        for callback in self.alert_callbacks:
            try:
                callback(alert['type'], alert)
            except Exception as e:
                self.logger.error(f"Error in alert callback: {e}")
    
    @log_exception()
    async def get_performance_insights(self) -> Dict[str, Any]:
        """
        Generate performance insights from the collected metrics.
        
        Returns:
            Dictionary with performance insights
        """
        try:
            insights = {}
            
            # Get performance summary
            summary = self.metrics_collector.get_performance_summary()
            
            # Identify top-performing action types
            top_performers = sorted(
                summary.actions_by_type.items(),
                key=lambda x: self.metrics_collector.get_success_rate_by_type(x[0]),
                reverse=True
            )[:3]
            
            # Identify problematic action types
            problematic = [
                action_type for action_type, count in summary.actions_by_type.items()
                if self.metrics_collector.get_success_rate_by_type(action_type) < 0.8
            ]
            
            # Calculate trends
            recent_metrics = list(self.metrics.collector.metrics_storage)[-20:] if self.metrics_collector.metrics_storage else []
            if len(recent_metrics) > 1:
                first_half = recent_metrics[:len(recent_metrics)//2]
                second_half = recent_metrics[len(recent_metrics)//2:]
                
                early_success_rate = sum(1 for m in first_half if m.success_rate == 1.0) / len(first_half) if first_half else 0.0
                late_success_rate = sum(1 for m in second_half if m.success_rate == 1.0) / len(second_half) if second_half else 0.0
                
                trend = 'improving' if late_success_rate > early_success_rate else 'declining' if late_success_rate < early_success_rate else 'stable'
            else:
                trend = 'insufficient_data'
            
            insights = {
                'system_health': 'healthy' if summary.success_rate > 0.9 else 'concerning' if summary.success_rate > 0.7 else 'unhealthy',
                'top_performing_action_types': top_performers,
                'problematic_action_types': problematic,
                'performance_trend': trend,
                'recommendations': self._generate_recommendations(problematic, trend),
                'efficiency_score': self._calculate_efficiency_score(summary)
            }
            
            return insights
            
        except Exception as e:
            self.logger.error(f"Error generating performance insights: {e}")
            raise VLAException(
                f"Performance insights error: {str(e)}", 
                VLAErrorType.MONITORING_ERROR,
                e
            )
    
    def _generate_recommendations(self, problematic_types: List[str], trend: str) -> List[str]:
        """
        Generate recommendations based on performance data.
        
        Args:
            problematic_types: List of problematic action types
            trend: Current performance trend
            
        Returns:
            List of recommendations
        """
        recommendations = []
        
        if problematic_types:
            recommendations.append(f"Review and improve implementation for action types: {', '.join(problematic_types)}")
        
        if trend == 'declining':
            recommendations.append("Investigate causes of declining performance trend")
        
        if self.metrics_collector.total_executions < 10:
            recommendations.append("More data needed for reliable performance analysis")
        
        # Check resource usage
        resource_summary = self.metrics_collector._get_resource_usage_summary()
        if resource_summary.get('cpu_usage_percent', 0) > 80:
            recommendations.append("High CPU usage detected - consider optimization")
        
        if resource_summary.get('memory_usage_mb', 0) > 100:
            recommendations.append("High memory usage detected - investigate potential leaks")
        
        return recommendations
    
    def _calculate_efficiency_score(self, summary: PerformanceSummary) -> float:
        """
        Calculate an overall efficiency score based on various metrics.
        
        Args:
            summary: PerformanceSummary with metrics
            
        Returns:
            Efficiency score (0.0-1.0)
        """
        # Base score on success rate (40% weight)
        success_weight = 0.4
        success_score = summary.success_rate
        
        # Add execution time consideration (30% weight)
        # Lower execution times get higher scores
        time_weight = 0.3
        # Normalize execution time against a reasonable baseline of 5 seconds
        baseline_avg_time = 5.0
        time_score = max(0.0, min(1.0, (baseline_avg_time / summary.avg_execution_time) if summary.avg_execution_time > 0 else 1.0))
        
        # Add error rate consideration (30% weight)
        error_weight = 0.3
        error_rate = summary.failed_actions / summary.total_actions if summary.total_actions > 0 else 0.0
        error_score = 1.0 - error_rate  # Lower error rate = higher score
        
        efficiency_score = (
            success_score * success_weight +
            time_score * time_weight +
            error_score * error_weight
        )
        
        return efficiency_score


# Global metrics collector and monitor instances
_metrics_collector = None
_action_monitor = None


def get_metrics_collector() -> ActionMetricsCollector:
    """Get the global action metrics collector instance."""
    global _metrics_collector
    if _metrics_collector is None:
        _metrics_collector = ActionMetricsCollector()
    return _metrics_collector


def get_action_monitor() -> ActionMonitor:
    """Get the global action monitor instance."""
    global _action_monitor
    if _action_monitor is None:
        _action_monitor = ActionMonitor()
    return _action_monitor


def record_action_start(action: ActionModel):
    """Convenience function to record action start."""
    collector = get_metrics_collector()
    collector.record_action_start(action)


def record_action_completion(action: ActionModel, response: ActionResponseModel, execution_time: float) -> ExecutionMetrics:
    """Convenience function to record action completion."""
    collector = get_metrics_collector()
    return collector.record_action_completion(action, response, execution_time)


async def get_performance_summary(time_window_hours: float = None) -> PerformanceSummary:
    """Convenience function to get performance summary."""
    collector = get_metrics_collector()
    if time_window_hours:
        return await collector.get_performance_summary(timedelta(hours=time_window_hours))
    else:
        return await collector.get_performance_summary()


async def get_real_time_metrics() -> Dict[str, Any]:
    """Convenience function to get real-time metrics."""
    collector = get_metrics_collector()
    return await collector.get_real_time_metrics()


async def get_performance_insights() -> Dict[str, Any]:
    """Convenience function to get performance insights."""
    monitor = get_action_monitor()
    return await monitor.get_performance_insights()


async def export_metrics(format_type: str = 'json') -> str:
    """Convenience function to export metrics."""
    collector = get_metrics_collector()
    return await collector.export_metrics(format_type)