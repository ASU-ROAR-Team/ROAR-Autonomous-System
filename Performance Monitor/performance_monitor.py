#!/usr/bin/env python3
"""
Generic Performance Monitor Module

A flexible performance monitoring system that can track timing, metrics, and statistics
for any application. Provides both real-time monitoring and logging capabilities.

Usage:
    monitor = PerformanceMonitor(enable_logging=True, log_file_path="performance.log")
    monitor.start_frame()
    
    monitor.start_timer("processing_stage")
    # ... do some work ...
    duration = monitor.end_timer("processing_stage")
    
    monitor.record_metric("items_processed", 100)
    monitor.end_frame()
    monitor.log_metrics()
"""

import time
import json
import csv
import os
from datetime import datetime
from typing import Dict, Any, Optional, List
from dataclasses import dataclass, field
from collections import defaultdict, deque
import threading


@dataclass
class PerformanceMetrics:
    """Container for performance metrics data"""
    # Timing data
    timers: Dict[str, float] = field(default_factory=dict)
    
    # Custom metrics
    metrics: Dict[str, Any] = field(default_factory=dict)
    
    # Frame information
    frame_id: str = ""
    timestamp: float = 0.0
    
    # Processing statistics
    total_time_ms: float = 0.0
    frame_count: int = 0
    
    # Performance flags
    flags: Dict[str, bool] = field(default_factory=dict)


@dataclass
class MonitorParams:
    """Configuration parameters for the performance monitor"""
    enable_detailed_timing: bool = True
    enable_debug_output: bool = True
    enable_performance_logging: bool = True
    timing_report_interval: float = 1.0  # seconds
    log_file_path: str = ""
    max_history_size: int = 100
    csv_logging: bool = False
    json_logging: bool = False


class PerformanceMonitor:
    """
    Generic Performance Monitor for tracking timing and metrics
    
    Features:
    - Multi-stage timing measurement
    - Custom metrics recording
    - Real-time statistics
    - File logging (CSV/JSON)
    - Thread-safe operation
    """
    
    def __init__(self, params: Optional[MonitorParams] = None):
        """
        Initialize the performance monitor
        
        Args:
            params: Configuration parameters, uses defaults if None
        """
        self.params = params or MonitorParams()
        
        # Current frame data
        self.current_metrics = PerformanceMetrics()
        self.active_timers: Dict[str, float] = {}
        
        # Historical data for statistics
        self.frame_times: deque = deque(maxlen=self.params.max_history_size)
        self.timer_history: Dict[str, deque] = defaultdict(
            lambda: deque(maxlen=self.params.max_history_size)
        )
        self.metric_history: Dict[str, deque] = defaultdict(
            lambda: deque(maxlen=self.params.max_history_size)
        )
        
        # Statistics
        self.frame_count = 0
        self.last_report_time = time.time()
        
        # Thread safety
        self._lock = threading.Lock()
        
        # Initialize logging
        if self.params.log_file_path and self.params.enable_performance_logging:
            self._initialize_logging()
    
    def _initialize_logging(self):
        """Initialize log files"""
        if self.params.csv_logging:
            self._create_csv_header()
        if self.params.json_logging:
            self._create_json_log()
    
    def _create_csv_header(self):
        """Create CSV file with headers"""
        try:
            with open(self.params.log_file_path + '.csv', 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # Basic headers - will be extended dynamically
                headers = ['timestamp', 'frame_id', 'total_time_ms', 'frame_count']
                writer.writerow(headers)
        except Exception as e:
            print(f"Warning: Could not create CSV log file: {e}")
    
    def _create_json_log(self):
        """Create JSON log file"""
        try:
            with open(self.params.log_file_path + '.json', 'w') as jsonfile:
                json.dump({"performance_log": []}, jsonfile)
        except Exception as e:
            print(f"Warning: Could not create JSON log file: {e}")
    
    def start_frame(self, frame_id: str = ""):
        """
        Start timing a new frame/iteration
        
        Args:
            frame_id: Optional identifier for this frame
        """
        with self._lock:
            self.current_metrics = PerformanceMetrics()
            self.current_metrics.frame_id = frame_id
            self.current_metrics.timestamp = time.time()
            self.current_metrics.frame_count = self.frame_count
            self.active_timers.clear()
    
    def end_frame(self):
        """End the current frame and update statistics"""
        with self._lock:
            end_time = time.time()
            self.current_metrics.total_time_ms = (end_time - self.current_metrics.timestamp) * 1000
            
            # Update statistics
            self.frame_times.append(self.current_metrics.total_time_ms)
            self.frame_count += 1
            
            # Update timer history
            for timer_name, duration in self.current_metrics.timers.items():
                self.timer_history[timer_name].append(duration)
            
            # Update metric history
            for metric_name, value in self.current_metrics.metrics.items():
                if isinstance(value, (int, float)):
                    self.metric_history[metric_name].append(value)
    
    def start_timer(self, stage_name: str):
        """
        Start timing a specific stage
        
        Args:
            stage_name: Name of the stage to time
        """
        with self._lock:
            self.active_timers[stage_name] = time.time()
    
    def end_timer(self, stage_name: str) -> float:
        """
        End timing for a specific stage
        
        Args:
            stage_name: Name of the stage to end timing for
            
        Returns:
            Duration in milliseconds
        """
        with self._lock:
            if stage_name in self.active_timers:
                end_time = time.time()
                duration_ms = (end_time - self.active_timers[stage_name]) * 1000
                self.current_metrics.timers[stage_name] = duration_ms
                del self.active_timers[stage_name]
                return duration_ms
            return 0.0
    
    def record_metric(self, metric_name: str, value: Any):
        """
        Record a custom metric
        
        Args:
            metric_name: Name of the metric
            value: Value to record (can be any type)
        """
        with self._lock:
            self.current_metrics.metrics[metric_name] = value
    
    def set_flag(self, flag_name: str, value: bool = True):
        """
        Set a performance flag
        
        Args:
            flag_name: Name of the flag
            value: Flag value
        """
        with self._lock:
            self.current_metrics.flags[flag_name] = value
    
    def get_current_metrics(self) -> PerformanceMetrics:
        """Get current frame metrics"""
        with self._lock:
            return self.current_metrics
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        Get performance statistics
        
        Returns:
            Dictionary containing various statistics
        """
        with self._lock:
            stats = {
                'frame_count': self.frame_count,
                'avg_frame_time_ms': 0.0,
                'min_frame_time_ms': 0.0,
                'max_frame_time_ms': 0.0,
                'timer_stats': {},
                'metric_stats': {}
            }
            
            if self.frame_times:
                stats['avg_frame_time_ms'] = sum(self.frame_times) / len(self.frame_times)
                stats['min_frame_time_ms'] = min(self.frame_times)
                stats['max_frame_time_ms'] = max(self.frame_times)
            
            # Timer statistics
            for timer_name, history in self.timer_history.items():
                if history:
                    stats['timer_stats'][timer_name] = {
                        'avg_ms': sum(history) / len(history),
                        'min_ms': min(history),
                        'max_ms': max(history),
                        'count': len(history)
                    }
            
            # Metric statistics
            for metric_name, history in self.metric_history.items():
                if history:
                    stats['metric_stats'][metric_name] = {
                        'avg': sum(history) / len(history),
                        'min': min(history),
                        'max': max(history),
                        'count': len(history)
                    }
            
            return stats
    
    def print_summary(self):
        """Print a summary of current performance"""
        current = self.get_current_metrics()
        stats = self.get_statistics()
        
        print(f"\n=== PERFORMANCE SUMMARY ===")
        print(f"Frame: {current.frame_id} | Total time: {current.total_time_ms:.2f} ms")
        print(f"Frame count: {self.frame_count}")
        
        if current.timers:
            print(f"\nTiming breakdown:")
            for stage, duration in current.timers.items():
                print(f"  {stage}: {duration:.2f} ms")
        
        if current.metrics:
            print(f"\nMetrics:")
            for name, value in current.metrics.items():
                print(f"  {name}: {value}")
        
        if stats['avg_frame_time_ms'] > 0:
            print(f"\nStatistics (last {len(self.frame_times)} frames):")
            print(f"  Avg frame time: {stats['avg_frame_time_ms']:.2f} ms")
            print(f"  Min frame time: {stats['min_frame_time_ms']:.2f} ms")
            print(f"  Max frame time: {stats['max_frame_time_ms']:.2f} ms")
    
    def print_detailed_timing(self):
        """Print detailed timing information"""
        current = self.get_current_metrics()
        
        print(f"\n=== DETAILED TIMING BREAKDOWN ===")
        print(f"Frame ID: {current.frame_id}")
        print(f"Timestamp: {datetime.fromtimestamp(current.timestamp)}")
        print(f"Total time: {current.total_time_ms:.2f} ms")
        
        if current.timers:
            print(f"\nStage timings:")
            for stage, duration in sorted(current.timers.items()):
                percentage = (duration / current.total_time_ms * 100) if current.total_time_ms > 0 else 0
                print(f"  {stage:20s}: {duration:8.2f} ms ({percentage:5.1f}%)")
    
    def log_metrics(self):
        """Log current metrics to file"""
        if not self.params.enable_performance_logging or not self.params.log_file_path:
            return
        
        current = self.get_current_metrics()
        
        try:
            if self.params.csv_logging:
                self._log_to_csv(current)
            if self.params.json_logging:
                self._log_to_json(current)
        except Exception as e:
            print(f"Warning: Failed to log metrics: {e}")
    
    def _log_to_csv(self, metrics: PerformanceMetrics):
        """Log metrics to CSV file"""
        csv_path = self.params.log_file_path + '.csv'
        
        # Prepare row data
        row = [
            metrics.timestamp,
            metrics.frame_id,
            metrics.total_time_ms,
            metrics.frame_count
        ]
        
        # Add timer data
        for timer_name, duration in metrics.timers.items():
            row.append(duration)
        
        # Add metric data
        for metric_name, value in metrics.metrics.items():
            row.append(value)
        
        with open(csv_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(row)
    
    def _log_to_json(self, metrics: PerformanceMetrics):
        """Log metrics to JSON file"""
        json_path = self.params.log_file_path + '.json'
        
        # Convert metrics to dictionary
        log_entry = {
            'timestamp': metrics.timestamp,
            'frame_id': metrics.frame_id,
            'total_time_ms': metrics.total_time_ms,
            'frame_count': metrics.frame_count,
            'timers': metrics.timers,
            'metrics': metrics.metrics,
            'flags': metrics.flags
        }
        
        # Read existing data
        try:
            with open(json_path, 'r') as jsonfile:
                data = json.load(jsonfile)
        except (FileNotFoundError, json.JSONDecodeError):
            data = {"performance_log": []}
        
        # Add new entry
        data["performance_log"].append(log_entry)
        
        # Write back
        with open(json_path, 'w') as jsonfile:
            json.dump(data, jsonfile, indent=2)
    
    def reset(self):
        """Reset all statistics and history"""
        with self._lock:
            self.current_metrics = PerformanceMetrics()
            self.active_timers.clear()
            self.frame_times.clear()
            self.timer_history.clear()
            self.metric_history.clear()
            self.frame_count = 0
            self.last_report_time = time.time()
    
    def set_params(self, params: MonitorParams):
        """Update monitor parameters"""
        with self._lock:
            self.params = params
            if self.params.log_file_path and self.params.enable_performance_logging:
                self._initialize_logging()


# Convenience functions for quick usage
def create_monitor(enable_logging: bool = True, 
                  log_file_path: str = "",
                  csv_logging: bool = False,
                  json_logging: bool = False) -> PerformanceMonitor:
    """
    Create a performance monitor with common settings
    
    Args:
        enable_logging: Enable file logging
        log_file_path: Path for log files
        csv_logging: Enable CSV logging
        json_logging: Enable JSON logging
        
    Returns:
        Configured PerformanceMonitor instance
    """
    params = MonitorParams(
        enable_performance_logging=enable_logging,
        log_file_path=log_file_path,
        csv_logging=csv_logging,
        json_logging=json_logging
    )
    return PerformanceMonitor(params)


# Context manager for easy timing
class TimerContext:
    """Context manager for timing code blocks"""
    
    def __init__(self, monitor: PerformanceMonitor, stage_name: str):
        self.monitor = monitor
        self.stage_name = stage_name
    
    def __enter__(self):
        self.monitor.start_timer(self.stage_name)
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.monitor.end_timer(self.stage_name)


# Decorator for timing functions
def time_function(monitor: PerformanceMonitor, stage_name: str = None):
    """
    Decorator to automatically time function execution
    
    Args:
        monitor: PerformanceMonitor instance
        stage_name: Name for the timer (defaults to function name)
    """
    def decorator(func):
        def wrapper(*args, **kwargs):
            timer_name = stage_name or func.__name__
            monitor.start_timer(timer_name)
            try:
                result = func(*args, **kwargs)
                return result
            finally:
                monitor.end_timer(timer_name)
        return wrapper
    return decorator


if __name__ == "__main__":
    # Example usage
    monitor = create_monitor(
        enable_logging=True,
        log_file_path="example_performance",
        csv_logging=True,
        json_logging=True
    )
    
    # Example frame processing
    monitor.start_frame("test_frame_001")
    
    monitor.start_timer("data_loading")
    time.sleep(0.1)  # Simulate work
    monitor.end_timer("data_loading")
    
    monitor.start_timer("processing")
    time.sleep(0.2)  # Simulate work
    monitor.end_timer("processing")
    
    monitor.record_metric("items_processed", 150)
    monitor.record_metric("error_count", 0)
    monitor.set_flag("success", True)
    
    monitor.end_frame()
    monitor.print_summary()
    monitor.log_metrics()
    
    # Example with context manager
    monitor.start_frame("test_frame_002")
    
    with TimerContext(monitor, "context_example"):
        time.sleep(0.05)  # Simulate work
    
    monitor.end_frame()
    monitor.print_detailed_timing() 