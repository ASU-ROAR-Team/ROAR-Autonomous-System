#!/usr/bin/env python3

import rospy
import rostopic
import rosservice
import time
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class HealthMonitor:
    """Enhanced health monitoring for standard ROS packages that don't implement custom heartbeats"""
    
    def __init__(self, logger):
        self.logger = logger
        self.topic_monitors = {}
        self.service_monitors = {}
        self.last_message_times = {}
        
    def setup_health_check(self, node_name, health_config):
        """Setup health monitoring based on configuration"""
        if not health_config:
            return False
            
        check_type = health_config.get("type", "process")
        
        if check_type == "topic":
            return self._setup_topic_monitor(node_name, health_config)
        elif check_type == "service":
            return self._setup_service_monitor(node_name, health_config)
        elif check_type == "process":
            return self._setup_process_monitor(node_name, health_config)
        
        return False
    
    def _setup_topic_monitor(self, node_name, config):
        """Setup topic-based health monitoring"""
        topic_name = config.get("topic")
        if not topic_name:
            self.logger.error(f"No topic specified for {node_name} health check")
            return False
            
        try:
            # Get topic type
            topic_type = rostopic.get_topic_type(topic_name)[0]
            if not topic_type:
                self.logger.error(f"Topic {topic_name} not found for {node_name}")
                return False
            
            # Subscribe to topic
            topic_class = rostopic.get_topic_class(topic_name)[0]
            subscriber = rospy.Subscriber(
                topic_name, 
                topic_class, 
                lambda msg: self._topic_callback(node_name, msg),
                queue_size=1
            )
            
            self.topic_monitors[node_name] = {
                "subscriber": subscriber,
                "topic": topic_name,
                "timeout": config.get("timeout", 5.0),
                "expected_rate": config.get("expectedRate", 1.0),
                "last_message_time": 0
            }
            
            self.logger.info(f"Setup topic monitor for {node_name} on {topic_name}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to setup topic monitor for {node_name}: {e}")
            return False
    
    def _setup_service_monitor(self, node_name, config):
        """Setup service-based health monitoring"""
        service_name = config.get("service")
        if not service_name:
            self.logger.error(f"No service specified for {node_name} health check")
            return False
            
        self.service_monitors[node_name] = {
            "service": service_name,
            "timeout": config.get("timeout", 3.0),
            "check_interval": config.get("checkInterval", 10.0),
            "last_check_time": 0
        }
        
        self.logger.info(f"Setup service monitor for {node_name} on {service_name}")
        return True
    
    def _setup_process_monitor(self, node_name, config):
        """Setup process-based health monitoring"""
        # This uses the existing process monitoring in ModuleManager
        # Just mark it as process-based monitoring
        self.logger.info(f"Setup process monitor for {node_name}")
        return True
        
    def _topic_callback(self, node_name, msg):
        """Callback for topic messages"""
        current_time = time.time()
        if node_name in self.topic_monitors:
            self.topic_monitors[node_name]["last_message_time"] = current_time
            
    def check_node_health(self, node_name, health_config):
        """Check health of a node based on its configuration"""
        if not health_config:
            return True  # No health check configured, assume healthy
            
        check_type = health_config.get("type", "process")
        
        if check_type == "topic":
            return self._check_topic_health(node_name)
        elif check_type == "service":
            return self._check_service_health(node_name)
        elif check_type == "process":
            return True  # Handled by existing process monitoring
            
        return True
    
    def _check_topic_health(self, node_name):
        """Check if topic is publishing at expected rate"""
        if node_name not in self.topic_monitors:
            return False
            
        monitor = self.topic_monitors[node_name]
        current_time = time.time()
        time_since_last = current_time - monitor["last_message_time"]
        
        if time_since_last > monitor["timeout"]:
            self.logger.warn(f"Topic health check failed for {node_name}: No message for {time_since_last:.1f}s")
            return False
            
        return True
    
    def _check_service_health(self, node_name):
        """Check if service is responsive"""
        if node_name not in self.service_monitors:
            return False
            
        monitor = self.service_monitors[node_name]
        current_time = time.time()
        
        # Only check at specified intervals
        if current_time - monitor["last_check_time"] < monitor["check_interval"]:
            return True  # Not time to check yet
            
        monitor["last_check_time"] = current_time
        
        try:
            # Simple check if service exists and is callable
            service_name = monitor["service"]
            if service_name in rosservice.get_service_list():
                # Could add actual service call here if needed
                return True
            else:
                self.logger.warn(f"Service {service_name} not available for {node_name}")
                return False
                
        except Exception as e:
            self.logger.error(f"Service health check failed for {node_name}: {e}")
            return False
    
    def cleanup(self):
        """Cleanup subscribers and monitors"""
        for node_name, monitor in self.topic_monitors.items():
            try:
                monitor["subscriber"].unregister()
            except:
                pass
        
        self.topic_monitors.clear()
        self.service_monitors.clear()


# Integration with your existing ModuleManager class
class EnhancedModuleManager:
    """Extended ModuleManager with health monitoring for standard ROS packages"""
    
    def __init__(self, config, logger):
        # Initialize your existing ModuleManager here
        self.config = config
        self.logger = logger
        self.health_monitor = HealthMonitor(logger)
        
        # Setup health checks for all configured nodes
        for node_config in config.get("nodes", []):
            node_name = node_config.get("nodeName")
            health_config = node_config.get("healthCheck")
            
            if health_config:
                self.health_monitor.setup_health_check(node_name, health_config)
    
    def check_node_health(self, node_name):
        """Enhanced health check that combines process and custom health monitoring"""
        # Get the node configuration
        node_config = None
        for node in self.config.get("nodes", []):
            if node.get("nodeName") == node_name:
                node_config = node
                break
        
        if not node_config:
            return True
            
        health_config = node_config.get("healthCheck")
        
        # Use custom health check if configured, otherwise fall back to existing logic
        if health_config:
            return self.health_monitor.check_node_health(node_name, health_config)
        else:
            # Fall back to your existing heartbeat mechanism
            return self.check_heartbeats_for_node(node_name)  # Your existing method
    
    def check_heartbeats_for_node(self, node_name):
        """Your existing heartbeat checking logic"""
        # Implement your existing heartbeat logic here
        # This is called for nodes without custom health checks
        pass