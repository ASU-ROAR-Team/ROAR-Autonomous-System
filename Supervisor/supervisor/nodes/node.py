# pylint: disable=all
# mypy: ignore-errors
#!/usr/bin/env python3

import rospy
import psutil
import json
import os
import signal
import sys
import atexit
import subprocess
import time
from threading import Lock
from collections import OrderedDict

# Import the new ROS messages and services
from roar_msgs.srv import MissionControl, MissionControlResponse
from roar_msgs.msg import RoverStatus, NodeStatus
from std_msgs.msg import String, Bool # Keep these for existing functionalities

# Assuming SupervisorLogger and ModuleManager are in the same package or path
from supervisor import ModuleManager, SupervisorLogger # Adjust if paths are different

class SupervisorNode:
    def __init__(self):
        rospy.init_node('rover_supervisor', anonymous=False)

        # State management
        self.rover_state = "IDLE"
        self.active_mission = ""
        self.state_lock = Lock()

        # Load configuration file path (before loading config itself)
        self.config_file = rospy.get_param('~config_file',
                                           os.path.expanduser('/home/iaminfadel/ROAR-Simulation/src/ROAR-Autonomous-System/Supervisor/supervisor/config/config.json'))

        # Load configuration
        self.config = self.load_config(self.config_file) # This now uses rospy.log* for initial errors

        # Initialize SupervisorLogger using the loaded config path
        custom_log_path = self.config.get("logDir", None)
        self.logger = SupervisorLogger(
            log_dir=custom_log_path,
            log_prefix=f"rover_supervisor_{self.config.get('taskName', 'default')}",
        )
        
        # Initialize ModuleManager with the loaded config and logger
        self.moduleManager = ModuleManager(self.config, self.logger)
        
        # ROS Setup
        self.setup_ros_interfaces()
        
        # Register cleanup handlers
        rospy.on_shutdown(self.shutdown_hook)
        atexit.register(self.shutdown_hook)
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        # Set up heartbeat subscriber (from your original code)
        self.heartbeat_sub = rospy.Subscriber(
            "/system/heartbeat", String, self.heartbeat_callback, queue_size=10
        )

        # >>> REMOVED: self.start_all_modules() was here. Modules now start via mission control requests. <<<

        # Set up monitoring timers (from your original code)
        self.resource_timer = rospy.Timer(rospy.Duration(1), self.monitor_resources_timer)
        self.heartbeat_timer = rospy.Timer(rospy.Duration(0.5), self.check_heartbeats_timer)
        
        # Status update rate for publishing RoverStatus
        self.status_update_rate = rospy.Rate(1) # 1 Hz
        
        self.logger.info("Rover Supervisor initialized successfully. Awaiting mission command.")
    
    def load_config(self, config_file_path):
        """Load node configuration from JSON file"""
        try:
            if os.path.exists(config_file_path):
                with open(config_file_path, 'r') as f:
                    config_data = json.load(f)
                # Use rospy.loginfo here, as self.logger might not be fully initialized yet
                rospy.loginfo(f"Loaded config from {config_file_path}")
                return config_data
            else:
                rospy.logerr(f"Config file not found at {config_file_path}")
                rospy.signal_shutdown("Configuration file not found.")
                return {}
        except Exception as e:
            rospy.logerr(f"Error loading config: {e}")
            rospy.signal_shutdown(f"Error loading configuration: {e}")
            return {}
            
    def setup_ros_interfaces(self):
        """Initialize ROS publishers, subscribers, and services"""
        # Service server for Mission Control
        self.mission_service = rospy.Service('/mission_control', 
                                             MissionControl, 
                                             self.handle_mission_control)
        
        # Status publisher for RoverStatus
        self.status_pub = rospy.Publisher('/rover_status', 
                                          RoverStatus, 
                                          queue_size=10)
        
        # Emergency Stop Publisher (from your original code)
        self.emergency_stop_pub = rospy.Publisher("/emergency/stop", Bool, queue_size=1)
        
        self.logger.info("ROS interfaces initialized")
    
    # >>> NEW Helper Function for starting all modules <<<
    def start_all_configured_modules(self):
        """Starts all modules defined in the configuration through ModuleManager."""
        self.logger.info("Starting all configured modules...")
        for name in self.moduleManager.modules.keys():
            self.moduleManager.start_module(name)
            self.logger.info(f"Initiated start for module: {name}")

    def handle_mission_control(self, req):
        """Handle mission control service requests"""
        response = MissionControlResponse()
        
        with self.state_lock:
            try:
                if req.request_type.lower() == "start":
                    response = self.start_mission(req.mission_name)
                elif req.request_type.lower() == "stop":
                    response = self.stop_mission()
                elif req.request_type.lower() == "reset":
                    response = self.reset_system()
                else:
                    response.success = False
                    response.message = f"Unknown request type: {req.request_type}"
                    response.current_state = self.rover_state
            except Exception as e:
                response.success = False
                response.message = f"Error processing request: {str(e)}"
                response.current_state = self.rover_state
                self.logger.error(f"Mission control error: {e}")
        
        # Publish updated status after handling the request
        self.publish_status()
        return response
    
    def start_mission(self, mission_name):
        """Start a specific mission (assumed to mean starting all configured modules)"""
        response = MissionControlResponse()
        
        if self.rover_state == "RUNNING":
            response.success = False
            response.message = f"Cannot start {mission_name}: Rover already running mission '{self.active_mission}'"
            response.current_state = self.rover_state
            return response
        
        # Validate if the requested mission name matches your configured taskName
        # or if you have a defined set of missions.
        # For now, we assume any 'start' command triggers the general operational state.
        # You might want a more specific check here if 'mission_name' is truly unique.
        
        # If your config has a 'taskName' and you want to ensure the mission_name matches
        # if mission_name != self.config.get('taskName', 'default'):
        #     response.success = False
        #     response.message = f"Unknown or unconfigured mission: {mission_name}. Current configured task is '{self.config.get('taskName', 'default')}'"
        #     response.current_state = self.rover_state
        #     return response

        try:
            # Start all configured modules when a mission is started
            self.start_all_configured_modules()
            
            self.rover_state = "RUNNING"
            self.active_mission = mission_name # Assign the requested mission name
            self.logger.info(f"Rover state changed to RUNNING, active mission: {mission_name}")
            
            response.success = True
            response.message = f"Successfully started mission: {mission_name}"
            response.current_state = self.rover_state
            
        except Exception as e:
            self.rover_state = "ERROR"
            response.success = False
            response.message = f"Failed to start mission {mission_name}: {str(e)}"
            response.current_state = self.rover_state
            self.logger.error(f"Error starting mission: {e}")
            
        return response
    
    def stop_mission(self):
        """Stop current mission and terminate all active modules, then transition to IDLE."""
        response = MissionControlResponse()
        
        try:
            if self.rover_state == "IDLE":
                response.success = True
                response.message = "Rover is already idle"
                response.current_state = self.rover_state
                return response
            
            self.logger.info(f"Stopping active mission: {self.active_mission}")
            
            # >>> FIX: Terminate all modules when stopping a mission <<<
            self.moduleManager.terminate_all()
            rospy.sleep(1.0) # Give modules a moment to terminate
            
            old_mission = self.active_mission
            self.rover_state = "IDLE"
            self.active_mission = ""
            
            response.success = True
            response.message = f"Successfully stopped mission: {old_mission}. All modules terminated."
            response.current_state = self.rover_state
            
        except Exception as e:
            self.rover_state = "ERROR"
            response.success = False
            response.message = f"Error stopping mission: {str(e)}"
            response.current_state = self.rover_state
            self.logger.error(f"Error stopping mission: {e}")
            
        return response
    
    def reset_system(self):
        """Reset system to initial state, stopping all modules and then restarting them."""
        response = MissionControlResponse()
        
        try:
            self.logger.info("Resetting rover system")
            
            # Always terminate all modules before resetting
            self.moduleManager.terminate_all()
            rospy.sleep(1.0) # Give some time for processes to truly terminate
            
            # Reset state variables
            self.rover_state = "IDLE"
            self.active_mission = ""
            
            # Restart all modules (this implicitly transitions to "RUNNING" if desired later via start_mission)
            # Or you could automatically transition to RUNNING after reset by calling start_mission here
            # self.start_all_configured_modules() # If you want reset to immediately restart everything
            
            response.success = True
            response.message = "System reset successfully. All modules terminated." # Now they are only started by 'start' command
            response.current_state = self.rover_state
            
        except Exception as e:
            self.rover_state = "ERROR"
            response.success = False
            response.message = f"Error resetting system: {str(e)}"
            response.current_state = self.rover_state
            self.logger.error(f"Error resetting system: {e}")
            
        return response
    
    def get_node_status(self, node_name):
        """Get status information for a specific node/module from ModuleManager."""
        node_status = NodeStatus()
        node_status.node_name = node_name
        node_status.cpu_usage = 0.0
        node_status.memory_usage = 0.0
        node_status.pid = -1
        node_status.last_error = ""
        node_status.status = "UNKNOWN" # Default

        with self.state_lock: # Ensure consistency when reading ModuleManager's state
            if node_name in self.moduleManager.modules: # Check if node is defined in config
                if node_name in self.moduleManager.processes: # Check if node is currently managed (started)
                    process_data = self.moduleManager.processes[node_name]
                    proc_obj = process_data["process"]
                    
                    try:
                        # Use psutil.Process to get up-to-date stats
                        ps_proc = psutil.Process(proc_obj.pid)
                        if ps_proc.is_running():
                            # Use ModuleManager's status (healthy, warning, failed) or default to RUNNING
                            mgr_status = process_data.get("status", "healthy").upper()
                            node_status.status = mgr_status if mgr_status in ["HEALTHY", "WARNING", "FAILED"] else "RUNNING"
                            node_status.pid = ps_proc.pid
                            node_status.cpu_usage = ps_proc.cpu_percent(interval=None) / psutil.cpu_count() # Per-core normalized
                            node_status.memory_usage = ps_proc.memory_info().rss / 1024 / 1024 # MB
                        else:
                            node_status.status = "STOPPED"
                            node_status.last_error = "Process not running (pid check)"
                    except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                        node_status.status = "STOPPED"
                        node_status.last_error = "Process not found or accessible by psutil"
                    except Exception as e:
                        node_status.status = "ERROR"
                        node_status.last_error = f"Error getting detailed stats: {str(e)}"
                else:
                    node_status.status = "INACTIVE" # Defined but not running
                    node_status.last_error = "Module not currently launched by ModuleManager"
            else:
                node_status.status = "UNDEFINED" # Not in config at all
                node_status.last_error = "Node name not found in configuration"
                    
        return node_status
    
    def publish_status(self):
        """Publish current rover status and node statuses"""
        status_msg = RoverStatus()
        status_msg.rover_state = self.rover_state
        status_msg.active_mission = self.active_mission
        status_msg.timestamp = rospy.Time.now().to_sec()
        status_msg.supervisor_message = "Supervisor running normally" # Default message
        
        # Check for critical module failures and update supervisor_message/rover_state
        has_critical_failure = False
        all_critical_modules_running = True

        for node_name in self.moduleManager.modules.keys(): # Iterate through all configured modules
            node_status = self.get_node_status(node_name)
            status_msg.node_statuses.append(node_status)

            is_critical = self.moduleManager.modules[node_name].get("critical", False)

            if is_critical:
                if node_status.status == "STOPPED" or node_status.status == "FAILED" or node_status.status == "ERROR":
                    has_critical_failure = True
                    status_msg.supervisor_message = f"CRITICAL ERROR: '{node_name}' has failed or is stopped!"
                    # Don't break, continue to list all node statuses
                # Ensure all critical modules are running/healthy for exiting EMERGENCY_STOP
                if node_status.status not in ["RUNNING", "HEALTHY"]:
                    all_critical_modules_running = False
            
        with self.state_lock:
            if has_critical_failure and self.rover_state != "EMERGENCY_STOP":
                self.rover_state = "EMERGENCY_STOP"
                self.logger.error("Critical module failure detected. Transitioning to EMERGENCY_STOP.")
                self.emergency_stop_pub.publish(Bool(data=True)) # Ensure emergency stop signal is sent
            elif not has_critical_failure and self.rover_state == "EMERGENCY_STOP":
                # Only transition out of EMERGENCY_STOP if ALL critical modules are now running/healthy
                if all_critical_modules_running:
                    self.rover_state = "IDLE" # Revert to IDLE, ready for new mission
                    status_msg.supervisor_message = "All critical issues resolved. Rover state returned to IDLE."


        self.status_pub.publish(status_msg)
    
    def heartbeat_callback(self, msg):
        """Process heartbeat messages from modules"""
        try:
            # Expected format: "module_name:status"
            parts = msg.data.split(":", 1)
            if len(parts) == 2:
                module_name, status = parts
                self.moduleManager.update_heartbeat(module_name, status)
            else:
                # If no status provided, assume healthy
                self.moduleManager.update_heartbeat(parts[0])
        except Exception as e:
            self.logger.error(f"Error processing heartbeat message: {e}")

    def monitor_resources_timer(self, event):
        """Callback for the resource monitoring timer"""
        # Only monitor resources if rover is in a "running" or "idle" state
        # (i.e., not shutting down or in an uninitialized state)
        if self.rover_state in ["IDLE", "RUNNING", "EMERGENCY_STOP", "ERROR"]:
            self.moduleManager.monitor_resources()

    def check_heartbeats_timer(self, event):
        """Callback for the heartbeat checking timer"""
        # Only check heartbeats if rover is in a "running" or "idle" state
        if self.rover_state in ["IDLE", "RUNNING", "EMERGENCY_STOP", "ERROR"]:
            failed_module = self.moduleManager.check_heartbeats()
            if failed_module:
                self.handle_module_failure(failed_module)

    def handle_module_failure(self, name):
        """Handles the failure of a module, including critical failures."""
        with self.state_lock: # Protect state transitions
            if self.moduleManager.modules[name].get("critical", False):
                self.logger.critical(f"Critical module '{name}' failure! Initiating emergency stop.")
                # Always transition to EMERGENCY_STOP if a critical module fails
                self.rover_state = "EMERGENCY_STOP"
                self.active_mission = "" # No active mission during emergency stop
                self.emergency_stop_pub.publish(Bool(data=True)) # Publish emergency stop signal
                self.moduleManager.restart_module(name)
            else:
                self.logger.warn(f"Non-critical module '{name}' failed. Attempting restart.")
                self.moduleManager.restart_module(name)
        
        # Publish status immediately after a failure event
        self.publish_status()

    def shutdown_hook(self):
        """Clean shutdown procedure to terminate all child processes and log."""
        self.logger.info("Shutdown requested - terminating all managed processes")
        if hasattr(self, "moduleManager"):
            self.moduleManager.terminate_all()
        if hasattr(self, "logger"):
            self.logger.info("Supervisor shutting down.")
            self.logger.close() # Assuming your SupervisorLogger has a close method
        rospy.loginfo("Rover Supervisor shutting down.") # Standard ROS log

    def signal_handler(self, sig, frame):
        """Handle termination signals"""
        self.logger.info(f"Received signal {sig} - shutting down")
        self.shutdown_hook()
        sys.exit(0)

    def run(self):
        """Main execution loop for the supervisor node."""
        self.logger.info("Rover Supervisor started. Monitoring system...")
        
        while not rospy.is_shutdown():
            self.publish_status()
            self.status_update_rate.sleep()

if __name__ == '__main__':
    try:
        supervisor = SupervisorNode()
        supervisor.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Rover Supervisor encountered a fatal error: {e}")
        if 'supervisor' in locals() and hasattr(supervisor, 'shutdown_hook'):
            supervisor.shutdown_hook()