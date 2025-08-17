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

# Import ROS messages and services
from roar_msgs.srv import MissionControl, MissionControlResponse
from roar_msgs.msg import RoverStatus, NodeStatus
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, TriggerResponse # For calling drilling start service

# Assuming SupervisorLogger and ModuleManager are in the same package or path
from supervisor import ModuleManager, SupervisorLogger # Removed DrillingModule import

class SupervisorNode:
    def __init__(self):
        rospy.init_node('rover_supervisor', anonymous=False)

        # State management
        self.rover_state = "IDLE"
        self.active_mission = ""
        self.state_lock = Lock()

        # Load configuration file path (before loading config itself)
        self.config_file = rospy.get_param('~config_file', '')
        
        if not self.config_file:
            task_name = rospy.get_param('~task_name', 'navigation').lower()
            config_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
            self.config_file = os.path.join(config_dir, 'config.json') # Always load the main config.json

        # Load configuration
        self.config = self.load_config(self.config_file)

        # Initialize SupervisorLogger using the loaded config path
        custom_log_path = self.config.get("logDir", None)
        self.logger = SupervisorLogger(
            log_dir=custom_log_path,
            log_prefix=f"rover_supervisor_{self.config.get('taskName', 'default')}",
        )
        
        # Initialize ModuleManager with the FULL config.
        # It will store all node definitions but not start them automatically.
        self.moduleManager = ModuleManager(self.config, self.logger)
        
        # ROS Setup
        self.setup_ros_interfaces()
        
        # Register cleanup handlers
        rospy.on_shutdown(self.shutdown_hook)
        atexit.register(self.shutdown_hook)
        signal.signal(signal.SIGINT, signal.SIG_DFL) # Use default handler for SIGINT (Ctrl+C)
        signal.signal(signal.SIGTERM, self.signal_handler) # Use custom handler for SIGTERM

        # Set up heartbeat subscriber
        self.heartbeat_sub = rospy.Subscriber(
            "/system/heartbeat", String, self.heartbeat_callback, queue_size=10
        )

        # IMPORTANT: Removed self.start_all_configured_modules() from here.
        # Modules are now started only when a mission is explicitly requested.

        # Set up monitoring timers
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
        
        # Emergency Stop Publisher
        self.emergency_stop_pub = rospy.Publisher("/emergency/stop", Bool, queue_size=1)
        
        self.logger.info("ROS interfaces initialized")
    
    # Removed start_all_configured_modules() as it's no longer needed

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
        
        self.publish_status()
        return response
    
    def start_mission(self, mission_name):
        """Start a specific mission by launching its configured modules."""
        response = MissionControlResponse()
        
        if self.rover_state == "RUNNING":
            response.success = False
            response.message = f"Cannot start {mission_name}: Rover already running mission '{self.active_mission}'"
            response.current_state = self.rover_state
            return response
        
        try:
            # First, terminate any currently running modules from a previous mission
            self.moduleManager.terminate_all()
            rospy.sleep(0.5) # Give some time for processes to clean up

            # Get the list of modules for the requested mission
            mission_modules = self.config.get('missions', {}).get(mission_name, {}).get('modules', [])
            
            if not mission_modules:
                response.success = False
                response.message = f"No modules defined for mission: {mission_name}"
                response.current_state = self.rover_state
                self.logger.error(f"Attempted to start mission '{mission_name}' but no modules were found in config.")
                return response

            self.logger.info(f"Starting modules for mission: {mission_name}")
            for module_name in mission_modules:
                self.moduleManager.start_module(module_name)
                self.logger.info(f"Initiated start for module: {module_name}")
            
            self.rover_state = "RUNNING"
            self.active_mission = mission_name
            self.logger.info(f"Rover state changed to RUNNING, active mission: {mission_name}")
            
            # --- Call Drilling Module's start service if it's a drilling mission ---
            if mission_name.lower() == "drilling":
                self.logger.info("Attempting to start Drilling Module FSM via service call...")
                try:
                    rospy.wait_for_service('/drilling/start_module', timeout=10)
                    start_drilling_fsm = rospy.ServiceProxy('/drilling/start_module', Trigger)
                    fsm_response = start_drilling_fsm()
                    if fsm_response.success:
                        self.logger.info(f"Drilling Module FSM started: {fsm_response.message}")
                    else:
                        self.logger.error(f"Failed to start Drilling Module FSM: {fsm_response.message}")
                        self.rover_state = "ERROR" # Indicate partial failure
                        response.message += f" (Drilling FSM error: {fsm_response.message})"
                        response.success = False
                except rospy.ServiceException as e:
                    self.logger.error(f"Drilling start service call failed: {e}")
                    self.rover_state = "ERROR" # Indicate critical failure
                    response.message += f" (Drilling FSM service unavailable: {e})"
                    response.success = False
            # --- END NEW ---

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
            
            # Call stop service on drilling module if active
            if self.active_mission.lower() == "drilling":
                self.logger.info("Attempting to stop Drilling Module via service call...")
                try:
                    rospy.wait_for_service('/drilling/stop_module', timeout=5)
                    stop_drilling_fsm = rospy.ServiceProxy('/drilling/stop_module', Trigger)
                    fsm_response = stop_drilling_fsm()
                    if fsm_response.success:
                        self.logger.info(f"Drilling Module stop requested: {fsm_response.message}")
                    else:
                        self.logger.warn(f"Failed to request stop for Drilling Module: {fsm_response.message}")
                except rospy.ServiceException as e:
                    self.logger.warn(f"Drilling stop service call failed: {e}")
            
            self.moduleManager.terminate_all() # Terminate all modules from any mission
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
        """Reset system to initial state, stopping all modules."""
        response = MissionControlResponse()
        
        try:
            self.logger.info("Resetting rover system")
            
            # Always terminate all modules
            self.moduleManager.terminate_all()
            rospy.sleep(1.0) # Give some time for processes to truly terminate
            
            # Reset state variables
            self.rover_state = "IDLE"
            self.active_mission = ""
            
            response.success = True
            response.message = "System reset successfully. All modules terminated."
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

        # Only check statuses of modules that are *expected* to be running for the current mission
        active_mission_modules = self.config.get('missions', {}).get(self.active_mission, {}).get('modules', [])

        for node_name in active_mission_modules: # Iterate through *only* active mission modules
            node_status = self.get_node_status(node_name)
            status_msg.node_statuses.append(node_status) # Still add to message

            is_critical = self.moduleManager.modules[node_name].get("critical", False) # Get critical flag from full definition

            if is_critical:
                if node_status.status in ["STOPPED", "FAILED", "ERROR"]:
                    has_critical_failure = True
                    status_msg.supervisor_message = f"CRITICAL ERROR: '{node_name}' has failed or is stopped!"
                # Ensure all critical modules are running/healthy for exiting EMERGENCY_STOP
                if node_status.status not in ["RUNNING", "HEALTHY"]:
                    all_critical_modules_running = False
            
        with self.state_lock:
            if has_critical_failure and self.rover_state != "EMERGENCY_STOP":
                self.rover_state = "EMERGENCY_STOP"
                self.logger.error("Critical module failure detected. Transitioning to EMERGENCY_STOP.")
                self.emergency_stop_pub.publish(Bool(data=True))
            elif not has_critical_failure and self.rover_state == "EMERGENCY_STOP":
                # Only transition out of EMERGENCY_STOP if ALL critical modules are now running/healthy
                # and if no mission is active or current mission's modules are healthy
                if all_critical_modules_running:
                    self.rover_state = "IDLE"
                    status_msg.supervisor_message = "All critical issues resolved. Rover state returned to IDLE."

        self.status_pub.publish(status_msg)
    
    def heartbeat_callback(self, msg):
        """Process heartbeat messages from modules"""
        try:
            parts = msg.data.split(":", 1)
            if len(parts) == 2:
                module_name, status = parts
                self.moduleManager.update_heartbeat(module_name, status)
            else:
                self.moduleManager.update_heartbeat(parts[0])
        except Exception as e:
            self.logger.error(f"Error processing heartbeat message: {e}")

    def monitor_resources_timer(self, event):
        """Callback for the resource monitoring timer"""
        if self.rover_state in ["IDLE", "RUNNING", "EMERGENCY_STOP", "ERROR"]:
            self.moduleManager.monitor_resources()

    def check_heartbeats_timer(self, event):
        """Callback for the heartbeat checking timer"""
        if self.rover_state in ["IDLE", "RUNNING", "EMERGENCY_STOP", "ERROR"]:
            failed_module = self.moduleManager.check_heartbeats()
            if failed_module:
                self.handle_module_failure(failed_module)

    def handle_module_failure(self, name):
        """Handles the failure of a module, including critical failures."""
        with self.state_lock:
            if self.moduleManager.modules[name].get("critical", False):
                self.logger.critical(f"Critical module '{name}' failure! Initiating emergency stop.")
                self.rover_state = "EMERGENCY_STOP"
                self.active_mission = ""
                self.emergency_stop_pub.publish(Bool(data=True))
                # Attempt to restart the critical module, supervisor will stay in EMERGENCY_STOP
                self.moduleManager.restart_module(name) 
            else:
                self.logger.warn(f"Non-critical module '{name}' failed. Attempting restart.")
                self.moduleManager.restart_module(name)
        
        self.publish_status()

    def shutdown_hook(self):
        """Clean shutdown procedure to terminate all child processes and log."""
        self.logger.info("Shutdown requested - terminating all managed processes")
        if hasattr(self, "moduleManager"):
            self.moduleManager.terminate_all()
        if hasattr(self, "logger"):
            self.logger.info("Supervisor shutting down.")
            self.logger.close()
        rospy.loginfo("Rover Supervisor shutting down.")

    def signal_handler(self, sig, frame):
        """Handle termination signals (e.g., SIGTERM, but SIGINT uses default)"""
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
