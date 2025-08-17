# pylint: disable=all
# mypy: ignore-errors
#!/usr/bin/env python3

import rospy
import time
from dataclasses import dataclass
from typing import Dict, Any, List, Optional, Generator
import threading

# Import standard ROS messages and custom messages/services
from std_msgs.msg import Float64, String
from roar_msgs.msg import DrillingCommand as DrillingCommandMsg
from roar_msgs.msg import DrillingStatus as DrillingStatusMsg # Import the custom message
from std_srvs.srv import Trigger, TriggerResponse # For start/stop mission services

class DrillingModule:
    def __init__(self, logger):
        self.logger = logger
        self.supervisor_callback = None  # Not directly used in this ROS service model
        self.mode = rospy.get_param('~mode', 'drilling')  # Mode: 'drilling' or 'teleop'
        self.fsm_state = "IDLE" # Initialize FSM state here

        # --- Global variables to store sensor data ---
        self.current_height = 0.0
        self.current_weight = 0.0
        self.platform_reached_target = False

        # Global flag to signal operation preemption
        self.stop_requested = False
        self.drilling_sequence_thread = None

        # ROS Parameters
        self.SURFACE_HEIGHT = rospy.get_param('~surface_height', 11.0)
        self.SAMPLING_HEIGHT = rospy.get_param('~sampling_height', 40.0)
        self.GATE_OPEN_DELAY_TIME = rospy.get_param('~gate_open_delay_time', 5.0)
        self.ASCENT_DELAY_TIME = rospy.get_param('~ascent_delay_time', 2.0)
        self.FSM_TIMEOUT = rospy.get_param('~fsm_timeout', 60.0)
        
        # Initialize current_command_msg to a safe, idle state
        self.current_command_msg = DrillingCommandMsg() 
        self.current_command_msg.target_height_cm = 0.0 # Home position, no movement
        self.current_command_msg.gate_open = True     # Gate open by default for safety
        self.current_command_msg.auger_on = False     # Auger off by default

        self.logger.info(f"Drilling module initialized in {self.mode} mode")
        self.logger.info(f"Drilling parameters loaded: Surface={self.SURFACE_HEIGHT}, "
                         f"Sampling={self.SAMPLING_HEIGHT}, Gate Open Delay={self.GATE_OPEN_DELAY_TIME}, "
                         f"Ascent Delay={self.ASCENT_DELAY_TIME}, FSM Timeout={self.FSM_TIMEOUT}")

        # --- ROS Communication Setup ---
        
        # Publishers
        self.fsm_state_publisher = rospy.Publisher('/drilling_fsm_state', String, queue_size=10)
        # This topic receives commands from autonomous FSM 
        self.drilling_command_pub = rospy.Publisher('/drilling/command_to_actuators', DrillingCommandMsg, queue_size=10) 

        # Service servers for supervisor
        self.start_module_service = rospy.Service('/drilling/start_module', Trigger, self.handle_start_module_service)
        self.stop_module_service = rospy.Service('/drilling/stop_module', Trigger, self.handle_stop_module_service)
        
        # Subscriber for DrillingStatus messages (feedback from bridge)
        self.feedback_sub = rospy.Subscriber('/drilling/feedback', DrillingStatusMsg, self.feedback_callback, queue_size=10)

        # Timer for constant rate publishing of current command
        self.command_publish_rate = rospy.Rate(10) # Publish at 10 Hz
        self.command_publish_timer = rospy.Timer(rospy.Duration(1.0 / 10.0), self._publish_current_command_periodically)

        self.logger.info("Drilling module initialized")
        self.send_state_update("IDLE") # Initial state

    def feedback_callback(self, msg):
        """Handle feedback from the drilling bridge node using the custom message"""
        self.current_height = msg.current_height
        self.current_weight = msg.current_weight
        self.logger.debug(f"Received feedback: height={self.current_height}cm, weight={self.current_weight}g")

        # Check if platform has reached target height (within tolerance) for FSM progression
        # Note: self.current_command_msg must accurately reflect the last *target* sent by FSM.
        target_height = self.current_command_msg.target_height_cm
        height_tolerance = 0.5  # cm tolerance
        if abs(self.current_height - target_height) <= height_tolerance:
            self.platform_reached_target = True
            self.logger.debug(f"Platform reached target height: {self.current_height} cm (target: {target_height} cm)")
        else:
            self.platform_reached_target = False

    def _publish_current_command_periodically(self, event):
        """
        Callback for the timer to publish the current command at a constant rate.
        This function now controls when commands are sent based on the module's mode and FSM state.
        """
        # Publish the command that is currently set in self.current_command_msg
        self.drilling_command_pub.publish(self.current_command_msg)
        self.logger.debug(f"Periodically publishing command: target={self.current_command_msg.target_height_cm:.2f}, auger={self.current_command_msg.auger_on}, gate={self.current_command_msg.gate_open}")


    def handle_start_module_service(self, req):
        """Service handler to start the autonomous drilling sequence, called by supervisor."""
        self.logger.info("Start drilling mission service received.")
        
        if self.mode == 'teleop':
            self.logger.warn("Ignoring start mission service call: module is in teleoperation mode.")
            return TriggerResponse(success=False, message="Module configured for teleoperation, not autonomous start.")

        if self.drilling_sequence_thread and self.drilling_sequence_thread.is_alive():
            self.logger.warn("Drilling sequence already in progress.")
            return TriggerResponse(success=False, message="Drilling sequence already in progress.")

        self.reset_stop_request() # Reset stop flag before starting new sequence
        self._set_safe_idle_command() # Ensure it's in a safe state before sequence begins
        # Start the drilling sequence in a separate thread to not block the service call
        self.drilling_sequence_thread = threading.Thread(target=self.execute_drilling_sequence)
        self.drilling_sequence_thread.daemon = True # Allow main program to exit even if thread is running
        self.drilling_sequence_thread.start()
        
        return TriggerResponse(success=True, message="Drilling sequence initiation commanded.")

    def handle_stop_module_service(self, req):
        """Service handler to stop the drilling operations, called by supervisor."""
        self.logger.info("Stop module service call received. Requesting operation preemption.")
        self.stop_requested = True # Signal the FSM thread to stop
        
        # Command a safe state immediately and set FSM to IDLE
        self._set_safe_idle_command() # Use helper for consistency
        self.send_state_update("IDLE") # Return to IDLE state immediately

        return TriggerResponse(success=True, message="Stop requested and actuators commanded to safe state.")

    def _set_safe_idle_command(self):
        """Helper to set current_command_msg to a safe, idle state."""
        self.current_command_msg.target_height_cm = 0.0 # Go to home
        self.current_command_msg.gate_open = True     # Gate open for safety
        self.current_command_msg.auger_on = False     # Auger off for safety

    def reset_stop_request(self):
        """Reset the stop request flag"""
        self.stop_requested = False

    def is_stop_requested(self):
        """Check if stop has been requested"""
        return self.stop_requested

    def send_state_update(self, state):
        """Publish the current state of the drilling module FSM"""
        self.fsm_state = state # Update internal state
        self.fsm_state_publisher.publish(String(data=state))

    def command_actuators(self, target_height_cm=0.0, gate_open=True, auger_on=False):
        """
        Updates the internal state of the current command. The periodic timer
        will then publish this state to /drilling/command_to_actuators.
        """
        self.logger.debug(f"Updating internal command state: target_height={target_height_cm}, gate_open={gate_open}, auger_on={auger_on}")
        self.current_command_msg.target_height_cm = target_height_cm
        self.current_command_msg.gate_open = gate_open
        self.current_command_msg.auger_on = auger_on

    def wait_for_height(self, target_height, gate_open=True, auger_on=False):
        """Wait until platform reaches target height or timeout occurs"""
        self.logger.info(f"wait_for_height called with target_height={target_height}")
        
        self.platform_reached_target = False # Reset flag for each new wait operation
        start_time = rospy.get_time()
        
        height_tolerance = 0.5  # cm tolerance

        # Command once to initiate movement (and update internal command state)
        self.command_actuators(target_height_cm=target_height, gate_open=gate_open, auger_on=auger_on)

        while not self.platform_reached_target and not rospy.is_shutdown():
            if self.is_stop_requested():
                self.logger.info("Operation preempted by external request during wait_for_height.")
                return False
                
            if (rospy.get_time() - start_time) > self.FSM_TIMEOUT:
                self.logger.error(f"Timeout: Platform failed to reach {target_height} cm. Current height: {self.current_height}. Returning failure.")
                return False
            
            # Continuously update command to maintain target until reached or preempted
            # The periodic publisher will send this command.
            self.command_actuators(target_height_cm=target_height, gate_open=gate_open, auger_on=auger_on)
            rospy.sleep(0.1) # Check and command at 10 Hz
            
        return True

    def wait_with_stop_check(self, duration):
        """Wait for a specified duration, checking for stop requests"""
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time) < duration and not rospy.is_shutdown():
            if self.is_stop_requested():
                self.logger.info("Operation preempted during wait period.")
                return False
            rospy.sleep(0.1)
        return True

    def collect_sample(self):
        """Collect sample until target weight is reached or timeout occurs"""
        self.logger.info("Starting sample collection.")
        start_time = rospy.get_time()
        
        while self.current_weight < 100 and not rospy.is_shutdown(): # Assuming 100g is target sample weight
            if self.is_stop_requested():
                self.logger.info("Operation preempted during sample collection.")
                return False
                
            if (rospy.get_time() - start_time) > self.FSM_TIMEOUT:
                self.logger.error("Timeout: Failed to collect sample of 100g. Returning failure.")
                # Ensure auger is off and platform is at sampling height on timeout
                self.command_actuators(target_height_cm=self.SAMPLING_HEIGHT, gate_open=False, auger_on=False) 
                return False

            self.command_actuators(target_height_cm=self.SAMPLING_HEIGHT, gate_open=False, auger_on=True)
            rospy.sleep(0.1)
            
        self.logger.info("Sample collection complete.")
        return True

    def execute_drilling_sequence(self):
        """Execute the complete autonomous drilling sequence (FSM steps)"""
        self.logger.info(f"Starting drilling sequence with parameters: Surface={self.SURFACE_HEIGHT}, Sampling={self.SAMPLING_HEIGHT}")
        self.reset_stop_request()
        
        try:
            # 1. Descend to surface (gate open, auger off)
            self.send_state_update("DESCENDING_TO_SURFACE")
            if not self.wait_for_height(self.SURFACE_HEIGHT, gate_open=True, auger_on=False):
                self.logger.error("Failed to descend to surface.")
                self.emergency_stop() # Call emergency stop for a failed sequence
                return False

            # 2. Descend to sampling point (gate open, auger on for initial cut)
            self.send_state_update("DESCENDING_TO_SAMPLE")
            if not self.wait_for_height(self.SAMPLING_HEIGHT, gate_open=True, auger_on=True):
                self.logger.error("Failed to descend to sampling point.")
                self.emergency_stop()
                return False
                
            # 3. Close gate, continue augering to collect sample
            self.send_state_update("COLLECTING_SAMPLE")
            # Ensure gate is closed for collection and auger is on
            self.command_actuators(target_height_cm=self.SAMPLING_HEIGHT, gate_open=False, auger_on=True)
            if not self.collect_sample():
                self.logger.error("Failed to collect sample.")
                self.emergency_stop()
                return False
                
            # 4. Wait for ascent delay (auger still on to clear, gate closed)
            self.logger.info(f"Sample collected. Waiting for ascent delay of {self.ASCENT_DELAY_TIME} seconds...")
            if not self.wait_with_stop_check(self.ASCENT_DELAY_TIME):
                self.logger.error("Failed during ascent delay.")
                self.emergency_stop()
                return False
                
            # 5. Ascend to top (auger off, gate closed - holding sample)
            self.send_state_update("ASCENDING_TO_TOP")
            # Turn off auger before ascending to prevent damage/waste
            self.command_actuators(target_height_cm=0.0, gate_open=False, auger_on=False) 
            if not self.wait_for_height(0.0, gate_open=False, auger_on=False):
                self.logger.error("Failed to ascend to top.")
                self.emergency_stop()
                return False
                
            # 6. Open gate at top position (auger off) to dispense sample
            self.send_state_update("DISPENSING_SAMPLE")
            self.command_actuators(target_height_cm=0.0, gate_open=True, auger_on=False)
            self.logger.info("Drilling sequence completed successfully. Sample dispensed.")
            self.send_state_update("IDLE") # Return to IDLE state
            return True
            
        except Exception as e:
            self.logger.error(f"Error during drilling sequence execution: {e}")
            self.logger.error("Transitioning to Emergency Stop due to unhandled exception.")
            self.emergency_stop()
            return False

    def emergency_stop(self):
        """Execute emergency stop procedure"""
        self.logger.error("EMERGENCY_STOP - A critical error occurred.")
        self.send_state_update("EMERGENCY_STOP") # Still set this temporarily for display
        self.logger.info("Shutting down all actuators for safety.")
        
        # Command to a safe state: home position, gate open, auger off
        self._set_safe_idle_command() # Use helper for consistency
        # The periodic publisher will immediately send this safe command.
        
        self.logger.info("System is now in a safe state and returning to IDLE.")
        self.send_state_update("IDLE") # Return to IDLE after achieving safe state

    def shutdown(self):
        """Perform clean shutdown"""
        self.logger.info("Shutting down drilling module")
        self.stop_requested = True # Signal any running FSM threads to stop
        if self.drilling_sequence_thread and self.drilling_sequence_thread.is_alive():
            self.drilling_sequence_thread.join(timeout=2.0) # Wait for thread to finish gracefully
        
        # Command actuators to a safe state on shutdown
        self._set_safe_idle_command() # Use helper for consistency
        # The periodic publisher will send this last safe command.

        self.logger.info("Actuators commanded to safe state on shutdown.")


if __name__ == '__main__':
    rospy.init_node('drilling_module_node', anonymous=False)
    try:
        from supervisor import SupervisorLogger
        logger = SupervisorLogger()
    except ImportError:
        class MockLogger:
            def info(self, msg): rospy.loginfo(msg)
            def debug(self, msg): rospy.logdebug(msg)
            def warn(self, msg): rospy.logwarn(msg)
            def error(self, msg): rospy.logerr(msg)
            def critical(self, msg): rospy.logfatal(msg)
            def log_exception(self, msg): rospy.logerr(msg)
        logger = MockLogger()
    
    try:
        drilling_module = DrillingModule(logger)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        logger.log_exception(f"Drilling Module encountered a fatal error: {e}")