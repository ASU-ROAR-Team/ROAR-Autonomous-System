#!/usr/bin/env python3

import rospy
import smach
import time
from dataclasses import dataclass
from typing import Dict, Any, List, Optional, Generator

# Import standard ROS messages and custom messages/services
from std_msgs.msg import Float64, String # Added String for FSM state publishing
from roar_msgs.msg import DrillingCommand as DrillingCommandMsg # <--- Changed here!
from roar_msgs.srv import StartModule, StartModuleResponse
from std_srvs.srv import Trigger, TriggerResponse # Import for the new StopModule service

# --- Global variables to store sensor data ---
current_height = 0.0
current_weight = 0.0

# Flag to signal that the platform has reached its target height
platform_reached_target = False

# Global flag to signal FSM preemption from the stop service
stop_requested = False

# ROS Publisher for FSM state
fsm_state_publisher = None

# --- ROS Parameters ---
SURFACE_HEIGHT = 0.0
SAMPLING_HEIGHT = 0.0
GATE_OPEN_DELAY_TIME = 0.0
ASCENT_DELAY_TIME = 0.0
FSM_TIMEOUT = 0.0

# --- CAN Communication Setup (for a self-contained example) ---
@dataclass
class CanMessage:
    """A simplified representation of a CAN message."""
    can_id: int
    dlc: int
    data: List[int]
    
    def __post_init__(self):
        if len(self.data) > 8: raise ValueError("CAN data payload cannot exceed 8 bytes")
        if self.dlc != len(self.data): raise ValueError("DLC must match data length")
        if not 0 <= self.can_id <= 0xFFFF: raise ValueError("CAN ID must be between 0 and 0xFFFF")
    
    def to_bytes(self) -> bytes:
        message = bytearray([0xAA])
        message.extend(self.can_id.to_bytes(2, 'big'))
        message.append(self.dlc)
        message.extend(self.data)
        message.append(self._calculate_checksum())
        message.append(0x55)
        return bytes(message)
    
    def _calculate_checksum(self) -> int:
        checksum = (self.can_id >> 8) ^ (self.can_id & 0xFF) ^ self.dlc
        for byte in self.data:
            checksum ^= byte
        return checksum

class BaseParser:
    """Base class with helper methods for parsing."""
    def extract_16bit_values(self, data: List[int], num_values: int) -> List[int]:
        values = []
        for i in range(num_values):
            if i * 2 + 1 < len(data):
                value = (data[i * 2] << 8) | data[i * 2 + 1]
                values.append(value)
        return values
    
class DrillingParser(BaseParser):
    SCALING_FACTOR = 10.0
    DRIVING_CAN_ID = 0x01
    FEEDBACK_CAN_ID = 0x02

    def parse(self, message: CanMessage) -> Optional[Dict[str, Any]]:
        if message.can_id != self.FEEDBACK_CAN_ID:
            return None
        
        if message.dlc != 4:
            rospy.logwarn(f"DrillingParser: Received message with unexpected DLC: {message.dlc}. Expected 4.")
            return None

        try:
            height_raw = self.extract_16bit_values(message.data[0:2], 1)[0]
            height_cm = float(height_raw) / self.SCALING_FACTOR
            
            weight_raw = self.extract_16bit_values(message.data[2:4], 1)[0]
            weight_g = float(weight_raw)

            return {
                "platform_height_cm": height_cm,
                "load_cell_g": weight_g,
            }
        except (IndexError, ValueError) as e:
            rospy.logerr(f"Error parsing drilling status message: {e}")
            return None

    def create_command_message(self, target_height_cm: float, gate_open: bool, auger_on: bool, manual_up: bool, manual_down: bool) -> CanMessage:
        height_raw = int(target_height_cm * self.SCALING_FACTOR)
        if not (0 <= height_raw <= 65535):
            rospy.logwarn(f"Target height {target_height_cm} cm (raw {height_raw}) out of 16-bit unsigned range. Clamping.")
            height_raw = max(0, min(65535, height_raw))

        height_bytes = height_raw.to_bytes(2, 'big', signed=False)

        control_byte = 0x00
        if gate_open:
            control_byte |= (1 << 0)
        if auger_on:
            control_byte |= (1 << 1)
        if manual_up:
            control_byte |= (1 << 2)
        if manual_down:
            control_byte |= (1 << 3)
        
        if manual_up and manual_down:
            rospy.logwarn("Both manual_up and manual_down commanded. Defaulting to STOP for platform motor.")
            control_byte &= ~((1 << 2) | (1 << 3)) # Clear both bits
        
        payload = [
            height_bytes[0],
            height_bytes[1],
            control_byte
        ]
        return CanMessage(can_id=self.DRIVING_CAN_ID, dlc=3, data=payload)

class MockUartCanInterface:
    def __init__(self, port: str, baudrate: int = 115200):
        rospy.loginfo(f"Mocking UartCanInterface on {port}@{baudrate}")
        self._is_connected = True
        self._mock_messages_to_receive = []

    def connect(self) -> None:
        self._is_connected = True
        rospy.loginfo("Mock UART connected.")

    def disconnect(self) -> None:
        self._is_connected = False
        rospy.loginfo("Mock UART disconnected.")

    def send_message(self, message: CanMessage) -> None:
        if not self._is_connected:
            raise ConnectionError("Mock UART not connected")
        rospy.logdebug(f"Mock UART SENT: ID={hex(message.can_id)}, DLC={message.dlc}, Data={message.data}")

    def receive_messages(self) -> Generator[Optional[CanMessage], None, None]:
        # This mock will endlessly yield None if no messages are pre-loaded.
        # In a real system, this would block or yield actual messages from the UART.
        while self._is_connected and not rospy.is_shutdown():
            if self._mock_messages_to_receive:
                yield self._mock_messages_to_receive.pop(0)
            else:
                yield None # Yield None to prevent StopIteration
            rospy.sleep(0.01)

    def is_connected(self) -> bool:
        return self._is_connected

class CanCommander:
    def __init__(self):
        self.parser = DrillingParser()
        self.uart_interface = MockUartCanInterface('/dev/ttyCAN0')
        self.current_command_msg = DrillingCommandMsg() # <--- Changed here!

        try:
            self.uart_interface.connect()
        except ConnectionError as e:
            rospy.logerr(f"Failed to connect to UART: {e}")

        # Timers are created here, which now happens after rospy.init_node()
        self.send_timer = rospy.Timer(rospy.Duration(0.1), self._send_command_callback)
        self.receive_timer = rospy.Timer(rospy.Duration(0.05), self._receive_feedback_callback)

    def _send_command_callback(self, event):
        self.send_drilling_command_internal(self.current_command_msg)

    def _receive_feedback_callback(self, event):
        feedback_data = self.uart_interface.receive_messages().__next__()
        if feedback_data:
            parsed = self.parser.parse(feedback_data)
            if parsed:
                global current_height, current_weight, platform_reached_target
                current_height = parsed.get("platform_height_cm", 0.0)
                current_weight = parsed.get("load_cell_g", 0.0)
                
                # Check if target is reached based on CAN feedback
                # This is a simplified check assuming the low-level system sends a status
                if abs(self.current_command_msg.target_height_cm - current_height) < 0.5:
                    platform_reached_target = True
                else:
                    platform_reached_target = False

    def send_drilling_command_internal(self, command_msg: DrillingCommandMsg):
        can_message = self.parser.create_command_message(
            target_height_cm=command_msg.target_height_cm,
            gate_open=command_msg.gate_open,
            auger_on=command_msg.auger_on,
            manual_up=command_msg.manual_up,
            manual_down=command_msg.manual_down
        )
        try:
            self.uart_interface.send_message(can_message)
        except ConnectionError as e:
            rospy.logerr(f"Failed to send CAN message: {e}")
    
    def set_current_command(self, command_msg: DrillingCommandMsg):
        self.current_command_msg = command_msg
        self.send_drilling_command_internal(command_msg)

# --- State Definitions ---
# The ManualControl state is removed as manual commands are now handled directly by GUI
# publishing to drilling_command topic when FSM is in IDLE.

class Idle(smach.State):
    def __init__(self, can_commander_instance):
        smach.State.__init__(self, outcomes=['start_drilling_command', 'preempted'])
        self.trigger_start = False
        self.can_commander = can_commander_instance

    def handle_start_module(self, req):
        rospy.loginfo("Start drilling command received via service.")
        self.trigger_start = True
        return StartModuleResponse(success=True)

    def execute(self, userdata):
        global stop_requested, fsm_state_publisher
        rospy.loginfo("State: IDLE - The state machine has successfully returned to idle.")
        fsm_state_publisher.publish(String(data="IDLE"))

        # Ensure actuators are in a safe, known state when idle
        self.can_commander.set_current_command(DrillingCommandMsg( # <--- Changed here!
            target_height_cm=0.0, # Home position
            gate_open=True,       # Gate open for safety/manual interaction
            auger_on=False        # Auger off
        ))

        self.trigger_start = False
        stop_requested = False # Reset stop request when entering IDLE
        
        # Loop while waiting for start command or preemption
        while not self.trigger_start and not rospy.is_shutdown():
            if self.preempt_requested() or stop_requested:
                rospy.loginfo("IDLE state preempted by external request.")
                stop_requested = False # Reset for next use
                return 'preempted'
            rospy.sleep(0.1)
        
        return 'start_drilling_command'

class DescendingToSurface(smach.State):
    def __init__(self, can_commander_instance):
        smach.State.__init__(self, outcomes=['reached_surface', 'timeout', 'preempted'])
        self.can_commander = can_commander_instance
    
    def execute(self, userdata):
        global platform_reached_target, stop_requested, fsm_state_publisher
        rospy.loginfo("State: DESCENDING TO SURFACE")
        fsm_state_publisher.publish(String(data="DESCENDING_TO_SURFACE"))
        
        platform_reached_target = False
        
        rate = rospy.Rate(10)
        start_time = rospy.get_time()
        
        while not platform_reached_target and not rospy.is_shutdown():
            if stop_requested:
                rospy.loginfo("Preemption requested in DESCENDING_TO_SURFACE.")
                return 'preempted'
            if (rospy.get_time() - start_time) > FSM_TIMEOUT:
                rospy.logerr(f"Timeout: Platform failed to reach {SURFACE_HEIGHT} cm.")
                return 'timeout'
            
            cmd_msg = DrillingCommandMsg( # <--- Changed here!
                target_height_cm=SURFACE_HEIGHT, gate_open=True, auger_on=False
            )
            self.can_commander.set_current_command(cmd_msg)

            rospy.logdebug(f"Waiting for platform to reach {SURFACE_HEIGHT} cm...")
            rate.sleep()

        rospy.loginfo(f"Platform has reached {SURFACE_HEIGHT} cm.")
        return 'reached_surface'

class DescendingToSample(smach.State):
    def __init__(self, can_commander_instance):
        smach.State.__init__(self, outcomes=['reached_sampling_point', 'timeout', 'preempted'])
        self.can_commander = can_commander_instance
    
    def execute(self, userdata):
        global platform_reached_target, stop_requested, fsm_state_publisher
        rospy.loginfo("State: DESCENDING TO SAMPLE")
        fsm_state_publisher.publish(String(data="DESCENDING_TO_SAMPLE"))

        platform_reached_target = False
        
        rate = rospy.Rate(10)
        start_time = rospy.get_time()
        
        while not platform_reached_target and not rospy.is_shutdown():
            if stop_requested:
                rospy.loginfo("Preemption requested in DESCENDING_TO_SAMPLE.")
                return 'preempted'
            if (rospy.get_time() - start_time) > FSM_TIMEOUT:
                rospy.logerr(f"Timeout: Platform failed to reach {SAMPLING_HEIGHT} cm.")
                return 'timeout'
            
            cmd_msg = DrillingCommandMsg( # <--- Changed here!
                target_height_cm=SAMPLING_HEIGHT, gate_open=True, auger_on=True
            )
            self.can_commander.set_current_command(cmd_msg)
            
            rospy.logdebug(f"Waiting for platform to reach {SAMPLING_HEIGHT} cm...")
            rate.sleep()

        rospy.loginfo(f"Platform has reached {SAMPLING_HEIGHT} cm.")
        rospy.loginfo(f"Waiting for {GATE_OPEN_DELAY_TIME} seconds before closing gate...")
        # Check preemption during sleep
        sleep_start_time = rospy.get_time()
        while (rospy.get_time() - sleep_start_time) < GATE_OPEN_DELAY_TIME and not rospy.is_shutdown():
            if stop_requested:
                rospy.loginfo("Preemption requested during GATE_OPEN_DELAY_TIME.")
                return 'preempted'
            rospy.sleep(0.1)

        rospy.loginfo("Delay complete. Proceeding to collecting state.")
        return 'reached_sampling_point'

class CollectingSample(smach.State):
    def __init__(self, can_commander_instance):
        smach.State.__init__(self, outcomes=['load_cell_ready', 'timeout', 'preempted'])
        self.can_commander = can_commander_instance

    def execute(self, userdata):
        global stop_requested, fsm_state_publisher
        rospy.loginfo("State: COLLECTING SAMPLE")
        fsm_state_publisher.publish(String(data="COLLECTING_SAMPLE"))
        
        rate = rospy.Rate(10)
        start_time = rospy.get_time()
        
        while current_weight < 100 and not rospy.is_shutdown():
            if stop_requested:
                rospy.loginfo("Preemption requested in COLLECTING_SAMPLE.")
                return 'preempted'
            if (rospy.get_time() - start_time) > FSM_TIMEOUT:
                rospy.logerr("Timeout: Failed to collect sample of 100g.")
                self.can_commander.set_current_command(DrillingCommandMsg(target_height_cm=SAMPLING_HEIGHT)) # <--- Changed here!
                return 'timeout'

            cmd_msg = DrillingCommandMsg( # <--- Changed here!
                target_height_cm=SAMPLING_HEIGHT, gate_open=False, auger_on=True
            )
            self.can_commander.set_current_command(cmd_msg)
            
            rospy.logdebug(f"Current weight: {current_weight:.2f} g. Waiting for 100 g...")
            rate.sleep()
        
        rospy.loginfo("Sample collected (>= 100g). Waiting for ascent delay...")
        # Check preemption during sleep
        sleep_start_time = rospy.get_time()
        while (rospy.get_time() - sleep_start_time) < ASCENT_DELAY_TIME and not rospy.is_shutdown():
            if stop_requested:
                rospy.loginfo("Preemption requested during ASCENT_DELAY_TIME.")
                return 'preempted'
            rospy.sleep(0.1)
        
        self.can_commander.set_current_command(DrillingCommandMsg( # <--- Changed here!
            target_height_cm=SAMPLING_HEIGHT, gate_open=False, auger_on=False
        ))
        rospy.loginfo("Ascent delay complete. Moving to next state.")
        return 'load_cell_ready'

class AscendingToTop(smach.State):
    def __init__(self, can_commander_instance):
        smach.State.__init__(self, outcomes=['reached_start_position', 'timeout', 'preempted'])
        self.can_commander = can_commander_instance
    
    def execute(self, userdata):
        global platform_reached_target, stop_requested, fsm_state_publisher
        rospy.loginfo("State: ASCENDING TO TOP")
        fsm_state_publisher.publish(String(data="ASCENDING_TO_TOP"))
        
        platform_reached_target = False
        
        rate = rospy.Rate(10)
        start_time = rospy.get_time()

        while not platform_reached_target and not rospy.is_shutdown():
            if stop_requested:
                rospy.loginfo("Preemption requested in ASCENDING_TO_TOP.")
                return 'preempted'
            if (rospy.get_time() - start_time) > FSM_TIMEOUT:
                rospy.logerr("Timeout: Platform failed to reach home position.")
                return 'timeout'

            cmd_msg = DrillingCommandMsg( # <--- Changed here!
                target_height_cm=0.0, gate_open=False, auger_on=False
            )
            self.can_commander.set_current_command(cmd_msg)
            
            rospy.logdebug("Waiting for platform to reach 0 cm...")
            rate.sleep()
        
        # After reaching home, ensure gate is open and auger is off for safety
        final_cmd_msg = DrillingCommandMsg( # <--- Changed here!
            target_height_cm=0.0, gate_open=True, auger_on=False
        )
        self.can_commander.set_current_command(final_cmd_msg)
        rospy.loginfo("Reached home position. Gate opened.")
        return 'reached_start_position'

class EmergencyStop(smach.State):
    def __init__(self, can_commander_instance):
        smach.State.__init__(self, outcomes=['repaired', 'preempted'])
        self.can_commander = can_commander_instance

    def execute(self, userdata):
        global stop_requested, fsm_state_publisher
        rospy.logerr("State: EMERGENCY_STOP - A critical error occurred.")
        fsm_state_publisher.publish(String(data="EMERGENCY_STOP"))
        rospy.loginfo("Shutting down all actuators for safety.")
        
        # Command to a safe state
        safe_cmd = DrillingCommandMsg( # <--- Changed here!
            target_height_cm=0.0, gate_open=True, auger_on=False
        )
        self.can_commander.set_current_command(safe_cmd)

        rospy.loginfo("System is now in a safe state. Please inspect and repair the issue.")
        rospy.loginfo("Waiting for external signal to return to IDLE state (e.g., stop_requested reset).")
        
        # This state will stay active until an external stop is requested or node shuts down
        while not rospy.is_shutdown():
            if stop_requested:
                rospy.loginfo("Preemption requested in EMERGENCY_STOP. Returning to IDLE.")
                stop_requested = False # Reset for next use
                return 'preempted'
            rospy.sleep(0.1)
        
        return 'repaired' # This outcome should ideally be triggered by an explicit 'repair' mechanism

# Service handler for stopping the FSM
def handle_stop_module(req):
    global stop_requested
    rospy.loginfo("Stop module service call received. Requesting FSM preemption.")
    stop_requested = True # Set the global flag to signal preemption
    return TriggerResponse(success=True, message="Stop requested.")

def main():
    global SURFACE_HEIGHT, SAMPLING_HEIGHT, GATE_OPEN_DELAY_TIME, ASCENT_DELAY_TIME, FSM_TIMEOUT, fsm_state_publisher
    
    rospy.init_node('drilling_state_machine')
    
    # Initialize FSM state publisher
    fsm_state_publisher = rospy.Publisher('/drilling_fsm_state', String, queue_size=10)

    # Instantiate CanCommander after rospy.init_node()
    can_commander = CanCommander()

    SURFACE_HEIGHT = rospy.get_param('~surface_height', 11.0)
    SAMPLING_HEIGHT = rospy.get_param('~sampling_height', 40.0)
    GATE_OPEN_DELAY_TIME = rospy.get_param('~gate_open_delay_time', 5.0)
    ASCENT_DELAY_TIME = rospy.get_param('~ascent_delay_time', 2.0)
    FSM_TIMEOUT = rospy.get_param('~fsm_timeout', 60.0)

    rospy.loginfo(f"Parameters loaded: Surface={SURFACE_HEIGHT}, Sampling={SAMPLING_HEIGHT}, Gate Open Delay={GATE_OPEN_DELAY_TIME}, Ascent Delay={ASCENT_DELAY_TIME}, FSM Timeout={FSM_TIMEOUT}")
    
    idle_state = Idle(can_commander)
    rospy.Service('start_module', StartModule, idle_state.handle_start_module)
    rospy.Service('stop_module', Trigger, handle_stop_module) # New stop service

    sm_top = smach.StateMachine(outcomes=['succeeded', 'preempted'])
    # Set IDLE as the initial state for the top-level state machine
    sm_top.set_initial_state(['IDLE'])

    with sm_top:
        # IDLE state: waits for a start command from the GUI, or handles preemption
        smach.StateMachine.add('IDLE', idle_state, 
                               transitions={'start_drilling_command':'DRILLING_SEQUENCE',
                                            'preempted':'IDLE'}) # Return to IDLE if preempted

        # Nested state machine for the autonomous drilling sequence
        sm_drilling = smach.StateMachine(outcomes=['drilling_succeeded', 'preempted', 'timeout_error'])
        
        # --- REMOVED THE LINE BELOW ---
        # sm_drilling.register_preempt_callback(lambda: rospy.loginfo("Drilling Sequence Preempted externally (container level)."))

        with sm_drilling:
            smach.StateMachine.add('DESCENDING_TO_SURFACE', DescendingToSurface(can_commander),
                                    transitions={'reached_surface':'DESCENDING_TO_SAMPLE',
                                                 'timeout':'timeout_error',
                                                 'preempted':'preempted'}) # Handle preemption to exit
            smach.StateMachine.add('DESCENDING_TO_SAMPLE', DescendingToSample(can_commander),
                                    transitions={'reached_sampling_point':'COLLECTING_SAMPLE',
                                                 'timeout':'timeout_error',
                                                 'preempted':'preempted'}) # Handle preemption to exit
            smach.StateMachine.add('COLLECTING_SAMPLE', CollectingSample(can_commander),
                                    transitions={'load_cell_ready':'ASCENDING_TO_TOP',
                                                 'timeout':'timeout_error',
                                                 'preempted':'preempted'}) # Handle preemption to exit
            smach.StateMachine.add('ASCENDING_TO_TOP', AscendingToTop(can_commander),
                                    transitions={'reached_start_position':'drilling_succeeded',
                                                 'timeout':'timeout_error',
                                                 'preempted':'preempted'}) # Handle preemption to exit
        
        # Add the drilling sequence to the top-level state machine
        smach.StateMachine.add('DRILLING_SEQUENCE', sm_drilling,
                               transitions={'drilling_succeeded':'IDLE',
                                            'preempted':'IDLE', # Go to IDLE if drilling sequence is preempted
                                            'timeout_error':'EMERGENCY_STOP'})
        
        # Emergency Stop state: for critical errors, allows manual intervention or reset
        smach.StateMachine.add('EMERGENCY_STOP', EmergencyStop(can_commander),
                               transitions={'repaired':'IDLE', # Manual repair acknowledged, return to IDLE
                                            'preempted':'IDLE'}) # Allow external stop to return to IDLE from emergency

    rospy.loginfo("Drilling State Machine Initialized and ready in IDLE state. Waiting for GUI commands.")
    try:
        # Execute the top-level state machine. This call is blocking until the state machine finishes
        # (e.g., preempted or reaches 'succeeded' which is not currently mapped).
        outcome = sm_top.execute()
        rospy.loginfo(f"State machine finished with outcome: {outcome}")
    except rospy.ROSInterruptException:
        rospy.loginfo("State machine interrupted (ROS node shutdown). Shutting down.")
    finally:
        # Ensure that on shutdown, the current command is a safe one.
        can_commander.set_current_command(DrillingCommandMsg( # <--- Changed here!
            target_height_cm=0.0, gate_open=True, auger_on=False
        ))
        rospy.loginfo("Actuators commanded to safe state on FSM shutdown.")

if __name__ == '__main__':
    main()
