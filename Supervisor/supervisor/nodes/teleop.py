#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64 # For individual wheel commands if needed, but Twist is more common
from roar_msgs.msg import DrillingCommand, DrillingManualInput # Import custom drilling command/input messages
from sensor_msgs.msg import Joy # For joystick input from GUI

class RoverTeleopModule:
    def __init__(self):
        rospy.init_node('rover_teleop_module')
        self.logger = rospy # Using rospy for logging for simplicity here

        # --- Rover Motion Publishers ---
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Assuming you'll move to Twist for rover motion.
        # If you still need individual wheel control, you'd publish to those topics here:
        # self.velocitylfPublisher = rospy.Publisher('/wheel_lhs_front_velocity_controller/command', Float64, queue_size=10)
        # ... and so on for all 6 wheels. For simplicity, I'll use /cmd_vel.

        # --- Drilling Control Publisher ---
        self.drilling_teleop_cmd_pub = rospy.Publisher('/drilling/command_teleop', DrillingCommand, queue_size=10)

        # --- Subscribers for GUI Input ---
        self.joy_sub = rospy.Subscriber('/joystick/raw_input', Joy, self.joy_callback, queue_size=10)
        self.drilling_manual_input_sub = rospy.Subscriber('/drilling/manual_input', DrillingManualInput, self.drilling_manual_input_callback, queue_size=10)
        
        # --- Internal State for Combining Commands ---
        self._current_twist = Twist()
        self._current_drilling_command = DrillingCommand()
        self._current_drilling_command.target_height_cm = 0.0 # Initialize to safe state
        self._current_drilling_command.gate_open = True
        self._current_drilling_command.auger_on = False
        self._current_drilling_command.manual_up = False
        self._current_drilling_command.manual_down = False

        # --- Parameters for Speed Scaling ---
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 1.0) # m/s
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 0.5) # rad/s

        # Timer to publish commands at a constant rate
        self.publish_rate = rospy.Rate(10) # 10 Hz
        self.publish_timer = rospy.Timer(rospy.Duration(1.0/10.0), self._publish_commands_periodically)

        self.logger.loginfo("Rover Teleop Module initialized. Awaiting input.")

    def joy_callback(self, msg: Joy):
        """Callback for raw joystick input (e.g., from JoystickView.js)."""
        # Assuming msg.axes[0] is for angular (left/right) and msg.axes[1] is for linear (forward/backward)
        # Adjust these indices based on your actual joystick mapping from JoystickView
        normalized_angular = msg.axes[0] # Typically X-axis of left stick
        normalized_linear = msg.axes[1]  # Typically Y-axis of left stick

        self._current_twist.linear.x = normalized_linear * self.max_linear_speed
        self._current_twist.angular.z = normalized_angular * self.max_angular_speed

        # You can also map joystick buttons to drilling manual commands here if desired,
        # but for now, we'll keep drilling buttons separate via DrillingManualInput.

        # self.logger.debug(f"Received Joy: Linear={self._current_twist.linear.x:.2f}, Angular={self._current_twist.angular.z:.2f}")

    def drilling_manual_input_callback(self, msg: DrillingManualInput):
        """Callback for manual drilling input (from DrillingControlView.js)."""
        self._current_drilling_command.manual_up = msg.manual_up
        self._current_drilling_command.manual_down = msg.manual_down
        self._current_drilling_command.auger_on = msg.auger_on
        self._current_drilling_command.gate_open = msg.gate_open
        # Note: target_height_cm is not set here, as manual movement handles continuous updates.
        # If you want to set a fixed target height, it would come from a different GUI element.

        # self.logger.debug(f"Received DrillingManualInput: Up={msg.manual_up}, Down={msg.manual_down}, Auger={msg.auger_on}, Gate={msg.gate_open}")

    def _publish_commands_periodically(self, event):
        """Publishes the current Twist and DrillingCommand at a fixed rate."""
        # Publish rover velocity
        self.cmd_vel_pub.publish(self._current_twist)

        # Publish drilling manual command
        self.drilling_teleop_cmd_pub.publish(self._current_drilling_command)
        
        # self.logger.debug("Published commands periodically.")


if __name__ == '__main__':
    try:
        teleop_module = RoverTeleopModule()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Rover Teleop Module encountered a fatal error: {e}")