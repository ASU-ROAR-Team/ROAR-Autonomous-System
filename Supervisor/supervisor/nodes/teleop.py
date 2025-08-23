#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64 
from roar_msgs.msg import DrillingCommand, DrillingManualInput 
from sensor_msgs.msg import Joy 

class RoverTeleopModule:
    def __init__(self):
        rospy.init_node('rover_teleop_module')
        self.logger = rospy 

        # --- Rover Motion Publishers ---
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # --- Drilling Control Publisher ---
        self.drilling_teleop_cmd_pub = rospy.Publisher('/drilling/command_to_actuators', DrillingCommand, queue_size=10)

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


        self.logger.loginfo("Rover Teleop Module initialized. Awaiting input.")

    def joy_callback(self, msg: Joy):
        """
        Callback for raw joystick input (e.g., from JoystickView.js).
        Publishes the Twist message immediately upon receiving joystick input.
        """
        # Assuming msg.axes[0] is for angular (left/right) and msg.axes[1] is for linear (forward/backward)
        # Adjust these indices based on your actual joystick mapping from JoystickView
        normalized_angular = msg.axes[0] # Typically X-axis of left stick
        normalized_linear = msg.axes[1]  # Typically Y-axis of left stick

        self._current_twist.linear.x = normalized_linear * self.max_linear_speed
        self._current_twist.angular.z = normalized_angular * self.max_angular_speed

        # Publish rover velocity immediately
        self.cmd_vel_pub.publish(self._current_twist)
        self.logger.debug(f"Published Joy Twist: Linear={self._current_twist.linear.x:.2f}, Angular={self._current_twist.angular.z:.2f}")

    def drilling_manual_input_callback(self, msg: DrillingManualInput):
        """
        Callback for manual drilling input (from DrillingControlView.js).
        Publishes the DrillingCommand message immediately upon receiving manual input.
        """
        self._current_drilling_command.manual_up = msg.manual_up
        self._current_drilling_command.manual_down = msg.manual_down
        self._current_drilling_command.auger_on = msg.auger_on
        self._current_drilling_command.gate_open = msg.gate_open
        # Note: target_height_cm is not set here, as manual movement handles continuous updates.

        # Publish drilling manual command immediately
        self.drilling_teleop_cmd_pub.publish(self._current_drilling_command)
        self.logger.debug(
            f"Published DrillingCommand: Up={msg.manual_up}, Down={msg.manual_down}, "
            f"Auger={msg.auger_on}, Gate={msg.gate_open}"
        )


if __name__ == '__main__':
    try:
        teleop_module = RoverTeleopModule()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Rover Teleop Module encountered a fatal error: {e}")
