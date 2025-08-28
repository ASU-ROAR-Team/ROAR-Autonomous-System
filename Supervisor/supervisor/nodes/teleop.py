#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from roar_msgs.msg import DrillingCommand, DrillingManualInput
from sensor_msgs.msg import Joy
import math


class RoverTeleopModule:
    def __init__(self):
        rospy.init_node('rover_teleop_module')

        # --- Rover Motion Publishers ---
        self.all_wheel_vel_pub = rospy.Publisher(
            'motor_control_cmd',
            Float32MultiArray,
            queue_size=10
        )

        # --- Drilling Control Publisher ---
        self.drilling_teleop_cmd_pub = rospy.Publisher('/drilling/command_to_actuators', DrillingCommand, queue_size=10)

        # --- Subscribers for GUI Input ---
        self.joy_sub = rospy.Subscriber('/joystick/raw_input', Joy, self.joy_callback, queue_size=10)
        self.drilling_manual_input_sub = rospy.Subscriber('/drilling/manual_input', DrillingManualInput, self.drilling_manual_input_callback, queue_size=10)
        
        # --- Internal State for Combining Commands ---
        self._current_drilling_command = DrillingCommand()
        self._current_drilling_command.target_height_cm = 0.0
        self._current_drilling_command.gate_open = True
        self._current_drilling_command.auger_on = False
        self._current_drilling_command.manual_up = False
        self._current_drilling_command.manual_down = False

        # --- Parameters for Speed Scaling ---
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 1.0)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 0.5)
        self.robot_width = rospy.get_param('~robot_width', 0.9)
        self.max_motor_rpm = rospy.get_param('~max_motor_rpm', 15.0)

        rospy.loginfo("Rover Teleop Module initialized. Awaiting input.")
        rospy.loginfo(f"Publishing wheel velocities to: {rospy.get_param('~wheel_velocity_topic', '/all_wheel_velocities/command')}")
        rospy.loginfo(f"Max linear speed: {self.max_linear_speed} m/s, Max angular speed: {self.max_angular_speed} rad/s")
        rospy.loginfo(f"Robot width: {self.robot_width} m, Max motor RPM: {self.max_motor_rpm}")

    def joy_callback(self, msg: Joy):
        """
        Callback for raw joystick input (e.g., from JoystickView.js).
        Converts joystick input to Float32MultiArray for 6-wheeled robot.
        """
        rospy.logdebug("Joy callback triggered, processing new data.")
        
        normalized_angular = msg.axes[0]
        normalized_linear = msg.axes[1]

        target_linear_vel = normalized_linear * self.max_linear_speed
        target_angular_vel = normalized_angular * self.max_angular_speed

        v_right = target_linear_vel + (target_angular_vel * self.robot_width / 2.0)
        v_left = target_linear_vel - (target_angular_vel * self.robot_width / 2.0)

        rpm_scale_factor = self.max_motor_rpm / self.max_linear_speed
        
        rpm_right = max(-self.max_motor_rpm, min(self.max_motor_rpm, v_right * rpm_scale_factor))
        rpm_left = max(-self.max_motor_rpm, min(self.max_motor_rpm, v_left * rpm_scale_factor))

        wheel_velocities_msg = Float32MultiArray()
        wheel_velocities_msg.data = [
            float(rpm_right),
            float(rpm_right),
            float(rpm_right),
            float(rpm_left),
            float(rpm_left),
            float(rpm_left)
        ]

        self.all_wheel_vel_pub.publish(wheel_velocities_msg)
        rospy.logdebug(
            f"Published Wheel RPMs: Right={rpm_right:.2f}, Left={rpm_left:.2f}"
        )

    def drilling_manual_input_callback(self, msg: DrillingManualInput):
        """
        Callback for manual drilling input (from DrillingControlView.js).
        Publishes the DrillingCommand message immediately upon receiving manual input.
        """
        self._current_drilling_command.manual_up = msg.manual_up
        self._current_drilling_command.manual_down = msg.manual_down
        self._current_drilling_command.auger_on = msg.auger_on
        self._current_drilling_command.gate_open = msg.gate_open

        self.drilling_teleop_cmd_pub.publish(self._current_drilling_command)
        rospy.logdebug(
            f"Published DrillingCommand: Up={msg.manual_up}, Down={msg.manual_down}, "
            f"Auger={msg.auger_on}, Gate={msg.gate_open}"
        )

if __name__ == '__main__':
    try:
        teleop_module = RoverTeleopModule()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down Rover Teleop Module.")
        pass
    except Exception as e:
        rospy.logerr(f"Rover Teleop Module encountered a fatal error: {e}")