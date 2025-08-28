#!/usr/bin/env python3
import rospy
import time
from roar_msgs.msg import DrillingCommand as DrillingCommandMsg
from roar_msgs.msg import DrillingStatus as DrillingStatusMsg

class DrillingFeedbackSimulator:
    def __init__(self):
        rospy.init_node('drilling_feedback_simulator', anonymous=True)
        self.pub = rospy.Publisher('/drilling/feedback', DrillingStatusMsg, queue_size=10)
        self.rate = rospy.Rate(10) # 10 Hz simulation rate

        # Internal simulated states
        self.current_sim_height = 0.0
        self.current_sim_weight = 0.0

        # Store last received command
        self.last_command = DrillingCommandMsg()
        self.last_command.target_height_cm = 0.0
        self.last_command.gate_open = True
        self.last_command.auger_on = False

        # Parameters for simulation physics (you can tune these)
        self.PLATFORM_SPEED = 5.0 # cm/s
        self.AUGER_COLLECTION_RATE = 20.0 # g/s
        self.HEIGHT_TOLERANCE = 0.5 # cm for reaching target height

        # Subscribe to the drilling command topic
        rospy.Subscriber('/drilling/command_to_actuators', DrillingCommandMsg, self.command_callback, queue_size=10)

        rospy.loginfo("Drilling Feedback Simulator initialized.")
        rospy.loginfo("Subscribing to /drilling/command_to_actuators and publishing to /drilling/feedback")

    def command_callback(self, msg):
        """Callback to store the latest command received from the DrillingModule."""
        self.last_command = msg
        rospy.logdebug(f"Simulator received command: target_height={msg.target_height_cm:.2f}, auger_on={msg.auger_on}, gate_open={msg.gate_open}")

    def run(self):
        """Main loop for the simulator."""
        while not rospy.is_shutdown():
            # Calculate time elapsed since last loop iteration
            dt = self.rate.sleep_dur.to_sec()

            # Simulate platform height movement
            target_height = self.last_command.target_height_cm
            
            # Move towards target height
            if abs(self.current_sim_height - target_height) > self.HEIGHT_TOLERANCE:
                if self.current_sim_height < target_height:
                    self.current_sim_height += self.PLATFORM_SPEED * dt
                    if self.current_sim_height > target_height:
                        self.current_sim_height = target_height
                else: # current_sim_height > target_height
                    self.current_sim_height -= self.PLATFORM_SPEED * dt
                    if self.current_sim_height < target_height:
                        self.current_sim_height = target_height
            else:
                self.current_sim_height = target_height # Snap to target if within tolerance

            # Simulate auger collection
            if self.last_command.auger_on and not self.last_command.gate_open:
                # Only collect if auger is on AND gate is closed (to prevent sample loss)
                self.current_sim_weight += self.AUGER_COLLECTION_RATE * dt
                # Cap weight at a reasonable maximum if needed, e.g., 200g
                if self.current_sim_weight > 200.0:
                    self.current_sim_weight = 200.0
            elif self.last_command.gate_open and self.current_sim_height < self.HEIGHT_TOLERANCE * 2:
                # If gate is open and platform is near surface, simulate dispensing (weight reset)
                self.current_sim_weight = 0.0 # Reset weight after dispensing

            # Publish the simulated status
            msg = DrillingStatusMsg()
            msg.current_height = self.current_sim_height
            msg.current_weight = self.current_sim_weight
            self.pub.publish(msg)
            
            rospy.logdebug(f"Simulator publishing: height={self.current_sim_height:.2f}, weight={self.current_sim_weight:.2f}")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        simulator = DrillingFeedbackSimulator()
        simulator.run()
    except rospy.ROSInterruptException:
        pass