#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

height = 0.0
target_height = 0.0
auger_state = 0.0
gate_state = 0.0
is_collecting = False
last_target_reached_status = False
load_cell_weight = 0.0
last_height_target = 0.0

def platform_cmd_callback(msg):
    """Receives a target height and updates the target variable."""
    global target_height, last_height_target, last_target_reached_status
    target_height = msg.data
    rospy.loginfo(f"Received new target height: {target_height} cm")
    if target_height != last_height_target:
        last_height_target = target_height
        platform_status_pub.publish(Float64(data=0.0))
        last_target_reached_status = False

def auger_cmd_callback(msg):
    """Updates auger state based on command from the FSM."""
    global auger_state
    auger_state = msg.data
    rospy.loginfo(f"Auger motor command received: {auger_state}")

def gate_cmd_callback(msg):
    """Updates gate state based on command from the FSM."""
    global gate_state
    gate_state = msg.data
    rospy.loginfo(f"Servo gate command received: {gate_state}")

def collect_sample_cmd_callback(msg):
    """Starts/stops the load cell simulation."""
    global is_collecting
    if msg.data > 0:
        is_collecting = True
        rospy.loginfo("Received command to START collecting sample.")
    else:
        is_collecting = False
        rospy.loginfo("Received command to STOP collecting sample.")

def low_level_sim():
    global height, last_target_reached_status, load_cell_weight, auger_state
    rospy.init_node('low_level_sim')
    
    # Subscribers for FSM commands
    rospy.Subscriber('/platform_motor/target_height', Float64, platform_cmd_callback)
    rospy.Subscriber('/auger_motor/cmd', Float64, auger_cmd_callback)
    rospy.Subscriber('/servo_gate/cmd', Float64, gate_cmd_callback)
    rospy.Subscriber('/collect_sample_cmd', Float64, collect_sample_cmd_callback)
    
    # Publishers for FSM feedback
    height_pub = rospy.Publisher('/height', Float64, queue_size=10)
    load_cell_pub = rospy.Publisher('/load_cell', Float64, queue_size=10)
    global platform_status_pub
    platform_status_pub = rospy.Publisher('/platform_motor/status', Float64, queue_size=10)

    rate = rospy.Rate(10) # 10 Hz simulation rate
    
    rospy.loginfo("Low-Level Simulation Node Started.")
    
    while not rospy.is_shutdown():
        # --- Platform Movement Logic ---
        if abs(height - target_height) > 0.05:
            if height < target_height:
                height += 0.05
            elif height > target_height:
                height -= 0.05
            
            if last_target_reached_status:
                platform_status_pub.publish(Float64(data=0.0))
                last_target_reached_status = False
        else:
            if not last_target_reached_status:
                platform_status_pub.publish(Float64(data=1.0))
                rospy.loginfo("Platform has reached its target height.")
                last_target_reached_status = True

        # --- Load Cell Logic ---
        # The load cell will only increase if all three conditions are met.
        if is_collecting and auger_state > 0 and height <= -40:
            if load_cell_weight == 0.0:
                rospy.loginfo("All conditions met. Starting load cell increase.")
            load_cell_weight += 2.0
            rospy.loginfo(f"Load cell is increasing. Current weight: {load_cell_weight:.2f} g")
        
        load_cell_pub.publish(Float64(data=load_cell_weight))

        # Publish the new simulated height
        height_pub.publish(Float64(data=height))
        
        rate.sleep()

if __name__ == '__main__':
    try:
        low_level_sim()
    except rospy.ROSInterruptException:
        pass