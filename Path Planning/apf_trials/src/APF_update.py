#!/usr/bin/env python3

#########################################################################################################
#Import the required libraries:
import rospy
from geometry_msgs.msg import Twist,Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import numpy as np
import math
import sympy as sym
from sympy import *
import json
import matplotlib.pyplot as plt

#######################################################################
#Initialize ROS Node
rospy.init_node('Path_Planning_APF', anonymous=True) #Identify ROS Node
#######################################################################
#######################################################################
#ROS Publisher Code for Velocity
pub1 = rospy.Publisher('/APF_Des_Pos', Path, queue_size=10) #Identify the publisher "pub1" to publish on topic "/APF_Des_Pos" to send message of type "Pose"
path = Path() #Identify msg variable of data type Path
i=0
for i in range(10):
   pose=PoseStamped()
   pose.pose.position.x = position[0]
   pose.pose.position.y = position[1]
   path.poses.append(pose)
rate = rospy.Rate(10) # rate of publishing msg 10hz
#######################################################################
#######################################################################
def euler_to_quaternion(yaw, pitch, roll):
    x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [x, y, z, w]
#######################################################################

#######################################################################
def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]
#######################################################################
#######################################################################

#ROS Subscriber Code for Position
flag_cont = 0	#Initialize flag by zero
pos_msg = Pose()	#Identify msg variable of data type Pose
position = np.zeros((1,6))
Velocity_msg = Twist()
velocity = np.zeros((1,6))

#######################################################################
#######################################################################


##Callback function for feedback the vehicle current position
#Callback function which is called when a new message of type Pose is received by the subscriber
def callback(data):
  global pos_msg	#Identify msg variable created as global variable
  global sub2		#Identify a subscriber as global variable
  global flag_cont
  global position 
  global Velocity_msg
  global velocity

  msg = data
  pos_msg.position.x = round(msg.pose.pose.position.x, 4)		#Round the value of x to 4 decimal places
  pos_msg.position.y = round(msg.pose.pose.position.y, 4)		#Round the value of y to 4 decimal places
  pos_msg.position.z = round(msg.pose.pose.position.z, 4)		#Round the value of y to 4 decimal places
  pos_msg.orientation.x = round(msg.pose.pose.orientation.x, 4)	#Round the value of theta to 4 decimal places
  pos_msg.orientation.y = round(msg.pose.pose.orientation.y, 4)	#Round the value of theta to 4 decimal places
  pos_msg.orientation.z = round(msg.pose.pose.orientation.z, 4)	#Round the value of theta to 4 decimal places
  pos_msg.orientation.w = round(msg.pose.pose.orientation.w, 4)	#Round the value of theta to 4 decimal places
  [yaw, pitch, roll] = quaternion_to_euler(pos_msg.orientation.x, pos_msg.orientation.y, pos_msg.orientation.z, pos_msg.orientation.w)
  position = [pos_msg.position.x,pos_msg.position.y,pos_msg.position.z,yaw, pitch, roll]
  Velocity_msg.linear.x = round(msg.twist.twist.linear.x, 4)
  Velocity_msg.linear.y = round(msg.twist.twist.linear.y, 4)
  Velocity_msg.linear.z = round(msg.twist.twist.linear.z, 4)
  Velocity_msg.angular.x = round(msg.twist.twist.angular.x, 4)
  Velocity_msg.angular.y = round(msg.twist.twist.angular.y, 4)
  Velocity_msg.angular.z = round(msg.twist.twist.angular.z, 4)
  velocity = [Velocity_msg.linear.x,Velocity_msg.linear.y,Velocity_msg.linear.z,Velocity_msg.angular.x,Velocity_msg.angular.y,Velocity_msg.angular.z]
  flag_cont = 1

sub2 = rospy.Subscriber('/odom', Odometry, callback) #Identify the subscriber "sub2" to subscribe topic "/odom" of type "Odometry"

#######################################################################
#######################################################################
# ROS Subscriber Code for Map
Obs_Pos_listt = []  # List to store obstacle positions
#######################################################################
#######################################################################
while flag_cont == 0:
  pass
#######################################################################
#######################################################################
Rob_pos = [position[0],position[1],position[3]]
Roc_vel = [velocity[0],velocity[5]]

x_p = Rob_pos[0]
y_p = Rob_pos[1]
vel_p_x = Roc_vel[0]*cos(Rob_pos[2])
vel_p_y = Roc_vel[0]*sin(Rob_pos[2])

#######################################################################
#######################################################################

#APF Inputs
Goal_Pos = [rospy.get_param("~x_Goal"),rospy.get_param("~y_Goal")]
Obs_Pos = [rospy.get_param("~x_Obs"),rospy.get_param("~y_Obs")]
APF_Param = [rospy.get_param("~K_att"),rospy.get_param("~K_rep"),rospy.get_param("~q_star")] #[K_att,K_rep,q_star]

#######################################################################
#######################################################################

#APF Equations
x_rob = symbols('x_rob')
y_rob = symbols('y_rob')
x_goal = symbols('x_goal')
y_goal = symbols('y_goal')
x_obs = symbols('x_obs')
y_obs = symbols('y_obs')

#Attraction Forces Equations
Fx_att = -APF_Param[0]*(x_rob-x_goal)
Fy_att = -APF_Param[0]*(y_rob-y_goal)
#Repulsion Forces Equations
d_obs = sqrt((x_rob-x_obs)**2+(y_rob-y_obs)**2)
Fx_rep = -APF_Param[1]*((1/d_obs)-(1/APF_Param[2]))*(-(x_rob-x_obs)/(d_obs**3))
Fy_rep = -APF_Param[1]*((1/d_obs)-(1/APF_Param[2]))*(-(y_rob-y_obs)/(d_obs**3))

#######################################################################
#######################################################################
def APF_Fn(Rob_pos,Goal_pos,Obs_pos,APF_Param):
   global Fx_att
   global Fy_att
   global d_obs
   global Fx_rep
   global Fy_rep
  
   Fx_att_val = Fx_att.subs([(x_rob,Rob_pos[0]),(x_goal,Goal_pos[0])])
   Fy_att_val = Fy_att.subs([(y_rob,Rob_pos[1]),(y_goal,Goal_pos[1])])
   d_obs_val = d_obs.subs([(x_rob,Rob_pos[0]),(y_rob,Rob_pos[1]),(x_obs,Obs_pos[0]),(y_obs,Obs_pos[1])])
   if d_obs_val<APF_Param[2]:
     Fx_rep_val = Fx_rep.subs([(x_rob,Rob_pos[0]),(y_rob,Rob_pos[1]),(x_obs,Obs_pos[0]),(y_obs,Obs_pos[1]),(d_obs,d_obs_val)])
     Fy_rep_val = Fy_rep.subs([(x_rob,Rob_pos[0]),(y_rob,Rob_pos[1]),(x_obs,Obs_pos[0]),(y_obs,Obs_pos[1]),(d_obs,d_obs_val)])
   else:
     Fx_rep_val = 0
     Fy_rep_val = 0 

   Fx_net_val = Fx_att_val + Fx_rep_val
   Fy_net_val = Fy_att_val + Fy_rep_val
   F_xy_net = [Fx_net_val,Fy_net_val]
   return F_xy_net

#########################################################################################################

#Simulation While Loop
tau = rospy.get_param("~tau") #Sampling Time
rob_mass = rospy.get_param("~rob_mass") #Robot Mass (Turtlebot 3 Waffle_pi)
trajectory_x = []
trajectory_y = []
while not rospy.is_shutdown():
   if flag_cont == 1:
       Rob_pos = [position[0], position[1], position[3]]
       Rob_vel = [velocity[0], velocity[5]]

       F_xy_net = APF_Fn(Rob_pos, Goal_Pos, Obs_Pos, APF_Param)
       F_net = float(sqrt(F_xy_net[0] ** 2 + F_xy_net[1] ** 2))
       F_net_direct = float(atan2(F_xy_net[1], F_xy_net[0]))

       vel_c_x = vel_p_x + (F_xy_net[0] / rob_mass) * tau
       vel_c_y = vel_p_y + (F_xy_net[1] / rob_mass) * tau
       x_des = x_p + vel_c_x * tau
       y_des = y_p + vel_c_y * tau
       theta_des = F_net_direct
       Rob_pos_des = [x_des, y_des, theta_des]

       vel_p_x = Rob_vel[0] * cos(Rob_pos[2])
       vel_p_y = Rob_vel[0] * sin(Rob_pos[2])
       x_p = Rob_pos[0]
       y_p = Rob_pos[1]
   else:
       Rob_pos_des = Rob_pos

   Des_Pos_msg.position.x = Rob_pos_des[0]
   Des_Pos_msg.position.y = Rob_pos_des[1]
   Des_Pos_msg.position.z = 0
   [qx_des, qy_des, qz_des, qw_des] = euler_to_quaternion(Rob_pos_des[2], 0, 0)
   Des_Pos_msg.orientation.x = qx_des
   Des_Pos_msg.orientation.y = qy_des
   Des_Pos_msg.orientation.z = qz_des
   Des_Pos_msg.orientation.w = qw_des
   

    # Store the robot's path
   trajectory_x.append(Rob_pos_des[0])
   trajectory_y.append(Rob_pos_des[1])
   

   pub1.publish(Des_Pos_msg)
   rate.sleep()
    #Check if goal is reached
   distance_to_goal = sqrt((Rob_pos[0] - Goal_Pos[0])**2 + (Rob_pos[1] - Goal_Pos[1])**2)
   if distance_to_goal < 0.1:
        rospy.loginfo("Goal reached!")
        break 


# Plot the path
fig, ax = plt.subplots()
plt.plot(trajectory_x, trajectory_y, label="Desired Path", linestyle="--")# desired path
plt.scatter(Goal_Pos[0], Goal_Pos[1], color='green', label="Goal", s=100)  # Goal point
obstacle = plt.Rectangle((1.85, 1.85), 1.15, 1.15, color="red", alpha=0.5, label="Obstacle") #obstacle as a rectangle
ax.add_patch(obstacle)
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.title("APF path")
plt.legend()
plt.grid()
plt.show()






