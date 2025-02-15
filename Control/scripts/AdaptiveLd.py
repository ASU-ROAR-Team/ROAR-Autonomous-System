#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates, LinkStates
from tf.transformations import euler_from_quaternion
import time
import numpy as np
import matplotlib.pyplot as plt
from turtlebot3_msgs.msg import wp_list
import pandas as pd

class Control:

    def __init__(self):

        rospy.init_node('controller', anonymous=True)
        self.velocityLFPublisher = rospy.Publisher('/wheel_lhs_front_velocity_controller/command', Float64, queue_size=10)
        self.velocityRFPublisher = rospy.Publisher('/wheel_rhs_front_velocity_controller/command', Float64, queue_size=10)
        self.velocityLRPublisher = rospy.Publisher('/wheel_lhs_rear_velocity_controller/command', Float64, queue_size=10)
        self.velocityRRPublisher = rospy.Publisher('/wheel_rhs_rear_velocity_controller/command', Float64, queue_size=10)
        self.velocityLMPublisher = rospy.Publisher('/wheel_lhs_mid_velocity_controller/command', Float64, queue_size=10)
        self.velocityRMPublisher = rospy.Publisher('/wheel_rhs_mid_velocity_controller/command', Float64, queue_size=10)
        self.poseSubscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.updatePose)

        self.pathSubscriber = rospy.Subscriber('tuple_list_topic', wp_list, self.tupleListCallback)  

        self.pose = ModelStates()

        self.rate = rospy.Rate(10)
        self.kp = 0.5
        self.ki = 0.5
        self.kd = 0.0
        self.distLD = 0.3
        self.indexLD=0

        self.dt = 0.1
        self.cuurentX = 0.0
        self.cuurentY = 0.0
        self.integral = 0.0
        self.maxVelocity = 1.57

        self.robotTheta = 0.0
        self.width = 0.8
        self.time_values = []
        self.error_values = []

        self.waypoints = []
        self.x_goal_point = 0.0
        self.y_goal_point = 0.0
        self.waypoints_plot = None

        # Setup matplotlib for plotting
        self.fig, self.ax1 = plt.subplots(1, 1, figsize=(6, 6))  # Single plot with one axis
        # Plot for waypoints
        self.ax1.set_xlabel('X')
        self.ax1.set_ylabel('Y')
        self.ax1.set_title('Waypoints and Robot Path')
        self.ax1.set_xlim(-5, 4)  # Set x-axis limits from -10 to 10
        self.ax1.set_ylim(-1, 6.5)  # Set y-axis limits from -10 to 10
        self.waypoints_x = []
        self.waypoints_y = []
        self.waypoints_plot, = self.ax1.plot([], [], 'b--', label='Waypoints')
        self.robot_position_plot, = self.ax1.plot([], [], 'r^', label='Robot Position', markersize=6)
        self.lookahead_point_plot, = self.ax1.plot([], [], 'go', label='Look ahead point', markersize=3)
        # Add a plot for the robot's path

        ##for debugging
        self.past_positions_x = []
        self.past_positions_y = []
        self.past_vl=[]
        self.past_vr=[]
        self.pastLD=[]
        self.past_curvature=[]
        self.past_headings=[]
        self.robot_path_plot, = self.ax1.plot([], [], 'r-', label='Robot Path')  # Robot path plot
        self.ax1.legend()

        plt.tight_layout()

    def tupleListCallback(self, msg):
        self.waypoints = [(msg.a[i], msg.b[i]) for i in range(msg.length)]
        self.x_goal_point = msg.a[0]
        self.y_goal_point = msg.b[0]
        self.waypoints.reverse()
        self.update_waypoints_plot()

    def updatePose(self, data:ModelStates):
        self.pose = data
        self.cuurentX = self.pose.pose[1].position.x 
        self.cuurentY = self.pose.pose[1].position.y
        orientation = self.pose.pose[1].orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robotTheta = yaw
    
    def map_velocity(self,velocity):
        # Clamping the value to be within the range -1.57 to 1.57
        velocity = min(max(velocity, -1.57), 1.57)
        
        if velocity < 0:
            # Mapping negative values from -1.57 to 0 to the range 0 to 61
            return int(((velocity + 1.57) / 1.57) * 61)
        else:
            # Mapping positive values from 0 to 1.57 to the range 67 to 127
            return int((velocity / 1.57) * 60 + 67)
    def set_velocity(self,k): 
        velocity=1.57/(1+abs(k))
        goal_point = self.waypoints[-1]
        distance_to_goal = np.linalg.norm(np.array((self.cuurentX, self.cuurentY)) - np.array(goal_point))
        if distance_to_goal<0.05:
            return 0
        return velocity
    
    def find_lookahead_point(self, robot_position): 


        for i, waypoint in enumerate(self.waypoints[self.indexLD:],self.indexLD): 
                print("new waypoint")
                distance_to_robot = np.linalg.norm(np.array(waypoint) - np.array(robot_position))
                print("waypoint = "+str(waypoint[0])+ ", "+str(waypoint[1]))
                print("dist="+str(distance_to_robot))

                if distance_to_robot < self.distLD: 
                        self.indexLD=i
                        lookahead_point= waypoint

                else:
                    lookahead_point=self.waypoints[self.indexLD]
                    break
        self.plot_lookahead_point(lookahead_point[0],lookahead_point[1])
        return lookahead_point

    def purePursuit(self):
        lookahead_point = self.find_lookahead_point((self.cuurentX, self.cuurentY))

        if lookahead_point is not None:
            alpha = math.atan2((lookahead_point[1] - self.cuurentY), (lookahead_point[0] - self.cuurentX))
            L = math.hypot(lookahead_point[0] - self.cuurentX, lookahead_point[1] - self.cuurentY)
            theta = alpha - self.robotTheta
            dx = L * math.cos(theta)
            k=dx/L**2
            self.past_curvature.append(k)
            vc=self.set_velocity(k) 
            print("vc="+str(vc))
            self.distLD=0.25*vc+0.05
            self.pastLD.append(self.distLD)
            self.past_headings.append(self.robotTheta)
            Vr = vc* (1 - self.width * dx / (L * L))
            Vl = vc * (1 + self.width * dx / (L * L))

            Vr = min(max(Vr, -1.57), 1.57)
            Vl = min(max(Vl, -1.57), 1.57)

            self.past_vl.append(Vl)
            self.past_vr.append(Vr)

            Vr_mapped = self.map_velocity(Vr)
            Vl_mapped = self.map_velocity(Vl)
            print('Right: ', Vr, ' Mapped Right:', Vr_mapped, ' Left: ', Vl, ' Mapped Left:', Vl_mapped)


            self.velocityLMPublisher.publish(Vl)
            self.velocityrmPublisher.publish(Vr)
            self.velocityLFPublisher.publish(Vl)
            self.velocityRFPublisher.publish(Vr)
            self.velocityLRPublisher.publish(Vl)
            self.velocityRRPublisher.publish(Vr)

            self.plot_rover_position()
            # Give time for plot to update
            plt.pause(0.001)
        else:
             print("fy 7aga 8lt")

    def plot_rover_position(self):
        # Append the current position to the past positions
        self.past_positions_x.append(self.cuurentX)
        self.past_positions_y.append(self.cuurentY)

        # Update the robot path plot
        self.robot_path_plot.set_data(self.past_positions_x, self.past_positions_y)
        self.robot_position_plot.set_data([self.cuurentX], [self.cuurentY])  # Update the current position plot

    def plot_lookahead_point(self,lookahead_x,lookahead_y): ##added (edited)
        self.lookahead_point_plot.set_data([lookahead_x], [lookahead_y])  # Update the current position 

    def update_waypoints_plot(self):
            if self.waypoints:
                self.waypoints_x, self.waypoints_y = zip(*self.waypoints)
            else:
                self.waypoints_x, self.waypoints_y = [], []

            if self.waypoints_plot is not None:
                self.waypoints_plot.set_data(self.waypoints_x, self.waypoints_y)
    def shutdown_Seq(self): ##added (edited)
        print("saving data")
        df=pd.DataFrame({"Vr":self.past_vr,"Vl":self.past_vl,"Positions_x":self.past_positions_x,"Positions_y":self.past_positions_y,"Lookahead distance":self.pastLD,"curvature":self.past_curvature,"heading":self.past_headings})
        df2=pd.DataFrame({"waypoint_x":self.waypoints_x,"waypoint_y":self.waypoints_y})
        df.to_csv("Roar_adaptive_pure_pursuit_trial_4.csv")
        df2.to_csv("waypoints_3.csv")

if __name__ == '__main__':
    try:
        x = Control()
        rospy.on_shutdown(x.shutdown_Seq)
        while not rospy.is_shutdown():
            if len(x.waypoints) > 0:
                x.purePursuit()
    except rospy.ROSInterruptException:
        pass