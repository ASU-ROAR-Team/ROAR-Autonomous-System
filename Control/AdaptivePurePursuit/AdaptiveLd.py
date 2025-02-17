#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry,Path
from gazebo_msgs.msg import ModelStates, LinkStates
from tf.transformations import euler_from_quaternion
import time
import numpy as np
import matplotlib.pyplot as plt
##from turtlebot3_msgs.msg import wp_list
import pandas as pd

class Control:
    "This class containts functions to control the rover's motion using adaptive pure pursuit algorithm"

    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        self.velocityLFPublisher = rospy.Publisher('/wheel_lhs_front_velocity_controller/command', Float64, queue_size=10)
        self.velocityRFPublisher = rospy.Publisher('/wheel_rhs_front_velocity_controller/command', Float64, queue_size=10)
        self.velocityLRPublisher = rospy.Publisher('/wheel_lhs_rear_velocity_controller/command', Float64, queue_size=10)
        self.velocityRRPublisher = rospy.Publisher('/wheel_rhs_rear_velocity_controller/command', Float64, queue_size=10)
        self.velocityLMPublisher = rospy.Publisher('/wheel_lhs_mid_velocity_controller/command', Float64, queue_size=10)
        self.velocityRMPublisher = rospy.Publisher('/wheel_rhs_mid_velocity_controller/command', Float64, queue_size=10)
        self.poseSubscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.updatePose)

        self.pathSubscriber = rospy.Subscriber('/Path', Path, self.pathCallback)  

        self.pose = ModelStates()

        self.rate = rospy.Rate(10)
        self.kp = 0.5
        self.ki = 0.5
        self.kd = 0.0
        self.distLD = 0.3
        self.indexLD = 0

        self.dt = 0.1
        self.currentX = 0.0
        self.currentY = 0.0
        self.integral = 0.0
        self.maxVelocity = 1.57

        self.robotTheta = 0.0
        self.width = 0.8


        self.waypoints = []
        self.xGoalPoint = 0.0
        self.yGoalPoint = 0.0
        self.waypointsPlot = None

        # Setup matplotlib for plotting
        self.fig, self.ax1 = plt.subplots(1, 1, figsize=(6, 6))  # Single plot with one axis
        # Plot for waypoints
        self.ax1.set_xlabel('X')
        self.ax1.set_ylabel('Y')
        self.ax1.set_title('Waypoints and Robot Path')
        self.ax1.set_xlim(-5, 4)  # Set x-axis limits from -10 to 10
        self.ax1.set_ylim(-1, 6.5)  # Set y-axis limits from -10 to 10
        self.waypointsX = []
        self.waypointsY = []
        self.waypointsPlot, = self.ax1.plot([], [], 'b--', label='Waypoints')
        self.robotPositionPlot, = self.ax1.plot([], [], 'r^', label='Robot Position', markersize=6)
        self.lookaheadPointPlot, = self.ax1.plot([], [], 'go', label='Look ahead point', markersize=3)
        # Add a plot for the robot's path

        ##for debugging
        self.pastPositionsX = []
        self.pastPositionsY = []
        self.pastVL=[]
        self.pastVR=[]
        self.pastLD=[]
        self.pastCurvature=[]
        self.pastHeadings=[]
        self.robotPathPlot, = self.ax1.plot([], [], 'r-', label='Robot Path')  # Robot path plot
        self.ax1.legend()

        plt.tight_layout()

    # def tupleListCallback(self, msg):
    #     self.waypoints = [(msg.a[i], msg.b[i]) for i in range(msg.length)]
    #     self.xGoalPoint = msg.a[0]
    #     self.yGoalPoint = msg.b[0]
    #     self.waypoints.reverse()
    #     self.updateWaypointsPlot()


    def pathCallback(self, msg:Path):
        """
        This is the callback function for the path
        
        Parameters:
        -----
        msg: this contains the path that the controller tracks 
        ----
        Returns:
        ----
        None
        ----
        """
        self.waypoints=[(pose.pose.position.x,pose.pose.position.y) for pose in msg.poses]
        self.updateWaypointsPlot()
        rospy.loginfo("got waypoints")

    def updatePose(self, data:ModelStates):
        """
        This is the callback function for the rover's position
        
        Parameters:
        -----
        data: this contains the rover's position
        ----
        Returns:
        ----
        None
        ----
        """

        self.pose = data
        self.currentX = self.pose.pose[1].position.x 
        self.currentY = self.pose.pose[1].position.y
        orientation = self.pose.pose[1].orientation
        orientationList = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientationList)
        self.robotTheta = yaw
    
    def mapVelocity(self,velocity:float):
        """
        This function maps the velocity from m/s to the values accepted by the simulator
        
        Parameters:
        -----
        velocity: the velocity in m/s 
        ----
        Returns:
        ----
        the velocity in the range accepted by the simulator
        ----
        """
        # Clamping the value to be within the range -1.57 to 1.57
        velocity = min(max(velocity, -1.57), 1.57)
        
        if velocity < 0:
            # Mapping negative values from -1.57 to 0 to the range 0 to 61
            return int(((velocity + 1.57) / 1.57) * 61)
        else:
            # Mapping positive values from 0 to 1.57 to the range 67 to 127
            return int((velocity / 1.57) * 60 + 67)
    def setVelocity(self,k:float):
        """
        This function calculates the suitable velocity from the curvature of the path (the lower the curvature the higher the speed)
        
        Parameters:
        -----
        k: the curvature of the path in 1/m
        ----
        Returns:
        ----
        velocity : the suitable velocity
        ----
        """
        velocity=1.57/(1+abs(k))
        goalPoint = self.waypoints[-1]
        distanceToGoal = np.linalg.norm(np.array((self.currentX, self.currentY)) - np.array(goalPoint))
        if distanceToGoal<0.05:
            return 0
        return velocity
    
    def findLookaheadPoint(self, robotPosition:tuple): 
        """
        This function selects the lookahead point from the path
        
        Parameters:
        -----
        robotPosition: a list-like structure containing the position of the robot in the form (x,y)
        ----
        Returns:
        ----
        looaheadPoint: a tuple containing the lookahead co-ordinates in the form (x,y)
        ----
        """
        for i, waypoint in enumerate(self.waypoints): 
                print("new waypoint")
                distanceToRobot = np.linalg.norm(np.array(waypoint) - np.array(robotPosition))
                print("waypoint = "+str(waypoint[0])+ ", "+str(waypoint[1]))
                print("dist="+str(distanceToRobot))

                if distanceToRobot < self.distLD: 
                        self.indexLD=i
                        lookaheadPoint= waypoint

                else:
                    lookaheadPoint=self.waypoints[self.indexLD]
                    break
        self.plotLookaheadPoint(lookaheadPoint[0],lookaheadPoint[1])
        return lookaheadPoint

    def purePursuit(self):
        """
        This function calculates and publishes the steering angle from the lookahead point using the pure pursuit equation
        
        Parameters:
        -----
        None
        ----
        Returns:
        ----
        None
        ----
        """
        lookaheadPoint = self.findLookaheadPoint((self.currentX, self.currentY))

        if lookaheadPoint is not None:
            alpha = math.atan2((lookaheadPoint[1] - self.currentY), (lookaheadPoint[0] - self.currentX))
            L = math.hypot(lookaheadPoint[0] - self.currentX, lookaheadPoint[1] - self.currentY)
            theta = alpha - self.robotTheta
            dx = L * math.cos(theta)
            k=dx/L**2
            self.pastCurvature.append(k)
            vc=self.setVelocity(k) 
            print("vc="+str(vc))
            self.distLD=0.25*vc+0.05
            self.pastLD.append(self.distLD)
            self.pastHeadings.append(self.robotTheta)
            Vr = vc* (1 - self.width * dx / (L * L))
            Vl = vc * (1 + self.width * dx / (L * L))

            Vr = min(max(Vr, -1.57), 1.57)
            Vl = min(max(Vl, -1.57), 1.57)

            self.pastVL.append(Vl)
            self.pastVR.append(Vr)

            VrMapped = self.mapVelocity(Vr)
            VlMapped = self.mapVelocity(Vl)
            print('Right: ', Vr, ' Mapped Right:', VrMapped, ' Left: ', Vl, ' Mapped Left:', VlMapped)


            self.velocityLMPublisher.publish(Vl)
            self.velocityRMPublisher.publish(Vr)
            self.velocityLFPublisher.publish(Vl)
            self.velocityRFPublisher.publish(Vr)
            self.velocityLRPublisher.publish(Vl)
            self.velocityRRPublisher.publish(Vr)

            self.plotRoverPosition()
            # Give time for plot to update
            plt.pause(0.001)

    def plotRoverPosition(self):
        """
        This function plots the rover position using matplotlib
        
        Parameters:
        -----
        None
        ----
        Returns:
        ----
        None
        ----
        """
        # Append the current position to the past positions
        self.pastPositionsX.append(self.currentX)
        self.pastPositionsY.append(self.currentY)

        # Update the robot path plot
        self.robotPathPlot.set_data(self.pastPositionsX, self.pastPositionsY)
        self.robotPositionPlot.set_data([self.currentX], [self.currentY])  # Update the current position plot

    def plotLookaheadPoint(self,lookaheadX,lookaheadY): ##added (edited)
        """
        This function plots the lookahead point position using matplotlib
        
        Parameters:
        -----
        None
        ----
        Returns:
        ----
        None
        ----
        """

        self.lookaheadPointPlot.set_data([lookaheadX], [lookaheadY])  # Update the current position 

    def updateWaypointsPlot(self):
        """
        This function plots the rover path using matplotlib
        
        Parameters:
        -----
        None
        ----
        Returns:
        ----
        None
        ----
        """
        if self.waypoints:
            self.waypointsX, self.waypointsY = zip(*self.waypoints)
        else:
            self.waypointsX, self.waypointsY = [], []

        if self.waypointsPlot is not None:
            self.waypointsPlot.set_data(self.waypointsX, self.waypointsY)
    def shutdownSeq(self): ##added (edited)
        """
        This function saves the rover's data in a csv for further analysis
        
        Parameters:
        -----
        None
        ----
        Returns:
        ----
        None
        ----
        """
        print("saving data")
        df=pd.DataFrame({"Vr":self.pastVR,"Vl":self.pastVL,"Positions_x":self.pastPositionsX,"Positions_y":self.pastPositionsY,"Lookahead distance":self.pastLD,"curvature":self.pastCurvature,"heading":self.pastHeadings})
        df2=pd.DataFrame({"waypoint_x":self.waypointsX,"waypoint_y":self.waypointsY})
        df.to_csv("Roar_adaptive_pure_pursuit_trial_4.csv")
        df2.to_csv("waypoints_3.csv")

if __name__ == '__main__':
    try:
        x = Control()
        rospy.on_shutdown(x.shutdownSeq)
        while not rospy.is_shutdown():
            if len(x.waypoints) > 0:
                x.purePursuit()
    except rospy.ROSInterruptException:
        pass