#!/usr/bin/env python3

# pylint: disable=all
# mypy: ignore-errors


"""This module handles the high level control of the rover
using the adaptive pure pursuit algorithm"""

import math
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
import numpy as np
import matplotlib.pyplot as plt

import pandas as pd


class Control:
    """This class containts functions to control the rover's motion using
    adaptive pure pursuit algorithm"""

    def __init__(self):
        rospy.init_node("controller", anonymous=True)
        self.velocityPublisher = {
            "LF": rospy.Publisher(
                rospy.get_param("publishing_topics/left_front_wheel"), Float64, queue_size=10
            ),
            "RF": rospy.Publisher(
                rospy.get_param("publishing_topics/right_front_wheel"), Float64, queue_size=10
            ),
            "LR": rospy.Publisher(
                rospy.get_param("publishing_topics/left_rear_wheel"), Float64, queue_size=10
            ),
            "RR": rospy.Publisher(
                rospy.get_param("publishing_topics/right_rear_wheel"), Float64, queue_size=10
            ),
            "LM": rospy.Publisher(
                rospy.get_param("publishing_topics/left_mid_wheel"), Float64, queue_size=10
            ),
            "RM": rospy.Publisher(
                rospy.get_param("publishing_topics/right_mid_wheel"), Float64, queue_size=10
            ),
        }
        # Publishers
        self.subscribers = {
            "Pose": rospy.Subscriber(
                rospy.get_param("subscribing_topics/pose"), Odometry, self.updatePose
            ),
            "Path": rospy.Subscriber(
                rospy.get_param("subscribing_topics/path"), Path, self.pathCallback
            ),
        }
        # Subscribers

        self.indexLD = 0
        self.currentPosition = [0, 0, 0]  ##[x,y,theta]
        self.distLd = 0.3
        self.MAXVELOCITY = rospy.get_param("robot_parameter/maxVelocity", 1.57)
        self.WIDTH = rospy.get_param("robot_parameters/width", 0.8)
        self.localPath = rospy.get_param("algorithm_parameters/localPath", False)
        self.visualize = rospy.get_param("meta_parameters/visualize")
        self.log = rospy.get_param("meta_parameters/log", False)
        self.KL = rospy.get_param("algorithm_parameters/KL", 0.25)
        self.KC = rospy.get_param("algorithm_parameters/KC", 0.05)

        self.waypoints = []
        if self.visualize:
            # Setup matplotlib for plotting
            _, self.ax1 = plt.subplots(1, 1, figsize=(6, 6))  # Single plot with one axis
            # Plot for waypoints
            self.ax1.set_xlabel("X")
            self.ax1.set_ylabel("Y")
            self.ax1.set_title("Waypoints and Robot Path")
            self.ax1.set_xlim(-5, 4)  # Set x-axis limits from -10 to 10
            self.ax1.set_ylim(-1, 6.5)  # Set y-axis limits from -10 to 10
            self.plots = {
                "waypointsPlot": self.ax1.plot([], [], "b--", label="Waypoints"),
                "robotPositionPlot": self.ax1.plot(
                    [], [], "r^", label="Robot Position", markersize=6
                ),
                "lookaheadPointPlot": self.ax1.plot(
                    [], [], "go", label="Look ahead point", markersize=3
                ),
                "robotPathPlot": self.ax1.plot([], [], "r-", label="Robot Path"),
            }
            # Add a plot for the robot's path

            ##for debugging
            self.debuggingLists = {
                "pastPositionsX": [],
                "pastPositionsY": [],
                "pastVL": [],
                "pastVR": [],
                "pastLD": [],
                "pastCurvature": [],
                "pastHeadings": [],
            }
            self.ax1.legend()

            plt.tight_layout()

    def pathCallback(self, msg: Path):
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
        self.waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        rospy.loginfo("got waypoints")
        if self.visualize:
            self.updateWaypointsPlot()

    def updatePose(self, msg: Odometry):
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

        self.currentPosition[0] = msg.pose.pose.position.x
        self.currentPosition[1] = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        orientationList = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientationList)
        self.currentPosition[2] = yaw

    def mapVelocity(self, velocity: float):
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
        velocity = min(max(velocity, -self.MAXVELOCITY), self.MAXVELOCITY)

        if velocity < 0:
            # Mapping negative values from -1.57 to 0 to the range 0 to 61
            return int(((velocity + self.MAXVELOCITY) / self.MAXVELOCITY) * 61)

        return int((velocity / self.MAXVELOCITY) * 60 + 67)

    def setVelocity(self, k: float):
        """
        This function calculates the suitable velocity from the curvature of the path
        (the lower the curvature the higher the speed)

        Parameters:
        -----
        k: the curvature of the path in 1/m
        ----
        Returns:
        ----
        velocity : the suitable velocity
        ----
        """
        velocity = self.MAXVELOCITY / (1 + abs(k))
        goalPoint = self.waypoints[-1]
        distanceToGoal = np.linalg.norm(
            np.array((self.currentPosition[0], self.currentPosition[1])) - np.array(goalPoint)
        )
        if distanceToGoal < 0.05:
            return 0
        return velocity

    def findLookaheadPoint(self, robotPosition: tuple):
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
        if self.localPath:
            self.indexLD = 0
        for i, waypoint in enumerate(self.waypoints[self.indexLD :], start=self.indexLD):
            print("new waypoint")
            distanceToRobot = np.linalg.norm(np.array(waypoint) - np.array(robotPosition))
            print("waypoint = " + str(waypoint[0]) + ", " + str(waypoint[1]))
            print("dist=" + str(distanceToRobot))

            if distanceToRobot < self.distLd:
                self.indexLD = i
                lookaheadPoint = waypoint

            else:
                lookaheadPoint = self.waypoints[self.indexLD]
                break
        if self.visualize:
            self.plotLookaheadPoint(lookaheadPoint[0], lookaheadPoint[1])
        return lookaheadPoint

    def purePursuit(self):
        """
        This function calculates and publishes the steering angle from the lookahead point
        using the pure pursuit equation

        Parameters:
        -----
        None
        ----
        Returns:
        ----
        None
        ----
        """
        lookaheadPoint = self.findLookaheadPoint((self.currentPosition[0], self.currentPosition[1]))

        if lookaheadPoint is not None:
            alpha = math.atan2(
                (lookaheadPoint[1] - self.currentPosition[1]),
                (lookaheadPoint[0] - self.currentPosition[0]),
            )
            actualLookahead = math.hypot(
                lookaheadPoint[0] - self.currentPosition[0],
                lookaheadPoint[1] - self.currentPosition[1],
            )
            theta = alpha - self.currentPosition[2]
            deltaX = actualLookahead * math.cos(theta)
            k = deltaX / actualLookahead**2
            self.debuggingLists["pastCurvature"].append(k)
            velocityCentre = self.setVelocity(k)
            # print("velocityCentre=" + str(velocityCentre))
            self.distLd = self.KL * velocityCentre + self.KC
            self.debuggingLists["pastLD"].append(self.distLd)
            self.debuggingLists["pastHeadings"].append(self.currentPosition[2])
            velocityRight = velocityCentre * (
                1 - self.WIDTH * deltaX / (actualLookahead * actualLookahead)
            )
            velocityLeft = velocityCentre * (
                1 + self.WIDTH * deltaX / (actualLookahead * actualLookahead)
            )

            velocityRight = min(max(velocityRight, -self.MAXVELOCITY), self.MAXVELOCITY)
            velocityLeft = min(max(velocityLeft, -self.MAXVELOCITY), self.MAXVELOCITY)

            self.debuggingLists["pastVL"].append(velocityLeft)
            self.debuggingLists["pastVR"].append(velocityRight)

            vrMapped = self.mapVelocity(velocityRight)
            vlMapped = self.mapVelocity(velocityLeft)
            # print(
            #     "Right: ",
            #     velocityRight,
            #     " Mapped Right:",
            #     vrMapped,
            #     " Left: ",
            #     velocityLeft,
            #     " Mapped Left:",
            #     vlMapped,
            # )

            self.velocityPublisher["LF"].publish(velocityLeft)
            self.velocityPublisher["RF"].publish(velocityRight)
            self.velocityPublisher["LM"].publish(velocityLeft)
            self.velocityPublisher["RM"].publish(velocityRight)
            self.velocityPublisher["LR"].publish(velocityLeft)
            self.velocityPublisher["RR"].publish(velocityRight)
            if self.visualize:
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
        self.debuggingLists["pastPositionsX"].append(self.currentPosition[0])
        self.debuggingLists["pastPositionsY"].append(self.currentPosition[1])

        # Update the robot path plot
        self.plots["robotPathPlot"][0].set_data(
            self.debuggingLists["pastPositionsX"], self.debuggingLists["pastPositionsY"]
        )
        self.plots["robotPositionPlot"][0].set_data(
            [self.currentPosition[0]], [self.currentPosition[1]]
        )  # Update the current position plot

    def plotLookaheadPoint(self, lookaheadX, lookaheadY):  ##added (edited)
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

        self.plots["lookaheadPointPlot"][0].set_data(
            [lookaheadX], [lookaheadY]
        )  # Update the current position

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
            waypointsX, waypointsY = zip(*self.waypoints)
        else:
            waypointsX, waypointsY = [], []

        if self.plots["waypointsPlot"][0] is not None:
            self.plots["waypointsPlot"][0].set_data(waypointsX, waypointsY)

    def shutdownSeq(self):  ##added (edited)
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
        dataFrame = pd.DataFrame(
            {
                "Vr": self.debuggingLists["pastVR"],
                "Vl": self.debuggingLists["pastVL"],
                "Positions_x": self.debuggingLists["pastPositionsX"],
                "Positions_y": self.debuggingLists["pastPositionsY"],
                "Lookahead distance": self.debuggingLists["pastLD"],
                "curvature": self.debuggingLists["pastCurvature"],
                "heading": self.debuggingLists["pastHeadings"],
            }
        )
        if self.waypoints:
            waypointsX, waypointsY = zip(*self.waypoints)
        else:
            waypointsX, waypointsY = [], []
        dataFrame2 = pd.DataFrame({"waypoint_x": waypointsX, "waypoint_y": waypointsY})
        dataFrame.to_csv("Roar_adaptive_pure_pursuit_trial_4.csv")
        dataFrame2.to_csv("waypoints_3.csv")


if __name__ == "__main__":
    try:
        control = Control()
        if control.log:
            rospy.on_shutdown(control.shutdownSeq)
        while not rospy.is_shutdown():
            if len(control.waypoints) > 0:
                control.purePursuit()
    except rospy.ROSInterruptException:
        pass
