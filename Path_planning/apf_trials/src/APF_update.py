#!/usr/bin/env python3
"""
APF Update Script for Path Planning

This script implements an Artificial Potential Field (APF) that acts
as a local planner to avoid dynamic obstacles (their positions are unknown)

"""
from typing import List, Dict
import threading
import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sympy import symbols, sqrt, cos, sin, atan2
import matplotlib.pyplot as plt
from roar_msgs.msg import Obstacle
from matplotlib.patches import Circle
from matplotlib.lines import Line2D


# from gazebo_msgs.msg import ModelStates, LinkStates

# This is the publisher node which publishes a path for the controller
rospy.init_node("Path_Planning_APF", anonymous=True)
pub = rospy.Publisher("/Path", Path, queue_size=10)
path = Path()
pub1 = rospy.Publisher(
    "/APF_Des_Pos", Pose, queue_size=10
)  # Identify the publisher "pub1" to publish on topic "/APF_Des_Pos" to send message of type "Pose"
rate = rospy.Rate(10)

# FLAG = 0	# flag to indicate if new data has been received
# POSMSG = Pose()	# variable to store the current POSITION
# POSITION = [0, 0, 0, 0, 0, 0] # initialization of the POSITION to be zero
# VELOCITYMSG= Twist() # variables to store current VELOCITY
# VELOCITY = [0, 0, 0, 0, 0, 0] # initialization of the VELOCITY to be zero

# def callback1(data:ModelStates):
#  """

#  This function feedbacks the ROVER current location

#  Parameters:
#  ----
#  data: this gives the ROVER POSITION and orientation
#  ----
#  Returns:
#  ----
#  None
#  ----
#  """
#  global POSMSG
#  global sub
#  global FLAG
#  global POSITION
#  global VELOCITYMSG
#  global VELOCITY
#  msg = data

#  POSMSG.position.x = round(msg.pose[1].position.x, 4)
#  POSMSG.position.y = round(msg.pose[1].position.y, 4)
#  POSMSG.position.z = round(msg.pose[1].position.z, 4)
#  POSMSG.orientation.x = round(msg.pose[1].orientation.x, 4)
#  POSMSG.orientation.y = round(msg.pose[1].orientation.y, 4)
#  POSMSG.orientation.z = round(msg.pose[1].orientation.z, 4)
#  POSMSG.orientation.w = round(msg.pose[1].orientation.w, 4)
#  [roll, pitch, yaw] = euler_from_quaternion(
# (POSMSG.orientation.x, POSMSG.orientation.y, POSMSG.orientation.z, POSMSG.orientation.w))
#  POSITION = [POSMSG.position.x,POSMSG.position.y,POSMSG.position.z,yaw, pitch, roll]
#  VELOCITYMSG.linear.x = round(msg.twist[1].linear.x, 4)
#  VELOCITYMSG.linear.y = round(msg.twist[1].linear.y, 4)
#  VELOCITYMSG.linear.z = round(msg.twist[1].linear.z, 4)
#  VELOCITYMSG.angular.x = round(msg.twist[1].angular.x, 4)
#  VELOCITYMSG.angular.y = round(msg.twist[1].angular.y, 4)
#  VELOCITYMSG.angular.z = round(msg.twist[1].angular.z, 4)
#  VELOCITY = [
# VELOCITYMSG.linear.x,
# VELOCITYMSG.linear.y,
# VELOCITYMSG.linear.z,
# VELOCITYMSG.angular.x,
# VELOCITYMSG.angular.y,
# VELOCITYMSG.angular.z]
#  FLAG = 1

# sub = rospy.Subscriber('/gazebo/model_states', ModelStates, callback1)
# #Identify the subscriber "sub2" to subscribe topic "/odom" of type "Odometry"
FLAG = 0  # Initialize flag by zero
POSMSG = Pose()  # Identify msg variable of data type Pose
POSITION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
VELOCITYMSG = Twist()
VELOCITY = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
OBSTACLEDIC: Dict[int, List[float]] = {}  # Dictionary to store obstacle positions and radii
obstacleLock = threading.Lock()  # Thread lock for OBSTACLEDIC
FLAG2 = 0  # Flag to indicate if obstacle data has been received
# APF Inputs
GOALPOS = [rospy.get_param("~xGoal"), rospy.get_param("~yGoal")]
apfParam = [
    rospy.get_param("~KATT"),
    rospy.get_param("~KREP"),
    rospy.get_param("~QSTAR"),
    rospy.get_param("~M"),
]  # [KATT,KREP,QSTAR,M]


def callback(data: Odometry) -> None:
    """

    This function feedbacks the ROVER current location

    Parameters:
    ----
    data: this gives the ROVER POSITION and orientation
    ----
    Returns:
    ----
    None
    ----
    """
    global FLAG, POSITION, VELOCITY  # pylint: disable=global-statement

    msg = data
    POSMSG.position.x = msg.pose.pose.position.x
    POSMSG.position.y = msg.pose.pose.position.y
    POSMSG.position.z = msg.pose.pose.position.z
    POSMSG.orientation.x = msg.pose.pose.orientation.x
    POSMSG.orientation.y = msg.pose.pose.orientation.y
    POSMSG.orientation.z = msg.pose.pose.orientation.z
    POSMSG.orientation.w = msg.pose.pose.orientation.w
    [roll, pitch, yaw] = euler_from_quaternion(
        (POSMSG.orientation.x, POSMSG.orientation.y, POSMSG.orientation.z, POSMSG.orientation.w)
    )
    POSITION = [POSMSG.position.x, POSMSG.position.y, POSMSG.position.z, yaw, pitch, roll]
    VELOCITYMSG.linear.x = msg.twist.twist.linear.x
    VELOCITYMSG.linear.y = msg.twist.twist.linear.y
    VELOCITYMSG.linear.z = msg.twist.twist.linear.z
    VELOCITYMSG.angular.x = msg.twist.twist.angular.x
    VELOCITYMSG.angular.y = msg.twist.twist.angular.y
    VELOCITYMSG.angular.z = msg.twist.twist.angular.z
    VELOCITY = [
        VELOCITYMSG.linear.x,
        VELOCITYMSG.linear.y,
        VELOCITYMSG.linear.z,
        VELOCITYMSG.angular.x,
        VELOCITYMSG.angular.y,
        VELOCITYMSG.angular.z,
    ]
    FLAG = 1


SUB = rospy.Subscriber(
    "/odom", Odometry, callback
)  # Identify the subscriber "sub2" to subscribe topic "/odom" of type "Odometry"


def obstacleCallback(data: Obstacle) -> None:
    """
    Callback function to update the obstacle position and radius.

    Parameters:
    ----
    data: Obstacle message containing the obstacle's position and radius.
    ----
    Returns:
    ----
    None
    ----
    """
    global FLAG2  # pylint: disable=global-statement

    # Update the obstacle position and radius
    obstacleId = data.id.data  # Get obstacle ID
    radius = data.radius.data
    influenceRange = apfParam[2] + radius
    obstacleInfo = [
        data.position.pose.position.x,
        data.position.pose.position.y,
        radius,
        influenceRange,
    ]
    with obstacleLock:
        # Update the dictionary with the latest obstacle data
        OBSTACLEDIC[obstacleId] = obstacleInfo
    FLAG2 = 1


obstacleTopic = rospy.get_param("~obstacle_topic", default="obstacle_topic")
OBSTSUB = rospy.Subscriber("obstacle_topic", Obstacle, obstacleCallback)

while FLAG == 0 or FLAG2 == 0:  # Wait until the Rover receives a message before proceeding
    pass

# APF Equations
xRob = symbols("xRob")
yRob = symbols("yRob")
xGoal = symbols("xGoal")
yGoal = symbols("yGoal")
xObs = symbols("xObs")
yObs = symbols("yObs")
rangeEff = symbols("rangeEff")

# Attraction Forces Equations
attVal = sqrt((xRob - xGoal) ** 2 + (yRob - yGoal) ** 2)
attAngle = atan2((yGoal - yRob), (xGoal - xRob))
fxAtt = apfParam[0] * attVal * cos(attAngle)
fyAtt = apfParam[0] * attVal * sin(attAngle)
# Repulsion Forces Equations
distObs = sqrt((xRob - xObs) ** 2 + (yRob - yObs) ** 2)
repValue1 = apfParam[1] * ((1 / distObs) - (1 / rangeEff)) * (attVal ** apfParam[3] / attVal**3)
repValue2 = (
    apfParam[1] * apfParam[3] * (((1 / distObs) - (1 / rangeEff)) ** 2) * (attVal ** apfParam[3])
)
repValue = repValue1 + repValue2
repAngle = atan2((yObs - yRob), (xObs - xRob))
fxRep = -repValue * cos(repAngle)
fyRep = -repValue * sin(repAngle)
# Gradient Forces Equations
# def gradForce(robCurrentPos, robPreviousPos):
#     global fxGrad
#     global fyGrad
#     grad = (robCurrentPos[5] - robPreviousPos[5]) + (robCurrentPos[4] - robPreviousPos[4])
#     fxGrad = sym.exp(grad) * cos(robCurrentPos[3])
#     fyGrad = sym.exp(grad) * sin(robCurrentPos[3])


def apfFn(
    robPos: List[float], goalPos: List[float], obstDict: Dict[int, List[float]]
) -> List[float]:
    """
    This function calculates the total force

    Parameters:
    ----
    robPos: gives the ROVER POSITION
    goalPos: provides the goal POSITION
    obstDict: provides the obstacle positions and radii
    apfParam: provides the apf constants
    ----
    Returns:
    ----
    fxyNet: a list containing the x and y values of the force
    ----
    """
    distGoalVal = float(
        attVal.subs(
            [(xRob, robPos[0]), (yRob, robPos[1]), (xGoal, goalPos[0]), (yGoal, goalPos[1])]
        ).evalf()
    )

    fxAttVal = fxAtt.subs(
        [
            (xRob, robPos[0]),
            (xGoal, goalPos[0]),
            (yRob, robPos[1]),
            (yGoal, goalPos[1]),
            (attVal, distGoalVal),
        ]
    )
    fyAttVal = fyAtt.subs(
        [
            (yRob, robPos[1]),
            (yGoal, goalPos[1]),
            (xRob, robPos[0]),
            (xGoal, goalPos[0]),
            (attVal, distGoalVal),
        ]
    )
    fxRepTotal = 0
    fyRepTotal = 0
    with obstacleLock:
        for obstInfo in obstDict.values():
            distObstVal = float(
                distObs.subs(
                    [(xRob, robPos[0]), (yRob, robPos[1]), (xObs, obstInfo[0]), (yObs, obstInfo[1])]
                ).evalf()
            )

            if distObstVal < obstInfo[3]:
                fxRepVal = fxRep.subs(
                    [
                        (xRob, robPos[0]),
                        (yRob, robPos[1]),
                        (xObs, obstInfo[0]),
                        (yObs, obstInfo[1]),
                        (xGoal, goalPos[0]),
                        (yGoal, goalPos[1]),
                        (distObs, distObstVal),
                        (attVal, distGoalVal),
                        (rangeEff, obstInfo[3]),
                    ]
                )

                fyRepVal = fyRep.subs(
                    [
                        (xRob, robPos[0]),
                        (yRob, robPos[1]),
                        (xObs, obstInfo[0]),
                        (yObs, obstInfo[1]),
                        (xGoal, goalPos[0]),
                        (yGoal, goalPos[1]),
                        (distObs, distObstVal),
                        (attVal, distGoalVal),
                        (rangeEff, obstInfo[3]),
                    ]
                )
                fxRepTotal += fxRepVal
                fyRepTotal += fyRepVal

    fxNetVal = fxAttVal + fxRepTotal  # +fxGrad
    fyNetVal = fyAttVal + fyRepTotal  # +fyGrad
    fxyNet = [fxNetVal, fyNetVal]
    # rospy.loginfo(f"Attractive Force: fx = {fxAttVal}, fy = {fyAttVal}")
    # rospy.loginfo(f"Repulsive Force: fx = {fxRepVal}, fy = {fyRepVal}")
    # rospy.loginfo(f"Net Force: fx = {fxNetVal}, fy = {fyNetVal}")
    return fxyNet


#######################################################################################

# parameters to be initialized
TAU = rospy.get_param("~TAU")  # Sampling Time
ROBMASS = rospy.get_param("~ROBMASS")  # Robot Mass
plannedTrajectoryx = []
plannedTrajectoryy = []
waypoints = []

# the plotting function to plot the goal, obstacle, planned path
fig, ax = plt.subplots()
GOALREACHED = False  # marker for goal

while not rospy.is_shutdown() and not GOALREACHED:
    if FLAG == 1:  # to check if a message is being sent
        # initial ROVER POSITION and VELOCITY
        robPos0 = [POSITION[0], POSITION[1], POSITION[2], POSITION[3], POSITION[4], POSITION[5]]
        posPrev = robPos0
        for i in range(100):  # the loop which caculates the path the ROVER should follow
            fxyTotal = apfFn(posPrev, GOALPOS, OBSTACLEDIC)
            fNet = float(sqrt(fxyTotal[0] ** 2 + fxyTotal[1] ** 2))
            fNetDir = float(atan2(fxyTotal[1], fxyTotal[0]))
            # rospy.loginfo(f"Step {i}: fxyTotal = {fxyTotal}, fNet = {fNet}, angleDes = {fNetDir}")
            xDes = posPrev[0] + cos(fNetDir) * TAU
            yDes = posPrev[1] + sin(fNetDir) * TAU
            #  print("VELOCITY = "+str(math.hypot(velCurx,velCury))+"at point"+str(i))
            thetaDes = fNetDir
            robPosDes = [xDes, yDes, thetaDes]
            waypoints.append((posPrev[0], posPrev[1]))
            posPrev = [xDes, yDes]
            distGoal = sqrt((xDes - GOALPOS[0]) ** 2 + (yDes - GOALPOS[1]) ** 2)
            pose = PoseStamped()

            pose.pose.position.x = robPosDes[0]
            pose.pose.position.y = robPosDes[1]
            pose.pose.position.z = 0
            [qx_des, qy_des, qz_des, qw_des] = quaternion_from_euler(0, 0, robPosDes[2])
            pose.pose.orientation.x = qx_des
            pose.pose.orientation.y = qy_des
            pose.pose.orientation.z = qz_des
            pose.pose.orientation.w = qw_des
            path.poses.append(pose)
            if distGoal < 0.1:
                GOALREACHED = True
                break
    else:
        robPosDes = robPos0

    # placing the waypoints in the planned path to plot it
    ax.cla()
    (robotDot,) = ax.plot([], [], "bo", markersize=5, label="Robot")  # Robot marker
    (plannedPathLine,) = ax.plot(
        [], [], "ro", linewidth=2, label="Planned Path"
    )  # Precomputed path
    plt.scatter(GOALPOS[0], GOALPOS[1], color="green", label="Goal", s=100)  # Goal point
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.set_title("Live APF Path")
    obstacle_legend = Line2D(
        [0], [0], marker="o", color="w", markerfacecolor="red", markersize=10, label="Obstacle"
    )
    plt.legend(handles=[robotDot, plannedPathLine, obstacle_legend])
    ax.grid()
    plannedTrajectoryx = [wp[0] for wp in waypoints]
    plannedTrajectoryy = [wp[1] for wp in waypoints]
    plannedPathLine.set_data(plannedTrajectoryx, plannedTrajectoryy)
    waypoints = []
    pub.publish(path)
    pub1.publish(path.poses[0].pose)
    path = Path()
    robotDot.set_data(POSITION[0], POSITION[1])  # ploting the current ROVER POSITION dynamically
    for obstPlot in OBSTACLEDIC.values():
        obstacle = Circle(
            (obstPlot[0], obstPlot[1]), obstPlot[2], color="red", alpha=0.5, label="Obstacle"
        )
        ax.add_patch(obstacle)
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_aspect("equal")
    plt.pause(0.001)
    rate.sleep()
