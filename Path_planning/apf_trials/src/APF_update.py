#!/usr/bin/env python3
"""
APF Path Planner with Checkpoint Enhancement
"""
from typing import List, Dict, Tuple, Any
import threading
import math
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sympy import symbols, sqrt, cos, sin, atan2, Expr
import matplotlib.pyplot as plt
from roar_msgs.msg import Obstacle
from matplotlib.patches import Circle
from gazebo_msgs.msg import ModelStates
import pandas as pd
import numpy as np


class APFPlanner:
    """Main APF planner class"""

    def __init__(self) -> None:
        """Initializes the instances in the APF code"""
        rospy.init_node("apfPathPlanner", anonymous=True)

        # ROS components
        self.rosComponents: Dict[str, Any] = {
            "pathPub": rospy.Publisher("/Path", Path, queue_size=10),
            "modelSub": rospy.Subscriber("/gazebo/model_states", ModelStates, self.modelCallback),
            "obstacleSub": rospy.Subscriber(
                rospy.get_param("~obstacleTopic", "obstacle_topic"), Obstacle, self.obstacleCallback
            ),
            "rate": rospy.Rate(10),
        }

        # Configuration parameters
        self.config: Dict[str, Any] = {
            "checkpoints": [(7.58, 10.874), (0.211, 9.454), (0.83, 19.867), (6.71, 19.515)],
            "goalPoints": self.loadWaypoints(
                rospy.get_param("~pathFile", "~/Downloads/real_path.csv")
            ),
            "costmap": pd.read_csv(
                rospy.get_param("~costmapFile", "~/Downloads/total_cost.csv"), header=None
            ).values,
            "apfParams": [
                rospy.get_param("~KATT"),
                rospy.get_param("~KREP"),
                rospy.get_param("~QSTAR"),
                rospy.get_param("~M"),
                rospy.get_param("~LOOKAHEADDIST"),
                rospy.get_param("~KGRADIENT"),
                rospy.get_param("~KENHANCED"),
                rospy.get_param("~TAU"),
                rospy.get_param("~CHECKPOINTDIST"),
            ],
        }

        # Robot state management
        self.robotState: Dict[str, Any] = {
            "position": [0.0] * 6,
            "isActive": False,
            "awayFromStart": False,
            "currentIndex": 0,
            "goalReached": False,
        }

        # APF components
        self.apfSymbols: Tuple[Expr, ...] = symbols("xRob yRob xGoal yGoal xObs yObs rangeEff")
        self.apfForces: Dict[str, Expr] = self.initAPFForces()

        # Obstacle handling
        self.obstacleData: Dict[str, Any] = {"obstacles": {}, "lock": threading.Lock()}

        # Visualization system
        self.vizComponents: Dict[str, Any] = self.initVisualization()

    def initAPFForces(self) -> Dict[str, Expr]:
        """Initialize APF force equations"""
        symbol = self.apfSymbols
        parameters = self.config["apfParams"]

        attVal: Expr = sqrt((symbol[0] - symbol[2]) ** 2 + (symbol[1] - symbol[3]) ** 2)
        attAngle: Expr = atan2(symbol[3] - symbol[1], symbol[2] - symbol[0])

        obsDist: Expr = sqrt((symbol[0] - symbol[4]) ** 2 + (symbol[1] - symbol[5]) ** 2)
        repTerm1: Expr = parameters[1] * (
            (1 / obsDist - 1 / symbol[6]) * (attVal ** parameters[3] / attVal**3)
        )
        repTerm2: Expr = (
            parameters[1]
            * parameters[3]
            * ((1 / obsDist - 1 / symbol[6]) ** 2 * attVal ** parameters[3])
        )
        repAngle: Expr = atan2(symbol[5] - symbol[1], symbol[4] - symbol[0])

        return {
            "attX": parameters[0] * attVal * cos(attAngle),
            "attY": parameters[0] * attVal * sin(attAngle),
            "repX": -(repTerm1 + repTerm2) * cos(repAngle),
            "repY": -(repTerm1 + repTerm2) * sin(repAngle),
        }

    def initVisualization(self) -> Dict[str, Any]:
        """Initialize visualization components"""
        fig = plt.figure()
        axes = fig.add_subplot(111)
        xMin, xMax = -2, 13
        yMin, yMax = 7, 22

        pxMin, pyMax = self.realToPixel(xMin, yMin)
        pxMax, pyMin = self.realToPixel(xMax, yMax)

        costmapRoi = np.flipud(
            self.config["costmap"][
                max(0, pyMin) : min(self.config["costmap"].shape[0], pyMax),
                max(0, pxMin) : min(self.config["costmap"].shape[1], pxMax),
            ]
        )

        return {
            "figure": fig,
            "axes": axes,
            "image": axes.imshow(
                costmapRoi, cmap="viridis", origin="lower", extent=[xMin, xMax, yMin, yMax]
            ),
            "colorbar": plt.colorbar(axes.imshow(costmapRoi), ax=axes),
            "limits": (xMin, xMax, yMin, yMax),
        }

    def modelCallback(self, msg: ModelStates) -> None:
        """Handle robot state updates"""
        pose = msg.pose[1]
        self.robotState["position"] = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            *euler_from_quaternion(
                [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            ),
        ]
        self.robotState["isActive"] = True

    def obstacleCallback(self, msg: Obstacle) -> None:
        """Update obstacle information"""
        with self.obstacleData["lock"]:
            self.obstacleData["obstacles"][msg.id.data] = [
                msg.position.pose.position.x,
                msg.position.pose.position.y,
                msg.radius.data,
                self.config["apfParams"][2] + msg.radius.data,
            ]

    def loadWaypoints(self, filePath: str) -> List[Tuple[float, float]]:
        """Load waypoints from CSV file"""
        dataFrame = pd.read_csv(filePath)
        return list(zip(dataFrame["real_x"], dataFrame["real_y"]))

    def realToPixel(self, x: float, y: float) -> Tuple[int, int]:
        """Convert real-world coordinates to pixel indices"""
        return int(208 + 20.2 * x), int(761 - 20.2 * y)

    def calculateGradientForce(self, position: List[float]) -> Tuple[float, float]:
        """Calculate costmap gradient forces"""
        xPixel, yPixel = self.realToPixel(position[0], position[1])
        xGradient, yGradient = 0.0, 0.0
        radius = int(0.5 * 20.2)

        for xDirection, yDirection in [(radius, 0), (-radius, 0), (0, radius), (0, -radius)]:
            xPixelNew, yPixelNew = xPixel + xDirection, yPixel - yDirection
            if (
                0 <= xPixelNew < self.config["costmap"].shape[1]
                and 0 <= yPixelNew < self.config["costmap"].shape[0]
            ):
                costDiff = (
                    self.config["costmap"][yPixelNew][xPixelNew]
                    - self.config["costmap"][yPixel][xPixel]
                )
                xGradient -= self.config["apfParams"][5] * costDiff * (1 if xDirection > 0 else -1)
                yGradient -= self.config["apfParams"][5] * costDiff * (1 if yDirection > 0 else -1)
        return (xGradient, yGradient)

    def calculateEnhancedAttraction(self, position: List[float]) -> Tuple[float, float]:
        """Calculate enhanced checkpoint attraction"""
        xEnhanced, yEnhanced = 0.0, 0.0
        if self.config["checkpoints"]:
            xDirection = position[0] - self.config["checkpoints"][0][0]
            yDirection = position[1] - self.config["checkpoints"][0][1]
            distance = sqrt(xDirection**2 + yDirection**2)

            if distance <= self.config["apfParams"][8]:
                angle = atan2(-yDirection, -xDirection)
                gain = self.config["apfParams"][6] * self.config["apfParams"][0] * distance
                xEnhanced = gain * cos(angle)
                yEnhanced = gain * sin(angle)
        return (xEnhanced, yEnhanced)

    def updateCheckpoints(self, position: List[float]) -> None:
        """Remove reached checkpoints"""
        if self.config["checkpoints"]:
            distance = sqrt(
                (position[0] - self.config["checkpoints"][0][0]) ** 2
                + (position[1] - self.config["checkpoints"][0][1]) ** 2
            )
            if distance <= 0.3:
                rospy.loginfo(f"Reached checkpoint: {self.config['checkpoints'][0]}")
                self.config["checkpoints"].pop(0)

    def calculateDynamicGoal(self, position: List[float]) -> Tuple[float, float]:
        """Determine current navigation target"""
        if not self.robotState["awayFromStart"]:
            startPoint = self.config["goalPoints"][-1]
            distance = sqrt((position[0] - startPoint[0]) ** 2 + (position[1] - startPoint[1]) ** 2)
            if distance >= 1.0:
                self.robotState["awayFromStart"] = True
                rospy.loginfo("Robot 1m from start, using full path")

        candidates = (
            self.config["goalPoints"]
            if self.robotState["awayFromStart"]
            else self.config["goalPoints"][:-30]
        )

        self.robotState["currentIndex"] = min(
            range(len(candidates)),
            key=lambda i: math.sqrt(
                (candidates[i][0] - position[0]) ** 2 + (candidates[i][1] - position[1]) ** 2
            ),
        )

        for idx in range(self.robotState["currentIndex"], len(candidates)):
            if (
                sqrt(
                    (candidates[idx][0] - position[0]) ** 2
                    + (candidates[idx][1] - position[1]) ** 2
                )
                >= self.config["apfParams"][4]
            ):
                return (candidates[idx][0], candidates[idx][1])
        return (candidates[-1][0], candidates[-1][0])

    def calculateForces(
        self, position: List[float], goal: Tuple[float, float]
    ) -> Tuple[float, float]:
        """Calculate total APF forces"""
        symbol = self.apfSymbols
        subs = {
            symbol[0]: position[0],
            symbol[1]: position[1],
            symbol[2]: goal[0],
            symbol[3]: goal[1],
        }

        fxAtt = float(self.apfForces["attX"].subs(subs).evalf())
        fyAtt = float(self.apfForces["attY"].subs(subs).evalf())

        fxEnhanced, fyEnhanced = self.calculateEnhancedAttraction(position)

        fxRep, fyRep = 0.0, 0.0
        with self.obstacleData["lock"]:
            for obst in self.obstacleData["obstacles"].values():
                subs.update({symbol[4]: obst[0], symbol[5]: obst[1], symbol[6]: obst[3]})
                if sqrt((position[0] - obst[0]) ** 2 + (position[1] - obst[1]) ** 2) < obst[3]:
                    fxRep += float(self.apfForces["repX"].subs(subs).evalf())
                    fyRep += float(self.apfForces["repY"].subs(subs).evalf())

        fxGrad, fyGrad = self.calculateGradientForce(position)

        return (fxAtt + fxEnhanced + fxRep + fxGrad, fyAtt + fyEnhanced + fyRep + fyGrad)

    def updateVisualization(
        self, trajectory: List[Tuple[float, float]], lookaheadPoint: Tuple[float, float]
    ) -> None:
        """Update real-time visualization"""
        axes = self.vizComponents["axes"]
        axes.clear()

        axes.imshow(
            self.vizComponents["image"].get_array(),
            cmap="viridis",
            origin="lower",
            extent=self.vizComponents["limits"],
        )

        axes.plot(*zip(*self.config["goalPoints"]), "y-", linewidth=2, label="Global Path")

        axes.plot(
            self.robotState["position"][0],
            self.robotState["position"][1],
            "bo",
            markersize=8,
            label="Robot",
        )

        axes.plot(*zip(*trajectory), "r--", linewidth=2, label="Planned Path")

        axes.scatter(
            lookaheadPoint[0],
            lookaheadPoint[1],
            color="c",
            marker="*",
            s=100,
            label="Lookahead Point",
        )

        if self.config["checkpoints"]:
            axes.scatter(
                *zip(*self.config["checkpoints"]), color="m", marker="s", s=100, label="Checkpoints"
            )

        with self.obstacleData["lock"]:
            for obst in self.obstacleData["obstacles"].values():
                axes.add_patch(Circle((obst[0], obst[1]), obst[2], color="red", alpha=0.5))

        axes.plot(
            self.config["goalPoints"][-1][0],
            self.config["goalPoints"][-1][1],
            "go",
            markersize=10,
            label="Final Goal",
        )

        axes.legend(loc="upper left")
        axes.set_xlabel("X Position (m)")
        axes.set_ylabel("Y Position (m)")
        axes.set_title("APF Path Planning Visualization")
        plt.pause(0.001)

    def publishPath(self, trajectory: List[Tuple[float, float]]) -> None:
        """Publish planned path for Controller"""
        pathMsg = Path()
        pathMsg.header.stamp = rospy.Time.now()
        pathMsg.header.frame_id = "map"

        for point in trajectory:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            orientation = quaternion_from_euler(0, 0, atan2(point[1], point[0]))
            pose.pose.orientation.x = orientation[0]
            pose.pose.orientation.y = orientation[1]
            pose.pose.orientation.z = orientation[2]
            pose.pose.orientation.w = orientation[3]
            pathMsg.poses.append(pose)

        self.rosComponents["pathPub"].publish(pathMsg)

    def run(self) -> None:
        """Main planning loop"""
        while not rospy.is_shutdown() and not self.robotState["goalReached"]:
            if not self.robotState["isActive"]:
                continue

            currentPos = self.robotState["position"][:2]
            self.updateCheckpoints(currentPos)

            finalGoal = self.config["goalPoints"][-1]
            if (
                self.robotState["awayFromStart"]
                and sqrt((currentPos[0] - finalGoal[0]) ** 2 + (currentPos[1] - finalGoal[1]) ** 2)
                <= 0.2
            ):
                self.robotState["goalReached"] = True
                rospy.loginfo("Final goal reached!")
                break

            lookaheadPoint = self.calculateDynamicGoal(currentPos)

            trajectory: List[Tuple[float, float]] = []
            posCopy = list(currentPos)
            for _ in range(20):
                forces = self.calculateForces(posCopy, lookaheadPoint)
                theta = atan2(forces[1], forces[0])
                posCopy = [
                    posCopy[0] + cos(theta) * self.config["apfParams"][7],
                    posCopy[1] + sin(theta) * self.config["apfParams"][7],
                ]
                trajectory.append((posCopy[0], posCopy[1]))

            self.updateVisualization(trajectory, lookaheadPoint)
            self.publishPath(trajectory)
            self.rosComponents["rate"].sleep()


if __name__ == "__main__":
    try:
        planner = APFPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
