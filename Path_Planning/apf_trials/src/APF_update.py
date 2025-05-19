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
from roar_msgs.msg import Obstacle, ObstacleArray
from matplotlib.patches import Circle
from gazebo_msgs.msg import ModelStates
import pandas as pd
import numpy as np


class APFPlanner:
    """Main APF planner class"""

    def __init__(self) -> None:
        """Initializes the instances in the APF code"""
        rospy.init_node("apfPathPlanner")

        # ROS components
        self.rosComponents: Dict[str, Any] = {
            "pathPub": rospy.Publisher("/Path", Path, queue_size=10),
            "modelSub": rospy.Subscriber("/gazebo/model_states", ModelStates, self.modelCallback),
            "obstacleSub": rospy.Subscriber(
                rospy.get_param("~obstacleTopic", "obstacle_topic"), ObstacleArray, self.obstacleCallback
            ),
            "rate": rospy.Rate(10),
        }

        # Configuration parameters
        self.config: Dict[str, Any] = {
            "checkpoints": rospy.get_param("/Paths/checkpoints"),
            "goalPoints": self.loadWaypoints(
                rospy.get_param("/Paths/pathFile", "/home/dopebiscuit/roar/roar_ws/src/heightmap_costmap/Results/straight_line_path.csv")
            ),
            "costmap": pd.read_csv(
                rospy.get_param("/Paths/costmapFile", "/home/dopebiscuit/roar/roar_ws/src/heightmap_costmap/Results/path_costmap.csv"), header=None
            ).values,
            "apfParams": [
                rospy.get_param("~KATT",4),
                rospy.get_param("~KREP",100),
                rospy.get_param("~QSTAR",0.5),
                rospy.get_param("~M",1),
                rospy.get_param("~LOOKAHEADDIST",2),
                rospy.get_param("~KGRADIENT",0.1),
                rospy.get_param("~KENHANCED",60),
                rospy.get_param("~TAU",0.1),
                rospy.get_param("~CHECKPOINTDIST",1.8),
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
        self.obstacleData: Dict[str, Any] = {"obstacles": [], "lock": threading.Lock()}

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
        if len(msg.pose) > 1:
            pose = msg.pose[2]
            self.robotState["position"] = [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                *euler_from_quaternion(
                    [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                ),
            ]
            self.robotState["isActive"] = True
        else:
             rospy.logwarn("Robot model not found in ModelStates message.")
             self.robotState["isActive"] = False

    def obstacleCallback(self, msg: ObstacleArray) -> None:
        """Update obstacle information from an ObstacleArray message."""
        with self.obstacleData["lock"]:
            currentObstacles = []
            for obstacle in msg.obstacles:
                obstacleX = obstacle.position.pose.position.x
                obstacleY = obstacle.position.pose.position.y
                obsRadius = obstacle.radius.data
                currentObstacles.append([obstacleX, obstacleY, obsRadius])
            self.obstacleData["obstacles"] = currentObstacles

    def loadWaypoints(self, filePath: str) -> List[Tuple[float, float]]:
        """Load waypoints with explicit float conversion"""
        try:
            dataFrame = pd.read_csv(filePath)
            if "real_x" not in dataFrame.columns or "real_y" not in dataFrame.columns:
                rospy.logerr(f"Waypoint file {filePath} must contain 'real_x' and 'real_y' columns.")
                return []
            return [tuple(row) for row in dataFrame.values]

        except FileNotFoundError:
            rospy.logerr(f"Waypoint file not found: {filePath}")
            return []
        except Exception as e:
            rospy.logerr(f"Error loading waypoints from {filePath}: {e}")
            return []

    def realToPixel(self, x: float, y: float) -> Tuple[int, int]:
        """Convert real-world coordinates to pixel indices"""
        # pixel_x = int(208 + 20.2 * x)
        # pixel_y = int(761 - 20.2 * y)
        costmap_shape = self.config["costmap"].shape
        
        # Define workspace bounds (matching visualization limits)
        x_min, x_max = -2, 13
        y_min, y_max = 7, 22
        
        # Calculate scaling factors
        x_scale = costmap_shape[1] / (x_max - x_min)
        y_scale = costmap_shape[0] / (y_max - y_min)
        
        # Convert to pixel coordinates
        pixel_x = int((x - x_min) * x_scale)
        pixel_y = int((y_max - y) * y_scale)  # Flip y-axis since pixel coordinates are top-down
        
        return pixel_x, pixel_y

    def calculateGradientForce(self, position: List[float]) -> Tuple[float, float]:
        """Calculate costmap gradient forces"""
        xPixel, yPixel = self.realToPixel(position[0], position[1])
        xGradient, yGradient = 0.0, 0.0
        sampling_radius_pixels = int(0.5 * 20.2)

        costmap_shape = self.config["costmap"].shape
        costmap = self.config["costmap"]
        k_gradient = self.config["apfParams"][5]
        print("xPixel, yPixel (calculateGradientForce): ", xPixel, yPixel)
        print("costmap_shape (calculateGradientForce): ", costmap_shape)
        if not (0 <= yPixel < costmap_shape[0] and 0 <= xPixel < costmap_shape[1]):
            rospy.logwarn("Robot position outside costmap bounds, gradient force is zero.")
            return (0.0, 0.0)

        current_cost = costmap[yPixel][xPixel]

        directions = [(sampling_radius_pixels, 0), (-sampling_radius_pixels, 0), (0, sampling_radius_pixels), (0, -sampling_radius_pixels)]

        for dx_pixel, dy_pixel in directions:
            xPixelNew = xPixel + dx_pixel
            yPixelNew = yPixel + dy_pixel

            if 0 <= yPixelNew < costmap_shape[0] and 0 <= xPixelNew < costmap_shape[1]:
                new_cost = costmap[yPixelNew][xPixelNew]
                costDiff = new_cost - current_cost

                if dx_pixel != 0:
                    xGradient -= k_gradient * costDiff * (1 if dx_pixel < 0 else -1)
                if dy_pixel != 0:
                     yGradient -= k_gradient * costDiff * (1 if dy_pixel > 0 else -1)

        return (xGradient, yGradient)

    def calculateEnhancedAttraction(self, position: List[float]) -> Tuple[float, float]:
        """Calculate enhanced checkpoint attraction"""
        xEnhanced, yEnhanced = 0.0, 0.0
        if self.config["checkpoints"]:
            checkpoint = self.config["checkpoints"][0]
            xDirection = position[0] - checkpoint[0]
            yDirection = position[1] - checkpoint[1]
            distance = math.sqrt(xDirection**2 + yDirection**2)

            checkpoint_dist_threshold = self.config["apfParams"][8]
            k_enhanced = self.config["apfParams"][6]
            k_att = self.config["apfParams"][0]

            if distance <= checkpoint_dist_threshold:
                angle = math.atan2(checkpoint[1] - position[1], checkpoint[0] - position[0])
                gain = k_enhanced * k_att * (checkpoint_dist_threshold - distance)
                xEnhanced = gain * math.cos(angle)
                yEnhanced = gain * math.sin(angle)
        return (xEnhanced, yEnhanced)

    def updateCheckpoints(self, position: List[float]) -> None:
        """Remove reached checkpoints"""
        if self.config["checkpoints"]:
            distance = math.sqrt(
                (position[0] - self.config["checkpoints"][0][0]) ** 2
                + (position[1] - self.config["checkpoints"][0][1]) ** 2
            )
            if distance <= 0.5:
                rospy.loginfo(f"Reached checkpoint: {self.config['checkpoints'][0]}")
                self.config["checkpoints"].pop(0)
                if not self.config["checkpoints"]:
                    rospy.loginfo("All checkpoints reached.")

    def calculateDynamicGoal(self, position: List[float]) -> Tuple[float, float]:
        """Determine current navigation target (lookahead point)"""
        if not self.robotState["awayFromStart"]:
            startPoint = self.config["goalPoints"][-1]
            distance_from_start = math.sqrt(
                (position[0] - startPoint[0]) ** 2 + (position[1] - startPoint[1]) ** 2
            )
            if distance_from_start >= 1.0:
                self.robotState["awayFromStart"] = True
                rospy.loginfo("Robot is 1m from start, using the full path.")

        candidates = (
            self.config["goalPoints"]
            if self.robotState["awayFromStart"]
            else self.config["goalPoints"][:max(1, len(self.config["goalPoints"]) - 30)]
        )

        if not candidates:
            rospy.logwarn("No candidate goal points available.")
            return (position[0], position[1])

        full_final_goal = self.config["goalPoints"][-1]
        distance_to_full_final_goal = math.sqrt((position[0] - full_final_goal[0]) ** 2 + (position[1] - full_final_goal[1]) ** 2)
        if distance_to_full_final_goal < 0.5:
             rospy.loginfo("Close to the final global goal.")
             self.robotState["goalReached"] = True
             return full_final_goal


        start_search_index = max(0, self.robotState["currentIndex"] - 5)
        closest_index = start_search_index
        min_distance = float('inf')

        for i in range(start_search_index, len(candidates)):
            point = candidates[i]
            distance = math.sqrt((point[0] - position[0])**2 + (point[1] - position[1])**2)
            if distance < min_distance:
                min_distance = distance
                closest_index = i

        self.robotState["currentIndex"] = closest_index

        lookahead_dist = self.config["apfParams"][4]

        for idx in range(self.robotState["currentIndex"], len(candidates)):
            point = candidates[idx]
            typedPoint: Tuple[float, float] = (float(point[0]), float(point[1]))
            distance_from_robot = math.sqrt(
                (typedPoint[0] - position[0]) ** 2 + (typedPoint[1] - position[1]) ** 2
            )
            if distance_from_robot >= lookahead_dist:
                return typedPoint

        return (float(candidates[-1][0]), float(candidates[-1][1]))

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

        print("position (calculateForces): ", position)
        fxEnhanced, fyEnhanced = self.calculateEnhancedAttraction(position)
        print("fxEnhanced, fyEnhanced (calculateForces): ", fxEnhanced, fyEnhanced)
        fxRep, fyRep = 0.0, 0.0
        with self.obstacleData["lock"]:
            for obst in self.obstacleData["obstacles"]:
                obstX = obst[0]
                obstY = obst[1]
                obstRadius = obst[2]
                rangeEff = self.config["apfParams"][2] + obstRadius

                distanceToObs = math.sqrt((position[0] - obstX) ** 2 + (position[1] - obstY) ** 2)

                if 0 < distanceToObs < rangeEff:
                    obstacleSubs = {
                        symbol[4]: obstX,
                        symbol[5]: obstY,
                        symbol[6]: rangeEff
                    }
                    full_subs = {**subs, **obstacleSubs}

                    fxRep += float(self.apfForces["repX"].subs(full_subs).evalf())
                    fyRep += float(self.apfForces["repY"].subs(full_subs).evalf())

        fxGrad, fyGrad = self.calculateGradientForce(position)
        print("fxGrad, fyGrad (calculateForces): ", fxGrad, fyGrad)
        total_fx = fxAtt + fxEnhanced + fxRep + fxGrad
        total_fy = fyAtt + fyEnhanced + fyRep + fyGrad

        return (total_fx, total_fy)

    def updateVisualization(
        self, trajectory: List[Tuple[float, float]], lookaheadPoint: Tuple[float, float]
    ) -> None:
        """Update real-time visualization"""
        if not hasattr(self.vizComponents["axes"], 'lines'):
             rospy.logwarn("Visualization axes not properly initialized.")
             return

        axes = self.vizComponents["axes"]
        axes.clear()

        axes.imshow(
            self.vizComponents["image"].get_array(),
            cmap="viridis",
            origin="lower",
            extent=self.vizComponents["limits"],
        )

        if self.config["goalPoints"]:
            axes.plot(*zip(*self.config["goalPoints"]), "y-", linewidth=2, label="Global Path")

        axes.plot(
            self.robotState["position"][0],
            self.robotState["position"][1],
            "bo",
            markersize=8,
            label="Robot",
        )

        if trajectory:
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
            for obst in self.obstacleData["obstacles"]:
                 obstacllesX = obst[0]
                 obstaclesY = obst[1]
                 obstcalesRadius = obst[2]
                 axes.add_patch(Circle((obstacllesX, obstaclesY), obstcalesRadius, color="red", alpha=0.5))

        if self.config["goalPoints"]:
             final_goal = self.config["goalPoints"][-1]
             axes.plot(
                final_goal[0],
                final_goal[1],
                "go",
                markersize=10,
                label="Final Goal",
             )

        axes.legend(loc="upper left")
        axes.set_xlabel("X Position (m)")
        axes.set_ylabel("Y Position (m)")
        axes.set_title("APF Path Planning Visualization")
        axes.set_xlim(self.vizComponents["limits"][0], self.vizComponents["limits"][1])
        axes.set_ylim(self.vizComponents["limits"][2], self.vizComponents["limits"][3])
        plt.grid(True)
        plt.draw()
        plt.pause(0.001)

    def publishPath(self, trajectory: List[Tuple[float, float]]) -> None:
        """Publish planned path for Controller"""
        pathMsg = Path()
        pathMsg.header.stamp = rospy.Time.now()
        pathMsg.header.frame_id = "map"

        if not trajectory:
             rospy.logwarn("No trajectory planned to publish.")
             return

        for point in trajectory:
            pose = PoseStamped()
            pose.header = pathMsg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]

            angle = 0.0
            if len(trajectory) > 1:
                 current_index = trajectory.index(point)
                 if current_index < len(trajectory) - 1:
                      next_point = trajectory[current_index + 1]
                      angle = math.atan2(next_point[1] - point[1], next_point[0] - point[0])
                 elif current_index > 0:
                       prev_point = trajectory[current_index - 1]
                       angle = math.atan2(point[1] - prev_point[1], point[0] - prev_point[0])
                 else:
                       angle = self.robotState["position"][5]

            else:
                 angle = self.robotState["position"][5]

            orientation = quaternion_from_euler(0, 0, angle)
            pose.pose.orientation.x = orientation[0]
            pose.pose.orientation.y = orientation[1]
            pose.pose.orientation.z = orientation[2]
            pose.pose.orientation.w = orientation[3]
            pathMsg.poses.append(pose)

        self.rosComponents["pathPub"].publish(pathMsg)

    def run(self) -> None:
        """Main planning loop"""
        rospy.loginfo("APF Path Planner started.")
        while not rospy.is_shutdown():
            if not self.robotState["isActive"]:
                rospy.logdebug("Robot state not active, waiting for data...")
                self.rosComponents["rate"].sleep()
                continue

            currentPos = self.robotState["position"][:2]

            finalGoal = self.config["goalPoints"][-1] if self.config["goalPoints"] else None
            if finalGoal and math.sqrt((currentPos[0] - finalGoal[0]) ** 2 + (currentPos[1] - finalGoal[1]) ** 2) <= 0.5:
                 self.robotState["goalReached"] = True
                 rospy.loginfo("Final goal reached!")
                 break

            self.updateCheckpoints(currentPos)

            if not self.config["goalPoints"]:
                 rospy.logwarn("No goal points loaded. Planner cannot operate.")
                 self.rosComponents["rate"].sleep()
                 continue


            lookaheadPoint = self.calculateDynamicGoal(currentPos)

            trajectory: List[Tuple[float, float]] = []
            posCopy = list(currentPos)
            print(posCopy)
            for _ in range(20):
                forces = self.calculateForces(posCopy, lookaheadPoint)
                force_magnitude = math.sqrt(forces[0]**2 + forces[1]**2)
                if force_magnitude > 1e-6:
                    theta = math.atan2(forces[1], forces[0])
                    step_distance = self.config["apfParams"][7]
                    posCopy[0] += math.cos(theta) * step_distance
                    posCopy[1] += math.sin(theta) * step_distance
                    trajectory.append((posCopy[0], posCopy[1]))
                else:
                     trajectory.append((posCopy[0], posCopy[1]))

            self.updateVisualization(trajectory, lookaheadPoint)
            self.publishPath(trajectory)
            self.rosComponents["rate"].sleep()

        rospy.loginfo("APF Path Planner shutting down.")


if __name__ == "__main__":
    try:
        planner = APFPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass