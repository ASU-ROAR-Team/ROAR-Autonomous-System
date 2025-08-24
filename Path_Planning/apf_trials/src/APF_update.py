#!/usr/bin/env python3
"""
APF Path Planner with Checkpoint Enhancement, ObstacleArray, and Pure Radial Repulsive Force

This version removes the tangential repulsive force and memory system for
more stable multi-obstacle avoidance. The lookahead point is now determined
by stepping forward a fixed number of points from the closest point on the
global path, rather than by a fixed distance from the robot's position.
It also includes a new feature to ignore checkpoints if an obstacle is too close.
"""
from typing import List, Dict, Tuple, Any
import threading
import math
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
from sympy import symbols, sqrt, cos, sin, atan2, Expr, Piecewise
# import matplotlib.pyplot as plt # type: ignore
from roar_msgs.msg import ObstacleArray
from nav_msgs.msg import Odometry
import pandas as pd # type: ignore
import numpy as np # type: ignore
# from matplotlib.patches import Circle # type: ignore
import tf2_ros # type: ignore
import os # For path expansion

class APFPlanner:
    """Main APF planner class"""

    def __init__(self) -> None:
        rospy.init_node("apfPathPlanner", anonymous=True)

        # TF setup
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(15.0))
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.target_frame = rospy.get_param("~targetFrame", "world")

        # ROS components
        self.rosComponents: Dict[str, Any] = {
            "pathPub": rospy.Publisher("/Path", Path, queue_size=10),
            "modelSub": rospy.Subscriber("/zed2i/zed_node/pose", PoseStamped, self.modelCallback),
            "obstacleArraySub": rospy.Subscriber(
                rospy.get_param("~obstacleArrayTopic", "/zed_obstacle/obstacle_array"),
                ObstacleArray,
                self.obstacleArrayCallback,
                queue_size=5
            ),
            "rate": rospy.Rate(10),
        }

        # Configuration parameters
        self.config: Dict[str, Any] = {
            "checkpoints": [(12.3282, 6.7797), (15.1159, -3.0854), (20.127, 0.0),(19.7272, 5.2386)],
            "goalPoints": self.loadWaypoints(
                rospy.get_param("~pathFile", "~/ttt/src/ROAR-Autonomous-System//Path_Planning/heightmap_costmap/Results/real_path.csv")
            ),
            "costmap": pd.read_csv(
                rospy.get_param("~costmapFile", "~/ttt/src/ROAR-Autonomous-System//Path_Planning/heightmap_costmap/Results/total_cost.csv"), header=None
            ).values,
            "apfParams": {
                "KATT": rospy.get_param("~KATT", 8.0),
                "KREP": rospy.get_param("~KREP", 120.0),
                "QSTAR": rospy.get_param("~QSTAR", 2.0),
                "M": rospy.get_param("~M", 1.0),
                "LOOKAHEADDIST": rospy.get_param("~LOOKAHEADDIST", 2.0),
                "LOOKAHEAD_STEPS": rospy.get_param("~LOOKAHEAD_STEPS", 15), # New parameter for path-based lookahead
                "KGRADIENT": rospy.get_param("~KGRADIENT", 0.1),
                "KENHANCED": rospy.get_param("~KENHANCED", 80.0),
                "TAU": rospy.get_param("~TAU", 0.1),
                "CHECKPOINTDIST": rospy.get_param("~CHECKPOINTDIST", 1.8),
                "PIXEL_SCALE": rospy.get_param("~PIXEL_SCALE", 20.2),
                "GRADIENT_RADIUS": rospy.get_param("~GRADIENT_RADIUS", 1),
                "OBSTACLE_CHECKPOINT_DIST": rospy.get_param("~OBSTACLE_CHECKPOINT_DIST", 1.0), # New parameter
            },
        }

        # Robot state management
        self.robotState: Dict[str, Any] = {
            "position": [0.0] * 6,
            "isActive": False,
            "awayFromStart": False,
            "currentIndex": 0,
            "goalReached": False,
            "lastClosestIndex": 0
        }

        # APF components
        self.apfSymbols: Tuple[Expr, ...] = symbols("xRob yRob xGoal yGoal xObs yObs rangeEff")
        self.apfForces: Dict[str, Expr] = self.initAPFForces()

        # Obstacle handling
        self.obstacleData: Dict[str, Any] = {"obstacles": {}, "lock": threading.Lock()}

        # Visualization system
        # self.vizComponents: Dict[str, Any] = self.initVisualization()
        # rospy.loginfo("APF Planner Initialized. Target frame: %s", self.target_frame)
        # rospy.loginfo("APF Params: KATT=%.1f, KREP=%.1f",
        #               self.config["apfParams"]["KATT"],
        #               self.config["apfParams"]["KREP"])

    def initAPFForces(self) -> Dict[str, Expr]:
        """Initialize APF force equations. Tangential force is now dynamic."""
        symbol_list = self.apfSymbols
        parameters = self.config["apfParams"]
        xRob, yRob, xGoal, yGoal, xObs, yObs, rangeEff = symbol_list

        attVal: Expr = sqrt((xRob - xGoal) ** 2 + (yRob - yGoal) ** 2)
        attAngle: Expr = atan2(yGoal - yRob, xGoal - xRob)
        
        obsDist: Expr = sqrt((xRob - xObs) ** 2 + (yRob - yObs) ** 2)

        # Repulsive force magnitude when within range
        rep_force_magnitude_when_active = parameters["KREP"] * (1/obsDist - 1/rangeEff) * (1/obsDist**2)
        
        # We will now calculate the radial and tangential components dynamically
        # within the main force calculation loop, since they depend on the goal position.
        repX_radial = rep_force_magnitude_when_active * (xRob - xObs)
        repY_radial = rep_force_magnitude_when_active * (yRob - yObs)

        # The repulsive force only has a radial component in this sympy expression.
        # The tangential component will be calculated dynamically later.
        return {
            "attX": parameters["KATT"] * attVal * cos(attAngle),
            "attY": parameters["KATT"] * attVal * sin(attAngle),
            "repX": Piecewise((repX_radial, obsDist < rangeEff), (0, True)),
            "repY": Piecewise((repY_radial, obsDist < rangeEff), (0, True)),
        }

    # def initVisualization(self) -> Dict[str, Any]:
    #     """Initialize visualization components"""
    #     fig, axes = plt.subplots()
    #     xMin, xMax = 10, 22
    #     yMin, yMax = 13, -4 

    #     if self.config["costmap"] is None or self.config["costmap"].size == 0:
    #         rospy.logwarn("Costmap is empty or not loaded. Visualization will not show costmap.")
    #         costmap_display_array = np.zeros((10,10))
    #     else:
    #         pxMin, pyMax = self.realToPixel(xMin, yMin)
    #         pxMax, pyMin = self.realToPixel(xMax, yMax)
    #         costmap_display_array = np.flipud(
    #             self.config["costmap"][
    #                 max(0, pyMin) : min(self.config["costmap"].shape[0], pyMax),
    #                 max(0, pxMin) : min(self.config["costmap"].shape[1], pxMax),
    #             ]
    #         )

    #     img_artist = axes.imshow(
    #             costmap_display_array, cmap="viridis", origin="lower", extent=[xMin, xMax, yMin, yMax]
    #         )

    #     return {
    #         "figure": fig,
    #         "axes": axes,
    #         "image_artist": img_artist,
    #         "colorbar": plt.colorbar(img_artist, ax=axes),
    #         "limits": (xMin, xMax, yMin, yMax),
    #     }

    def modelCallback(self, msg: PoseStamped) -> None:
        """Handle robot state updates from PoseStamped"""
        try:
            # The PoseStamped message contains the pose directly
            self.robotState["position"] = [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                *euler_from_quaternion([
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w
                ]),
            ]
            if not self.robotState["isActive"]:
                rospy.loginfo(f"Robot pose received: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
            self.robotState["isActive"] = True
        except Exception as e:
            rospy.logerr(f"Error in modelCallback: {e}")

    def obstacleArrayCallback(self, msg: ObstacleArray) -> None:
        with self.obstacleData["lock"]:
            new_obstacle_data: Dict[int, List[float]] = {}
            Qstar = self.config["apfParams"]["QSTAR"]

            if msg.header.frame_id != self.target_frame:
                rospy.logwarn_throttle(5.0,
                    f"ObstacleArray received in frame '{msg.header.frame_id}' "
                    f"but APF expects '{self.target_frame}'. Obstacles might be misinterpreted if not in world frame.")

            for obs_ros in msg.obstacles:
                obs_id = obs_ros.id.data
                x_map = obs_ros.position.pose.position.x
                y_map = obs_ros.position.pose.position.y
                true_r = obs_ros.radius.data
                eff_r = true_r + Qstar

                new_obstacle_data[obs_id] = [x_map, y_map, true_r, eff_r]
            
            self.obstacleData["obstacles"] = new_obstacle_data


    def loadWaypoints(self, filePath: str) -> List[Tuple[float, float]]:
        """Load waypoints with explicit float conversion"""
        expanded_path = os.path.expanduser(filePath)
        if not os.path.exists(expanded_path):
            rospy.logerr(f"Waypoint file not found: {expanded_path}")
            return [(0.0, 0.0)]
        
        try:
            dataFrame = pd.read_csv(expanded_path)
            return [(float(row["real_x"]), float(row["real_y"])) for _, row in dataFrame.iterrows()]
        except Exception as e:
            rospy.logerr(f"Error loading waypoints from {expanded_path}: {e}")
            return [(0.0, 0.0)]


    def realToPixel(self, x: float, y: float) -> Tuple[int, int]:
        """Convert real-world coordinates to pixel indices"""
        pixel_scale = self.config["apfParams"]["PIXEL_SCALE"]
        return int(70 + pixel_scale * x), int(246 + pixel_scale * y)

    def calculateGradientForce(self, position: List[float]) -> Tuple[float, float]:
            """Calculate costmap gradient forces"""
            if self.config["costmap"] is None or self.config["costmap"].size == 0:
                return (0.0, 0.0)

            xPixel, yPixel = self.realToPixel(position[0], position[1])
            xGradient, yGradient = 0.0, 0.0
            radius_pixels = int(self.config["apfParams"]["GRADIENT_RADIUS"] * self.config["apfParams"]["PIXEL_SCALE"])

            if not (0 <= xPixel < self.config["costmap"].shape[1] and \
                    0 <= yPixel < self.config["costmap"].shape[0]):
                return (0.0, 0.0)

            current_cost = self.config["costmap"][yPixel][xPixel]

            for dx_pixel, dy_pixel_map_axis in [(radius_pixels, 0), (-radius_pixels, 0), (0, radius_pixels), (0, -radius_pixels)]:
                xPixelNew, yPixelNew = xPixel + dx_pixel, yPixel + dy_pixel_map_axis

                if (
                    0 <= xPixelNew < self.config["costmap"].shape[1]
                    and 0 <= yPixelNew < self.config["costmap"].shape[0]
                ):
                    neighbor_cost = self.config["costmap"][yPixelNew][xPixelNew]
                    costDiff = neighbor_cost - current_cost

                    if dx_pixel != 0:
                        xGradient -= self.config["apfParams"]["KGRADIENT"] * costDiff * np.sign(dx_pixel)
                    if dy_pixel_map_axis != 0:
                        yGradient -= self.config["apfParams"]["KGRADIENT"] * costDiff * np.sign(-dy_pixel_map_axis)
            return (xGradient, yGradient)


    def calculateEnhancedAttraction(self, position: List[float]) -> Tuple[float, float]:
        """Calculate enhanced checkpoint attraction"""
        xEnhanced, yEnhanced = 0.0, 0.0
        if self.config["checkpoints"]:
            cx, cy = self.config["checkpoints"][0]
            vec_to_checkpoint_x = cx - position[0]
            vec_to_checkpoint_y = cy - position[1]

            distance_to_checkpoint = math.hypot(vec_to_checkpoint_x, vec_to_checkpoint_y)

            if distance_to_checkpoint <= self.config["apfParams"]["CHECKPOINTDIST"] and distance_to_checkpoint > 0.01:
                angle_to_checkpoint = math.atan2(vec_to_checkpoint_y, vec_to_checkpoint_x)
                gain = self.config["apfParams"]["KENHANCED"] * self.config["apfParams"]["KATT"] * distance_to_checkpoint
                xEnhanced = gain * math.cos(angle_to_checkpoint)
                yEnhanced = gain * math.sin(angle_to_checkpoint)
        return (xEnhanced, yEnhanced)

    def update_checkpoints(self, position: List[float]) -> None:
        """Remove reached checkpoints or checkpoints too close to an obstacle."""
        while self.config["checkpoints"]:
            cx, cy = self.config["checkpoints"][0]
            
            # 1. Check if an obstacle is too close to the checkpoint
            obstacle_too_close = False
            with self.obstacleData["lock"]:
                for _id, obst_data in self.obstacleData["obstacles"].items():
                    obs_x, obs_y, _, _ = obst_data
                    dist_to_obstacle = math.hypot(cx - obs_x, cy - obs_y)
                    if dist_to_obstacle < self.config["apfParams"]["OBSTACLE_CHECKPOINT_DIST"]:
                        rospy.loginfo(f"Ignoring checkpoint {self.config['checkpoints'][0]} because an obstacle is too close.")
                        self.config["checkpoints"].pop(0)
                        obstacle_too_close = True
                        break # An obstacle is too close, move to the next checkpoint check
            if obstacle_too_close:
                continue # Restart the while loop with the next checkpoint

            # 2. Check if the robot has reached the checkpoint
            distance_to_checkpoint = math.hypot(
                (position[0] - cx), (position[1] - cy)
            )
            if distance_to_checkpoint <= self.config["apfParams"]["CHECKPOINTDIST"]:
                rospy.loginfo(f"Reached checkpoint: {self.config['checkpoints'][0]}")
                self.config["checkpoints"].pop(0)
                continue # Restart the while loop with the next checkpoint
            
            # If neither condition is met, we are done checking the first checkpoint
            break

        rospy.loginfo(f"Current checkpoints: {self.config['checkpoints']}")


    def calculateDynamicGoal(self, position: List[float]) -> Tuple[float, float]:
        """
        Determine current navigation target (lookahead point) by finding the closest
        point on the path and then stepping forward a fixed number of points.
        """
        if not self.robotState["awayFromStart"]:
            start_point_of_path = self.config["goalPoints"][0]
            distance_from_true_start = math.hypot(
                (position[0] - start_point_of_path[0]), (position[1] - start_point_of_path[1])
            )
            if distance_from_true_start >= 1.0:
                 if not self.robotState["awayFromStart"]:
                    rospy.loginfo("Robot moved >1m from path start. Using full path now.")
                 self.robotState["awayFromStart"] = True

        candidates = (
            self.config["goalPoints"]
            if self.robotState["awayFromStart"]
            else self.config["goalPoints"][:-30]
        )
        if not candidates:
            rospy.logwarn("No candidate points for dynamic goal. Using final goal of full path.")
            return (float(self.config["goalPoints"][-1][0]), float(self.config["goalPoints"][-1][1]))

        finalGoalOfCandidates: Tuple[float, float] = (float(candidates[-1][0]), float(candidates[-1][1]))

        # We've reached the end of the candidate path, so the final goal is our target
        if math.hypot((position[0] - finalGoalOfCandidates[0]), (position[1] - finalGoalOfCandidates[1])) < 0.5:
            return finalGoalOfCandidates

        # Find the closest point on the path to the robot
        min_dist_sq = float('inf')
        closest_idx = self.robotState.get("lastClosestIndex", 0)
        for i in range(closest_idx, len(candidates)):
            p_tuple = candidates[i]
            p = (float(p_tuple[0]), float(p_tuple[1]))
            dist_sq = (p[0] - position[0])**2 + (p[1] - position[1])**2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                closest_idx = i

        self.robotState["currentIndex"] = closest_idx
        self.robotState["lastClosestIndex"] = closest_idx

        # Calculate the lookahead point by stepping forward a fixed number of points
        lookahead_steps = self.config["apfParams"]["LOOKAHEAD_STEPS"]
        lookahead_idx = min(closest_idx + lookahead_steps, len(candidates) - 1)
        
        point_tuple = candidates[lookahead_idx]
        lookahead_point = (float(point_tuple[0]), float(point_tuple[1]))
        
        return lookahead_point


    def calculateForces(
        self, position: List[float], lookaheadPoint: Tuple[float, float]
    ) -> Tuple[float, float]:
        """
        Calculate total APF forces using attractive, repulsive, enhanced, and
        gradient forces. The repulsive force is now purely radial.
        """
        symbol_list = self.apfSymbols
        subs_dict = {
            symbol_list[0]: position[0], # xRob
            symbol_list[1]: position[1], # yRob
            symbol_list[2]: lookaheadPoint[0],     # xGoal
            symbol_list[3]: lookaheadPoint[1],     # yGoal
        }

        fxAtt = float(self.apfForces["attX"].evalf(subs=subs_dict))
        fyAtt = float(self.apfForces["attY"].evalf(subs=subs_dict))

        fxEnhanced, fyEnhanced = self.calculateEnhancedAttraction(position)

        fxRep_total, fyRep_total = 0.0, 0.0
        with self.obstacleData["lock"]:
            for obs_id, obst_params in self.obstacleData["obstacles"].items():
                obs_x, obs_y, _, obs_eff_radius = obst_params

                obs_dist = math.hypot(position[0] - obs_x, position[1] - obs_y)
                
                # Check if the robot is within the obstacle's effective range
                if obs_dist < obs_eff_radius:
                    # Calculate the radial repulsive force
                    rep_force_magnitude = self.config["apfParams"]["KREP"] * (1/obs_dist - 1/obs_eff_radius) * (1/obs_dist**2)
                    fxRep_radial = rep_force_magnitude * (position[0] - obs_x)
                    fyRep_radial = rep_force_magnitude * (position[1] - obs_y)
                    
                    fxRep_total += fxRep_radial
                    fyRep_total += fyRep_radial

        fxGrad, fyGrad = self.calculateGradientForce(position)
        
        return (
            fxAtt + fxEnhanced + fxRep_total + fxGrad,
            fyAtt + fyEnhanced + fyRep_total + fyGrad
        )

    # def updateVisualization(
    #     self, trajectory: List[Tuple[float, float]], lookaheadPoint: Tuple[float, float]
    # ) -> None:
    #     """Update real-time visualization"""
    #     if not plt.fignum_exists(self.vizComponents["figure"].number):
    #          rospy.logwarn_throttle(5.0,"Plot closed or not available, skipping visualization update.")
    #          return
    #     try:
    #         axes = self.vizComponents["axes"]
    #         axes.clear()

    #         if self.vizComponents["image_artist"].get_array().size > 1:
    #             axes.imshow(
    #                 self.vizComponents["image_artist"].get_array(),
    #                 cmap="viridis",
    #                 origin="lower",
    #                 extent=self.vizComponents["limits"],
    #             )

    #         if self.config["goalPoints"] and len(self.config["goalPoints"]) > 1:
    #             axes.plot(*zip(*self.config["goalPoints"]), "y-", linewidth=1, label="Global Path")

    #         axes.plot(
    #             self.robotState["position"][0],
    #             self.robotState["position"][1],
    #             "bo", markersize=6, label="Robot"
    #         )
    #         if trajectory:
    #             axes.plot(*zip(*trajectory), "r--", linewidth=1.5, label="Planned Segment")
    #         axes.scatter(
    #             lookaheadPoint[0], lookaheadPoint[1],
    #             color="cyan", marker="*", s=80, label="Lookahead Pt", zorder=5
    #         )

    #         if self.config["checkpoints"]:
    #             axes.scatter(
    #                 *zip(*self.config["checkpoints"]), color="magenta", marker="s", s=80, label="Checkpoints"
    #             )

    #         with self.obstacleData["lock"]:
    #             for _id, obst_data in self.obstacleData["obstacles"].items():
    #                 axes.add_patch(Circle((obst_data[0], obst_data[1]), obst_data[2], color="red", alpha=0.6, zorder=4))
    #                 axes.add_patch(Circle((obst_data[0], obst_data[1]), obst_data[3], color="orange", alpha=0.2, linestyle='--', zorder=3))


    #         if self.config["goalPoints"] and len(self.config["goalPoints"][0]) == 2:
    #             axes.plot(
    #                 self.config["goalPoints"][-1][0], self.config["goalPoints"][-1][1],
    #                 "go", markersize=8, label="Final Goal"
    #             )

    #         axes.legend(loc="upper left", fontsize='small')
    #         axes.set_xlabel("X Position (m)")
    #         axes.set_ylabel("Y Position (m)")
    #         axes.set_title("APF Path Planning")
    #         axes.set_xlim(self.vizComponents["limits"][0], self.vizComponents["limits"][1])
    #         axes.set_ylim(self.vizComponents["limits"][2], self.vizComponents["limits"][3])
    #         axes.set_aspect('equal', adjustable='box')
    #         plt.tight_layout()
    #         plt.pause(0.001)
    #     except Exception as e:
    #         rospy.logwarn_throttle(5.0, f"Error during visualization update: {e}")


    def publishPath(self, trajectory: List[Tuple[float, float]]) -> None:
        """Publish the planned trajectory segment as a nav_msgs/Path"""
        pathMsg = Path()
        pathMsg.header.stamp = rospy.Time.now()
        pathMsg.header.frame_id = self.target_frame

        for point_tuple in trajectory:
            point = (float(point_tuple[0]), float(point_tuple[1]))
            pose = PoseStamped()
            pose.header.stamp = pathMsg.header.stamp
            pose.header.frame_id = self.target_frame
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0
            pose.pose.orientation.w = 1.0
            pathMsg.poses.append(pose)

        self.rosComponents["pathPub"].publish(pathMsg)

    def run(self) -> None:
        """Main planning loop"""
        rospy.loginfo("APF Planner starting run loop.")
        # plt.ion()

        while not rospy.is_shutdown() and not self.robotState["goalReached"]:
            if not self.robotState["isActive"]:
                rospy.loginfo_throttle(5.0,"Waiting for robot pose...")
                self.rosComponents["rate"].sleep()
                continue

            currentPos = self.robotState["position"][:2]
            self.update_checkpoints(currentPos) # Updated to use the new method

            if not self.config["goalPoints"] or not self.config["goalPoints"][0]:
                rospy.logerr_throttle(5.0, "Goal points not loaded or empty. Stopping planner.")
                break
            finalGoal = (float(self.config["goalPoints"][-1][0]), float(self.config["goalPoints"][-1][1]))


            if (math.hypot((currentPos[0] - finalGoal[0]), (currentPos[1] - finalGoal[1])) <= 0.3) and self.robotState["awayFromStart"]:
                self.robotState["goalReached"] = True
                rospy.loginfo("Final goal reached!")
                self.publishPath([])
                break

            lookaheadPoint = self.calculateDynamicGoal(currentPos)

            trajectory: List[Tuple[float, float]] = []
            posCopy = list(currentPos)

            num_simulation_steps = 10
            step_size = self.config["apfParams"]["TAU"]

            for _ in range(num_simulation_steps):
                forces_x, forces_y = self.calculateForces(posCopy, lookaheadPoint)
                force_magnitude = math.hypot(forces_x, forces_y)

                if force_magnitude > 1e-4:
                    theta = math.atan2(forces_y, forces_x)
                    posCopy[0] += math.cos(theta) * step_size
                    posCopy[1] += math.sin(theta) * step_size
                
                trajectory.append((posCopy[0], posCopy[1]))


            if trajectory:
                # if plt.fignum_exists(self.vizComponents["figure"].number):
                #     self.updateVisualization(trajectory, lookaheadPoint)
                self.publishPath(trajectory)

            self.rosComponents["rate"].sleep()

        rospy.loginfo("APF Planner run loop finished.")
        # if plt:
        #     plt.close('all')


if __name__ == "__main__":
    try:
        planner = APFPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("APF Planner interrupted. Exiting.")
    except KeyboardInterrupt:
        rospy.loginfo("APF Planner interrupted by Ctrl+C. Exiting.")
    finally:
        # if plt:
        #     plt.close('all')
        pass