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
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf_transformations import euler_from_quaternion
from sympy import symbols, sqrt, cos, sin, atan2, Expr, Piecewise
#from roar_msgs.msg import ObstacleArray
from nav_msgs.msg import Odometry
import pandas as pd
import numpy as np
import tf2_ros
import os
import matplotlib.pyplot as plt

class APFPlanner(Node):
    """Main APF planner class"""

    def __init__(self) -> None:
        super().__init__("apfPathPlanner")

        # TF setup
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        
        # Declare parameters
        self.declare_parameter("targetFrame", "world")
        #self.declare_parameter("obstacleArrayTopic", "/zed_obstacle/obstacle_array")
        self.declare_parameter("KATT", 8.0)
        self.declare_parameter("KREP", 120.0)
        self.declare_parameter("QSTAR", 2.0)
        self.declare_parameter("M", 1.0)
        self.declare_parameter("LOOKAHEADDIST", 2.0)
        self.declare_parameter("LOOKAHEAD_STEPS", 15)
        self.declare_parameter("KGRADIENT", 0.1)
        self.declare_parameter("KENHANCED", 80.0)
        self.declare_parameter("TAU", 0.1)
        self.declare_parameter("CHECKPOINTDIST", 1.8)
        self.declare_parameter("PIXEL_SCALE", 20.2)
        self.declare_parameter("GRADIENT_RADIUS", 1)
        self.declare_parameter("OBSTACLE_CHECKPOINT_DIST", 1.0)
        self.declare_parameter("pathFile", "~/roar_ws/ROAR-Autonomous-System/Path_Planning/heightmap_costmap/Results/real_path.csv")
        self.declare_parameter("costmapFile", "~/roar_ws/ROAR-Autonomous-System/Path_Planning/heightmap_costmap/Results/total_cost.csv")
        
        self.target_frame = self.get_parameter("targetFrame").get_parameter_value().string_value

        # ROS components
        self.pathPub = self.create_publisher(Path, "/Path", 10)
        self.modelSub = self.create_subscription(
            Odometry, 
            "/filtered_state", 
            self.modelCallback, 
            10
        )
        #self.obstacleArraySub = self.create_subscription(
        #    ObstacleArray,
        #    self.get_parameter("obstacleArrayTopic").get_parameter_value().string_value,
        #    self.obstacleArrayCallback,
        #    5
        #)
        
        # Create timer for main loop (10 Hz)
        self.timer = self.create_timer(0.1, self.run_iteration)

        # Configuration parameters
        self.config: Dict[str, Any] = {
            "checkpoints": [],
            "goalPoints": self.loadWaypoints(
                self.get_parameter("pathFile").get_parameter_value().string_value
            ),
            "costmap": pd.read_csv(
                self.get_parameter("costmapFile").get_parameter_value().string_value, header=None
            ).values,
            "apfParams": {
                "KATT": self.get_parameter("KATT").get_parameter_value().double_value,
                "KREP": self.get_parameter("KREP").get_parameter_value().double_value,
                "QSTAR": self.get_parameter("QSTAR").get_parameter_value().double_value,
                "M": self.get_parameter("M").get_parameter_value().double_value,
                "LOOKAHEADDIST": self.get_parameter("LOOKAHEADDIST").get_parameter_value().double_value,
                "LOOKAHEAD_STEPS": self.get_parameter("LOOKAHEAD_STEPS").get_parameter_value().integer_value,
                "KGRADIENT": self.get_parameter("KGRADIENT").get_parameter_value().double_value,
                "KENHANCED": self.get_parameter("KENHANCED").get_parameter_value().double_value,
                "TAU": self.get_parameter("TAU").get_parameter_value().double_value,
                "CHECKPOINTDIST": self.get_parameter("CHECKPOINTDIST").get_parameter_value().double_value,
                "PIXEL_SCALE": self.get_parameter("PIXEL_SCALE").get_parameter_value().double_value,
                "GRADIENT_RADIUS": self.get_parameter("GRADIENT_RADIUS").get_parameter_value().integer_value,
                "OBSTACLE_CHECKPOINT_DIST": self.get_parameter("OBSTACLE_CHECKPOINT_DIST").get_parameter_value().double_value,
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

        self.get_logger().info("APF Planner Initialized")

    def initAPFForces(self) -> Dict[str, Expr]:
        """Initialize APF force equations. Tangential force is now dynamic."""
        symbol_list = self.apfSymbols
        parameters = self.config["apfParams"]
        xRob, yRob, xGoal, yGoal, xObs, yObs, rangeEff = symbol_list

        attVal: Expr = sqrt((xRob - xGoal) ** 2 + (yRob - yGoal) ** 2)
        attAngle: Expr = atan2(yGoal - yRob, xGoal - xRob)
        
        obsDist: Expr = sqrt((xRob - xObs) ** 2 + (yRob - yObs) ** 2)

        rep_force_magnitude_when_active = parameters["KREP"] * (1/obsDist - 1/rangeEff) * (1/obsDist**2)
        
        repX_radial = rep_force_magnitude_when_active * (xRob - xObs)
        repY_radial = rep_force_magnitude_when_active * (yRob - yObs)

        return {
            "attX": parameters["KATT"] * attVal * cos(attAngle),
            "attY": parameters["KATT"] * attVal * sin(attAngle),
            "repX": Piecewise((repX_radial, obsDist < rangeEff), (0, True)),
            "repY": Piecewise((repY_radial, obsDist < rangeEff), (0, True)),
        }

    def initVisualization(self) -> Dict[str, Any]:
        """Initialize visualization components"""
        fig, axes = plt.subplots()
        xMin, xMax = 10, 22
        yMin, yMax = 13, -4 
        costmap_display_array = np.zeros((10,10))

        img_artist = axes.imshow(costmap_display_array, cmap="viridis", origin="lower", extent=[xMin, xMax, yMin, yMax])

        return {
           "figure": fig,
            "axes": axes,
            "image_artist": img_artist,
            "colorbar": plt.colorbar(img_artist, ax=axes),
            "limits": (xMin, xMax, yMin, yMax),
        }

    def modelCallback(self, msg: Odometry) -> None:
        """Handle robot state updates from Odometry"""
        try:
            self.robotState["position"] = [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                *euler_from_quaternion([
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w
                ]),
            ]
            if not self.robotState["isActive"]:
                self.get_logger().info(f"Robot pose received: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}")
            self.robotState["isActive"] = True
        except Exception as e:
            self.get_logger().error(f"Error in modelCallback: {e}")

    #def obstacleArrayCallback(self, msg: ObstacleArray) -> None:
        with self.obstacleData["lock"]:
            new_obstacle_data: Dict[int, List[float]] = {}
            Qstar = self.config["apfParams"]["QSTAR"]

            if msg.header.frame_id != self.target_frame:
                self.get_logger().warn(
                    f"ObstacleArray received in frame '{msg.header.frame_id}' "
                    f"but APF expects '{self.target_frame}'",
                    throttle_duration_sec=5.0
                )

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
            self.get_logger().error(f"Waypoint file not found: {expanded_path}")
            return [(0.0, 0.0)]
        
        try:
            dataFrame = pd.read_csv(expanded_path)
            return [(float(row["real_x"]), float(row["real_y"])) for _, row in dataFrame.iterrows()]
        except Exception as e:
            self.get_logger().error(f"Error loading waypoints from {expanded_path}: {e}")
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
            
            obstacle_too_close = False
            with self.obstacleData["lock"]:
                for _id, obst_data in self.obstacleData["obstacles"].items():
                    obs_x, obs_y, _, _ = obst_data
                    dist_to_obstacle = math.hypot(cx - obs_x, cy - obs_y)
                    if dist_to_obstacle < self.config["apfParams"]["OBSTACLE_CHECKPOINT_DIST"]:
                        self.get_logger().info(f"Ignoring checkpoint {self.config['checkpoints'][0]} because an obstacle is too close.")
                        self.config["checkpoints"].pop(0)
                        obstacle_too_close = True
                        break
            if obstacle_too_close:
                continue

            distance_to_checkpoint = math.hypot(
                (position[0] - cx), (position[1] - cy)
            )
            if distance_to_checkpoint <= self.config["apfParams"]["CHECKPOINTDIST"]:
                self.get_logger().info(f"Reached checkpoint: {self.config['checkpoints'][0]}")
                self.config["checkpoints"].pop(0)
                continue
            
            break

        self.get_logger().info(f"Current checkpoints: {self.config['checkpoints']}")

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
                    self.get_logger().info("Robot moved >1m from path start. Using full path now.")
                self.robotState["awayFromStart"] = True

        candidates = (
            self.config["goalPoints"]
            if self.robotState["awayFromStart"]
            else self.config["goalPoints"][:-30]
        )
        if not candidates:
            self.get_logger().warn("No candidate points for dynamic goal. Using final goal of full path.")
            return (float(self.config["goalPoints"][-1][0]), float(self.config["goalPoints"][-1][1]))

        finalGoalOfCandidates: Tuple[float, float] = (float(candidates[-1][0]), float(candidates[-1][1]))

        if math.hypot((position[0] - finalGoalOfCandidates[0]), (position[1] - finalGoalOfCandidates[1])) < 0.5:
            return finalGoalOfCandidates

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
            symbol_list[0]: position[0],
            symbol_list[1]: position[1],
            symbol_list[2]: lookaheadPoint[0],
            symbol_list[3]: lookaheadPoint[1],
        }

        fxAtt = float(self.apfForces["attX"].evalf(subs=subs_dict))
        fyAtt = float(self.apfForces["attY"].evalf(subs=subs_dict))

        fxEnhanced, fyEnhanced = self.calculateEnhancedAttraction(position)

        fxRep_total, fyRep_total = 0.0, 0.0
        with self.obstacleData["lock"]:
            for obs_id, obst_params in self.obstacleData["obstacles"].items():
                obs_x, obs_y, _, obs_eff_radius = obst_params

                obs_dist = math.hypot(position[0] - obs_x, position[1] - obs_y)
                
                if obs_dist < obs_eff_radius:
                    rep_force_magnitude = self.config["apfParams"]["KREP"] * (1/obs_dist - 1/obs_eff_radius) * (1/obs_dist**2)
                    fxRep_radial = rep_force_magnitude * (position[0] - obs_x)
                    fyRep_radial = rep_force_magnitude * (position[1] - obs_y)
                    
                    fxRep_total += fxRep_radial
                    fyRep_total += fyRep_radial
        
        return (
            fxAtt + fxEnhanced + fxRep_total,
            fyAtt + fyEnhanced + fyRep_total
        )

    def updateVisualization(self, trajectory: List[Tuple[float, float]], lookaheadPoint: Tuple[float, float]) -> None:
        """Update real-time visualization"""
        if not plt.fignum_exists(self.vizComponents["figure"].number):
            rclpy.logwarn_throttle(5.0,"Plot closed or not available, skipping visualization update.")
            return
        try:
            axes = self.vizComponents["axes"]
            axes.clear()

            if self.vizComponents["image_artist"].get_array().size > 1:
                axes.imshow(
                self.vizComponents["image_artist"].get_array(),
                cmap="viridis", 
                origin="lower",
                extent=self.vizComponents["limits"],
                )

            if self.config["goalPoints"] and len(self.config["goalPoints"]) > 1:
                axes.plot(*zip(*self.config["goalPoints"]), "y-", linewidth=1, label="Global Path")
                axes.plot(
                    self.robotState["position"][0],
                    self.robotState["position"][1],
                    "bo", markersize=6, label="Robot"
                )
            if trajectory:
                axes.plot(*zip(*trajectory), "r--", linewidth=1.5, label="Planned Segment")
                axes.scatter(
                    lookaheadPoint[0], lookaheadPoint[1],
                    color="cyan", marker="*", s=80, label="Lookahead Pt", zorder=5
                )

                if self.config["checkpoints"]:
                    axes.scatter(
                        *zip(*self.config["checkpoints"]), color="magenta", marker="s", s=80, label="Checkpoints"
                    )

                with self.obstacleData["lock"]:
                    for _id, obst_data in self.obstacleData["obstacles"].items():
                        axes.add_patch(Circle((obst_data[0], obst_data[1]), obst_data[2], color="red", alpha=0.6, zorder=4))
                        axes.add_patch(Circle((obst_data[0], obst_data[1]), obst_data[3], color="orange", alpha=0.2, linestyle='--', zorder=3))


                if self.config["goalPoints"] and len(self.config["goalPoints"][0]) == 2:
                    axes.plot(
                        self.config["goalPoints"][-1][0], self.config["goalPoints"][-1][1],
                        "go", markersize=8, label="Final Goal"
                    )

                axes.legend(loc="upper left", fontsize='small')
                axes.set_xlabel("X Position (m)")
                axes.set_ylabel("Y Position (m)")
                axes.set_title("APF Path Planning")
                axes.set_xlim(self.vizComponents["limits"][0], self.vizComponents["limits"][1])
                axes.set_ylim(self.vizComponents["limits"][2], self.vizComponents["limits"][3])
                axes.set_aspect('equal', adjustable='box')
                plt.tight_layout()
                plt.pause(0.001)
        except Exception as e:
            rclpy.logwarn_throttle(5.0, f"Error during visualization update: {e}")

    def publishPath(self, trajectory: List[Tuple[float, float]]) -> None:
        """Publish the planned trajectory segment as a nav_msgs/Path"""
        pathMsg = Path()
        pathMsg.header.stamp = self.get_clock().now().to_msg()
        pathMsg.header.frame_id = self.target_frame

        for point_tuple in trajectory:
            point = (float(point_tuple[0]), float(point_tuple[1]))
            pose = PoseStamped()
            pose.header.stamp = pathMsg.header.stamp
            pose.header.frame_id = self.target_frame
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            pathMsg.poses.append(pose)

        self.pathPub.publish(pathMsg)

    def run_iteration(self) -> None:
        """Main planning loop iteration (called by timer)"""
        if self.robotState["goalReached"]:
            return

        if not self.robotState["isActive"]:
            self.get_logger().info("Waiting for robot pose...", throttle_duration_sec=5.0)
            return

        currentPos = self.robotState["position"][:2]
        self.update_checkpoints(currentPos)

        if not self.config["goalPoints"] or not self.config["goalPoints"][0]:
            self.get_logger().error("Goal points not loaded or empty. Stopping planner.", throttle_duration_sec=5.0)
            return
        
        finalGoal = (float(self.config["goalPoints"][-1][0]), float(self.config["goalPoints"][-1][1]))

        if (math.hypot((currentPos[0] - finalGoal[0]), (currentPos[1] - finalGoal[1])) <= 0.3) and self.robotState["awayFromStart"]:
            self.robotState["goalReached"] = True
            self.get_logger().info("Final goal reached!")
            self.publishPath([])
            return

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
            self.updateVisualization(trajectory, lookaheadPoint)
            self.publishPath(trajectory)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        planner = APFPlanner()
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()