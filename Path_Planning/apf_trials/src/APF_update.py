#!/usr/bin/env python3
"""
APF Path Planner with Checkpoint Enhancement and ObstacleArray support
"""
from typing import List, Dict, Tuple, Any
import threading
import math
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sympy import symbols, sqrt, cos, sin, atan2, Expr, Piecewise # <<< IMPORT Piecewise
import matplotlib.pyplot as plt # type: ignore
from roar_msgs.msg import ObstacleArray
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import pandas as pd # type: ignore
import numpy as np # type: ignore
from matplotlib.patches import Circle # type: ignore
import tf2_ros # type: ignore
# import tf2_geometry_msgs # Not strictly needed if obstacles are already in target frame


class APFPlanner:
    """Main APF planner class"""

    def __init__(self) -> None:
        rospy.init_node("apfPathPlanner", anonymous=True)

        # TF setup (still useful for robot pose if not using ground_truth directly)
        self.tfBuffer   = tf2_ros.Buffer(rospy.Duration(15.0))
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        # target frame for all transforms AND expected obstacle frame
        self.target_frame = rospy.get_param("~targetFrame", "world")

        # ROS components
        self.rosComponents: Dict[str, Any] = {
            "pathPub": rospy.Publisher("/Path", Path, queue_size=10),
            "modelSub": rospy.Subscriber("/filtered_state", Odometry, self.modelCallback),
            "obstacleArraySub": rospy.Subscriber(
                rospy.get_param("~obstacleArrayTopic", "/zed_obstacle/obstacle_array"),
                ObstacleArray,
                self.obstacleArrayCallback,
                queue_size=5 # Increased queue size slightly
            ),
            "rate": rospy.Rate(10),
        }

        # Configuration parameters
        self.config: Dict[str, Any] = {
            "checkpoints": [(7.58, 10.874), (0.211, 9.454), (0.83, 19.867), (6.71, 19.515)],
            "goalPoints": self.loadWaypoints(
                rospy.get_param("~pathFile", "~/ttt/src/ROAR-Autonomous-System//Path_Planning/heightmap_costmap/Results/real_path.csv") # Use absolute path or find_package
            ),
            "costmap": pd.read_csv(
                rospy.get_param("~costmapFile", "~/ttt/src/ROAR-Autonomous-System//Path_Planning/heightmap_costmap/Results/total_cost.csv"), header=None # Use absolute path or find_package
            ).values,
            "apfParams": {
                "KATT": rospy.get_param("~KATT", 8.0),
                "KREP": rospy.get_param("~KREP", 120.0),
                "QSTAR": rospy.get_param("~QSTAR", 2.0),
                "M": rospy.get_param("~M", 1.0),
                "LOOKAHEADDIST": rospy.get_param("~LOOKAHEADDIST", 2.0),
                "KGRADIENT": rospy.get_param("~KGRADIENT", 0.1),
                "KENHANCED": rospy.get_param("~KENHANCED", 80.0),
                "TAU": rospy.get_param("~TAU", 0.1),
                "CHECKPOINTDIST": rospy.get_param("~CHECKPOINTDIST", 1.8),
                "PIXEL_SCALE": rospy.get_param("~PIXEL_SCALE", 20.2),
                "GRADIENT_RADIUS": rospy.get_param("~GRADIENT_RADIUS", 1),
                # Force smoothing parameters
                "FORCE_SMOOTHING_FACTOR": rospy.get_param("~FORCE_SMOOTHING_FACTOR", 0.7),
                "MOMENTUM_FACTOR": rospy.get_param("~MOMENTUM_FACTOR", 0.3),
                "FORCE_THRESHOLD": rospy.get_param("~FORCE_THRESHOLD", 0.1),
                "OSCILLATION_DETECTION_WINDOW": rospy.get_param("~OSCILLATION_DETECTION_WINDOW", 5),
                "MAX_FORCE_CHANGE": rospy.get_param("~MAX_FORCE_CHANGE", 2.0),
            },
        }

        # Robot state management
        self.robotState: Dict[str, Any] = {
            "position": [0.0] * 6, # x, y, z, roll, pitch, yaw
            "isActive": False,
            "awayFromStart": False,
            "currentIndex": 0,
            "goalReached": False,
        }

        # APF components
        self.apfSymbols: Tuple[Expr, ...] = symbols("xRob yRob xGoal yGoal xObs yObs rangeEff")
        self.apfForces: Dict[str, Expr] = self.initAPFForces()

        # Obstacle handling
        # Obstacle dict: key=obstacle_id (int), value=[x_world, y_world, true_radius, effective_radius_apf]
        self.obstacleData: Dict[str, Any] = {"obstacles": {}, "lock": threading.Lock()}

        # Force smoothing and momentum tracking
        self.forceHistory: Dict[str, Any] = {
            "previous_force_x": 0.0,
            "previous_force_y": 0.0,
            "smoothed_force_x": 0.0,
            "smoothed_force_y": 0.0,
            "force_history_x": [],
            "force_history_y": [],
            "oscillation_detected": False,
            "lock": threading.Lock()
        }

        # Visualization system
        self.vizComponents: Dict[str, Any] = self.initVisualization()
        rospy.loginfo("APF Planner Initialized. Target frame: %s", self.target_frame)
        rospy.loginfo("APF Params: KATT=%.1f, KREP=%.1f, QSTAR=%.1f",
                      self.config["apfParams"]["KATT"], self.config["apfParams"]["KREP"], self.config["apfParams"]["QSTAR"])
        rospy.loginfo("Force Smoothing: factor=%.2f, momentum=%.2f, threshold=%.2f",
                      self.config["apfParams"]["FORCE_SMOOTHING_FACTOR"],
                      self.config["apfParams"]["MOMENTUM_FACTOR"],
                      self.config["apfParams"]["FORCE_THRESHOLD"])


    def initAPFForces(self) -> Dict[str, Expr]:
        """Initialize APF force equations"""
        symbol_list = self.apfSymbols
        parameters = self.config["apfParams"]

        xRob, yRob, xGoal, yGoal, xObs, yObs, rangeEff = symbol_list

        attVal: Expr = sqrt((xRob - xGoal) ** 2 + (yRob - yGoal) ** 2)
        attAngle: Expr = atan2(yGoal - yRob, xGoal - xRob)

        obsDist: Expr = sqrt((xRob - xObs) ** 2 + (yRob - yObs) ** 2)

        # Define the repulsive force magnitude when within range
        # U_rep = 0.5 * KREP * (1/obsDist - 1/rangeEff)**2
        # F_rep_component = KREP * (1/obsDist - 1/rangeEff) * (1/obsDist**3)
        # This is a standard formula for repulsive force from a potential field.
        # It pushes the robot directly away from the obstacle.
        # KREP is parameters[1]
        rep_force_magnitude_when_active = parameters["KREP"] * (1/obsDist - 1/rangeEff) * (1/obsDist**2) # Note: common formula uses 1/obsDist**3 not **2 for the last term when derived from U_rep.
                                                                                                 # Using your form: (1/obsDist**2)

        # Repulsive force vector components (pushing robot away from obstacle)
        # Vector from obstacle to robot is (xRob - xObs, yRob - yObs)
        # Normalized direction from obstacle to robot: (xRob - xObs)/obsDist, (yRob - yObs)/obsDist
        
        repX_active = rep_force_magnitude_when_active * (xRob - xObs) # Simplified: magnitude * component of unit vector
        repY_active = rep_force_magnitude_when_active * (yRob - yObs) # Simplified: magnitude * component of unit vector

        return {
            "attX": parameters["KATT"] * attVal * cos(attAngle),
            "attY": parameters["KATT"] * attVal * sin(attAngle),
            # Use Piecewise for conditional symbolic expression
            # Piecewise((expression, condition), (default_expression, True))
            "repX": Piecewise((repX_active, obsDist < rangeEff), (0, True)),
            "repY": Piecewise((repY_active, obsDist < rangeEff), (0, True)),
        }

    def initVisualization(self) -> Dict[str, Any]:
        """Initialize visualization components"""
        fig, axes = plt.subplots() # More conventional way
        xMin, xMax = -2, 13
        yMin, yMax = 7, 22

        # Ensure costmap is available
        if self.config["costmap"] is None or self.config["costmap"].size == 0:
            rospy.logwarn("Costmap is empty or not loaded. Visualization will not show costmap.")
            costmap_display_array = np.zeros((10,10)) # Placeholder
        else:
            pxMin, pyMax = self.realToPixel(xMin, yMin)
            pxMax, pyMin = self.realToPixel(xMax, yMax)
            costmap_display_array = np.flipud(
                self.config["costmap"][
                    max(0, pyMin) : min(self.config["costmap"].shape[0], pyMax),
                    max(0, pxMin) : min(self.config["costmap"].shape[1], pxMax),
                ]
            )

        img_artist = axes.imshow(
                costmap_display_array, cmap="viridis", origin="lower", extent=[xMin, xMax, yMin, yMax]
            )

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
                rospy.loginfo(f"Robot pose received: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}")
            self.robotState["isActive"] = True
        except Exception as e:
            rospy.logerr(f"Error in modelCallback: {e}")


    def obstacleArrayCallback(self, msg: ObstacleArray) -> None:
        with self.obstacleData["lock"]:
            new_obstacle_data: Dict[int, List[float]] = {} # Key: obs_id, Value: [x, y, true_r, eff_r]
            Qstar = self.config["apfParams"]["QSTAR"]

            if msg.header.frame_id != self.target_frame:
                rospy.logwarn_throttle(5.0,
                    f"ObstacleArray received in frame '{msg.header.frame_id}' "
                    f"but APF expects '{self.target_frame}'. Obstacles might be misinterpreted if not in world frame.")

            for obs_ros in msg.obstacles:
                # Assuming obs_ros.position is already in self.target_frame (e.g., "world")
                # as per the C++ node's design.
                obs_id = obs_ros.id.data

                x_map = obs_ros.position.pose.position.x
                y_map = obs_ros.position.pose.position.y
                true_r = obs_ros.radius.data
                eff_r  = true_r + Qstar

                new_obstacle_data[obs_id] = [x_map, y_map, true_r, eff_r]
                # rospy.logdebug(f"APF processed obstacle ID {obs_id} at (world) "
                #               f"x={x_map:.2f}, y={y_map:.2f}, R_true={true_r:.2f}, R_eff={eff_r:.2f}")

            self.obstacleData["obstacles"] = new_obstacle_data


    def loadWaypoints(self, filePath: str) -> List[Tuple[float, float]]:
        """Load waypoints with explicit float conversion"""
        # Expanduser for ~ paths
        expanded_path = os.path.expanduser(filePath)
        if not os.path.exists(expanded_path):
            rospy.logerr(f"Waypoint file not found: {expanded_path}")
            # Try to resolve path relative to package if filePath is not absolute
            # This requires knowing the package name or using roslaunch to set full path
            # For now, returning default.
            try:
                # This is a common pattern but requires APF_update.py to be in a specific location relative to the file
                # Or filePath to be a package-relative path like "pkg_name/path/to/file.csv"
                # A better way is to use roslaunch to pass the full path via rosparam
                # or use rospkg to find the file.
                import rospkg
                rospack = rospkg.RosPack()
                # Assuming filePath could be "my_pkg_name/data/real_path.csv"
                if '/' in filePath and not filePath.startswith('/'):
                    pkg_name = filePath.split('/')[0]
                    rel_path = '/'.join(filePath.split('/')[1:])
                    pkg_path = rospack.get_path(pkg_name)
                    expanded_path = os.path.join(pkg_path, rel_path)
                    if not os.path.exists(expanded_path):
                         rospy.logerr(f"Waypoint file still not found at package path: {expanded_path}")
                         return [(0.0,0.0)] # Return a default to avoid crash
                else: # If it's just a filename or already expanded and not found
                    return [(0.0,0.0)]
            except Exception as e_rospkg:
                rospy.logwarn(f"Could not use rospkg to find waypoint file {filePath}: {e_rospkg}")
                return [(0.0,0.0)]


        try:
            dataFrame = pd.read_csv(expanded_path)
            return [(float(row["real_x"]), float(row["real_y"])) for _, row in dataFrame.iterrows()]
        except Exception as e:
            rospy.logerr(f"Error loading waypoints from {expanded_path}: {e}")
            return [(0.0,0.0)]


    def realToPixel(self, x: float, y: float) -> Tuple[int, int]:
        """Convert real-world coordinates to pixel indices"""
        pixel_scale = self.config["apfParams"]["PIXEL_SCALE"]
        return int(208 + pixel_scale * x), int(761 - pixel_scale * y)

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

            if distance_to_checkpoint <= self.config["apfParams"]["CHECKPOINTDIST"] and distance_to_checkpoint > 0.01: # CHECKPOINTDIST
                angle_to_checkpoint = math.atan2(vec_to_checkpoint_y, vec_to_checkpoint_x)
                gain = self.config["apfParams"]["KENHANCED"] * self.config["apfParams"]["KATT"] * distance_to_checkpoint # KENHANCED * KATT * distance
                xEnhanced = gain * math.cos(angle_to_checkpoint)
                yEnhanced = gain * math.sin(angle_to_checkpoint)
        return (xEnhanced, yEnhanced)

    def smoothForces(self, current_force_x: float, current_force_y: float) -> Tuple[float, float]:
        """
        Apply force smoothing to prevent oscillatory behavior
        
        Args:
            current_force_x: Current calculated force in X direction
            current_force_y: Current calculated force in Y direction
            
        Returns:
            Tuple of smoothed forces (x, y)
        """
        with self.forceHistory["lock"]:
            # Get previous smoothed forces
            prev_smoothed_x = self.forceHistory["smoothed_force_x"]
            prev_smoothed_y = self.forceHistory["smoothed_force_y"]
            
            # Calculate force change magnitude
            force_change_x = current_force_x - self.forceHistory["previous_force_x"]
            force_change_y = current_force_y - self.forceHistory["previous_force_y"]
            force_change_magnitude = math.hypot(force_change_x, force_change_y)
            
            # Limit maximum force change to prevent sudden jumps
            max_change = self.config["apfParams"]["MAX_FORCE_CHANGE"]
            if force_change_magnitude > max_change:
                scale_factor = max_change / force_change_magnitude
                force_change_x *= scale_factor
                force_change_y *= scale_factor
            
            # Apply exponential smoothing
            smoothing_factor = self.config["apfParams"]["FORCE_SMOOTHING_FACTOR"]
            momentum_factor = self.config["apfParams"]["MOMENTUM_FACTOR"]
            
            # Smooth the current force
            smoothed_x = (smoothing_factor * current_force_x + 
                         (1 - smoothing_factor) * prev_smoothed_x)
            smoothed_y = (smoothing_factor * current_force_y + 
                         (1 - smoothing_factor) * prev_smoothed_y)
            
            # Add momentum from previous direction
            if abs(prev_smoothed_x) > self.config["apfParams"]["FORCE_THRESHOLD"] or \
               abs(prev_smoothed_y) > self.config["apfParams"]["FORCE_THRESHOLD"]:
                smoothed_x += momentum_factor * prev_smoothed_x
                smoothed_y += momentum_factor * prev_smoothed_y
            
            # Update force history
            self.forceHistory["previous_force_x"] = current_force_x
            self.forceHistory["previous_force_y"] = current_force_y
            self.forceHistory["smoothed_force_x"] = smoothed_x
            self.forceHistory["smoothed_force_y"] = smoothed_y
            
            # Store force history for oscillation detection
            self.forceHistory["force_history_x"].append(smoothed_x)
            self.forceHistory["force_history_y"].append(smoothed_y)
            
            # Keep only recent history
            max_history = self.config["apfParams"]["OSCILLATION_DETECTION_WINDOW"]
            if len(self.forceHistory["force_history_x"]) > max_history:
                self.forceHistory["force_history_x"] = self.forceHistory["force_history_x"][-max_history:]
                self.forceHistory["force_history_y"] = self.forceHistory["force_history_y"][-max_history:]
            
            # Detect oscillation
            self.detectOscillation()
            
            return smoothed_x, smoothed_y

    def detectOscillation(self) -> None:
        """Detect oscillatory behavior in force history"""
        if len(self.forceHistory["force_history_x"]) < 4:
            return
        
        # Calculate direction changes
        direction_changes = 0
        for i in range(1, len(self.forceHistory["force_history_x"])):
            prev_angle = math.atan2(self.forceHistory["force_history_y"][i-1], 
                                   self.forceHistory["force_history_x"][i-1])
            curr_angle = math.atan2(self.forceHistory["force_history_y"][i], 
                                   self.forceHistory["force_history_x"][i])
            
            # Calculate angle difference
            angle_diff = abs(curr_angle - prev_angle)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            # Count significant direction changes
            if angle_diff > math.pi / 2:  # 90 degrees
                direction_changes += 1
        
        # Detect oscillation if too many direction changes
        self.forceHistory["oscillation_detected"] = direction_changes >= 2
        
        if self.forceHistory["oscillation_detected"]:
            rospy.logwarn_throttle(2.0, "Oscillation detected in force vectors. Applying additional smoothing.")

    def applyOscillationCorrection(self, force_x: float, force_y: float) -> Tuple[float, float]:
        """
        Apply additional correction when oscillation is detected
        
        Args:
            force_x: Current force in X direction
            force_y: Current force in Y direction
            
        Returns:
            Corrected forces (x, y)
        """
        if not self.forceHistory["oscillation_detected"]:
            return force_x, force_y
        
        # When oscillation is detected, reduce force magnitude and increase smoothing
        correction_factor = 0.5  # Reduce force magnitude by 50%
        additional_smoothing = 0.8  # Apply additional smoothing
        
        # Get previous smoothed force
        prev_x = self.forceHistory["smoothed_force_x"]
        prev_y = self.forceHistory["smoothed_force_y"]
        
        # Apply correction
        corrected_x = (additional_smoothing * prev_x + 
                      (1 - additional_smoothing) * force_x) * correction_factor
        corrected_y = (additional_smoothing * prev_y + 
                      (1 - additional_smoothing) * force_y) * correction_factor
        
        return corrected_x, corrected_y

    def updateCheckpoints(self, position: List[float]) -> None:
        """Remove reached checkpoints"""
        if self.config["checkpoints"]:
            distance = math.hypot(
                (position[0] - self.config["checkpoints"][0][0]),
                (position[1] - self.config["checkpoints"][0][1])
            )
            if distance <= 0.3: # Threshold to consider checkpoint reached
                rospy.loginfo(f"Reached checkpoint: {self.config['checkpoints'][0]}")
                self.config["checkpoints"].pop(0)

    def calculateDynamicGoal(self, position: List[float]) -> Tuple[float, float]:
        """Determine current navigation target (lookahead point)"""
        if not self.robotState["awayFromStart"]:
            start_point_of_path = self.config["goalPoints"][0]
            distance_from_true_start = math.hypot(
                (position[0] - start_point_of_path[0]), (position[1] - start_point_of_path[1])
            )
            if distance_from_true_start >= 1.0 : # If robot has moved 1m from the very first waypoint
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

        if math.hypot((position[0] - finalGoalOfCandidates[0]), (position[1] - finalGoalOfCandidates[1])) < 0.5:
            return finalGoalOfCandidates

        min_dist_sq = float('inf')
        closest_idx = 0
        for i, p_tuple in enumerate(candidates):
            p = (float(p_tuple[0]), float(p_tuple[1])) # Ensure floats
            dist_sq = (p[0] - position[0])**2 + (p[1] - position[1])**2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                closest_idx = i

        self.robotState["currentIndex"] = closest_idx

        lookahead_dist = self.config["apfParams"]["LOOKAHEADDIST"] # LOOKAHEADDIST
        for idx in range(self.robotState["currentIndex"], len(candidates)):
            point_tuple = candidates[idx]
            point = (float(point_tuple[0]), float(point_tuple[1])) # Ensure floats
            distance_from_robot = math.hypot(
                (point[0] - position[0]), (point[1] - position[1])
            )
            if distance_from_robot >= lookahead_dist:
                return point

        return finalGoalOfCandidates

    def calculateForces(
        self, position: List[float], goal: Tuple[float, float]
    ) -> Tuple[float, float]:
        """Calculate total APF forces with smoothing"""
        symbol_list = self.apfSymbols
        subs_dict = {
            symbol_list[0]: position[0], # xRob
            symbol_list[1]: position[1], # yRob
            symbol_list[2]: goal[0],     # xGoal
            symbol_list[3]: goal[1],     # yGoal
        }

        fxAtt = float(self.apfForces["attX"].evalf(subs=subs_dict))
        fyAtt = float(self.apfForces["attY"].evalf(subs=subs_dict))

        fxEnhanced, fyEnhanced = self.calculateEnhancedAttraction(position)

        fxRep_total, fyRep_total = 0.0, 0.0
        with self.obstacleData["lock"]:
            for obs_id, obst_params in self.obstacleData["obstacles"].items():
                obs_x, obs_y, _, obs_eff_radius = obst_params

                subs_dict_rep = subs_dict.copy() # Start with robot and goal positions
                subs_dict_rep.update({
                    symbol_list[4]: obs_x,          # xObs
                    symbol_list[5]: obs_y,          # yObs
                    symbol_list[6]: obs_eff_radius  # rangeEff
                })
                try:
                    # Evaluate the Piecewise expression
                    fxRep_current = float(self.apfForces["repX"].evalf(subs=subs_dict_rep))
                    fyRep_current = float(self.apfForces["repY"].evalf(subs=subs_dict_rep))
                    fxRep_total += fxRep_current
                    fyRep_total += fyRep_current
                except Exception as e:
                    rospy.logwarn_throttle(5.0, f"Error evaluating repulsive force for obs {obs_id}: {e}. Subs: {subs_dict_rep}")

        fxGrad, fyGrad = self.calculateGradientForce(position)

        # Calculate raw total forces
        raw_fx_total = fxAtt + fxEnhanced + fxRep_total + fxGrad
        raw_fy_total = fyAtt + fyEnhanced + fyRep_total + fyGrad

        # Apply force smoothing to prevent oscillatory behavior
        smoothed_fx, smoothed_fy = self.smoothForces(raw_fx_total, raw_fy_total)
        
        # Apply oscillation correction if needed
        final_fx, final_fy = self.applyOscillationCorrection(smoothed_fx, smoothed_fy)

        # Log force components for debugging (throttled to avoid spam)
        rospy.logdebug_throttle(1.0, 
            f"Forces: Raw({raw_fx_total:.2f},{raw_fy_total:.2f}), "
            f"Smoothed({smoothed_fx:.2f},{smoothed_fy:.2f}), "
            f"Final({final_fx:.2f},{final_fy:.2f})")

        return (final_fx, final_fy)

    def updateVisualization(
        self, trajectory: List[Tuple[float, float]], lookaheadPoint: Tuple[float, float]
    ) -> None:
        """Update real-time visualization"""
        if not plt.fignum_exists(self.vizComponents["figure"].number): # Check if figure still exists
             rospy.logwarn_throttle(5.0,"Plot closed or not available, skipping visualization update.")
             return
        try:
            axes = self.vizComponents["axes"]
            axes.clear()

            if self.vizComponents["image_artist"].get_array().size > 1: # Check if costmap data is valid
                axes.imshow(
                    self.vizComponents["image_artist"].get_array(),
                    cmap="viridis",
                    origin="lower",
                    extent=self.vizComponents["limits"],
                )

            if self.config["goalPoints"] and len(self.config["goalPoints"]) > 1 and len(self.config["goalPoints"][0]) == 2 :
                axes.plot(*zip(*self.config["goalPoints"]), "y-", linewidth=1, label="Global Path")

            rospy.logdebug(f"Robot position: {self.robotState['position'][:2]}")
            # Plot robot position
            robot_x, robot_y = self.robotState["position"][0], self.robotState["position"][1]
            axes.plot(robot_x, robot_y, "bo", markersize=6, label="Robot")
            
            # Plot force vectors
            with self.forceHistory["lock"]:
                if abs(self.forceHistory["smoothed_force_x"]) > 0.01 or abs(self.forceHistory["smoothed_force_y"]) > 0.01:
                    # Scale force vector for visualization
                    force_scale = 0.5
                    force_x = self.forceHistory["smoothed_force_x"] * force_scale
                    force_y = self.forceHistory["smoothed_force_y"] * force_scale
                    
                    # Plot force vector
                    axes.arrow(robot_x, robot_y, force_x, force_y, 
                              head_width=0.2, head_length=0.3, fc='red', ec='red', 
                              alpha=0.7, label="APF Force")
                    
                    # Add oscillation indicator
                    if self.forceHistory["oscillation_detected"]:
                        axes.text(robot_x + 1, robot_y + 1, "OSCILLATION", 
                                color='red', fontsize=10, fontweight='bold',
                                bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.7))
            
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
            rospy.logwarn_throttle(5.0, f"Error during visualization update: {e}")


    def publishPath(self, trajectory: List[Tuple[float, float]]) -> None:
        """Publish the planned trajectory segment as a nav_msgs/Path"""
        pathMsg = Path()
        pathMsg.header.stamp = rospy.Time.now()
        pathMsg.header.frame_id = self.target_frame

        for point_tuple in trajectory:
            point = (float(point_tuple[0]), float(point_tuple[1])) # Ensure floats
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
        plt.ion() # Turn on interactive mode for plotting

        while not rospy.is_shutdown() and not self.robotState["goalReached"]:
            if not self.robotState["isActive"]:
                rospy.loginfo_throttle(5.0,"Waiting for robot pose...")
                self.rosComponents["rate"].sleep()
                continue

            currentPos = self.robotState["position"][:2]
            self.updateCheckpoints(currentPos)

            if not self.config["goalPoints"] or not self.config["goalPoints"][0]: # Check if goalPoints is valid
                rospy.logerr_throttle(5.0, "Goal points not loaded or empty. Stopping planner.")
                break
            finalGoal = (float(self.config["goalPoints"][-1][0]), float(self.config["goalPoints"][-1][1]))


            if (math.hypot((currentPos[0] - finalGoal[0]), (currentPos[1] - finalGoal[1])) <= 0.3):
                self.robotState["goalReached"] = True
                rospy.loginfo("Final goal reached!")
                self.publishPath([])
                break

            lookaheadPoint = self.calculateDynamicGoal(currentPos)

            trajectory: List[Tuple[float, float]] = []
            posCopy = list(currentPos)

            num_simulation_steps = 10
            step_size = self.config["apfParams"]["TAU"] # TAU

            for _ in range(num_simulation_steps):
                forces_x, forces_y = self.calculateForces(posCopy, lookaheadPoint)
                force_magnitude = math.hypot(forces_x, forces_y)

                if force_magnitude > 1e-4: # If there's a significant force
                    theta = math.atan2(forces_y, forces_x)
                    posCopy[0] += math.cos(theta) * step_size
                    posCopy[1] += math.sin(theta) * step_size
                # else: # If forces are negligible, robot effectively stops or hovers
                    # rospy.logdebug_throttle(1.0, "Negligible forces, robot will hover.")
                trajectory.append((posCopy[0], posCopy[1]))


            if trajectory:
                if plt.fignum_exists(self.vizComponents["figure"].number):
                    self.updateVisualization(trajectory, lookaheadPoint)
                self.publishPath(trajectory)

            self.rosComponents["rate"].sleep()

        rospy.loginfo("APF Planner run loop finished.")
        if plt.fignum_exists(self.vizComponents["figure"].number):
            plt.ioff()
            # plt.show() # Keep plot open if desired, but often not needed for ROS nodes
        plt.close('all')


if __name__ == "__main__":
    import os # For path expansion
    try:
        planner = APFPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("APF Planner interrupted. Exiting.")
    except KeyboardInterrupt:
        rospy.loginfo("APF Planner interrupted by Ctrl+C. Exiting.")
    finally:
        if plt: # Check if plt was imported and used
            plt.close('all')