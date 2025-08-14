# apf\_trials ROS Package

This package provides an advanced local planner that receives a global path from a high-level planner (like A\*) and uses it to generate a safe and efficient local path for a pure pursuit controller. It leverages an Artificial Potential Field (APF) approach, dynamically calculating various forces to navigate the robot through its environment.

-----

## Features

  - **Dynamic Local Path Planning:** Generates a real-time local path by considering multiple forces.
  - **APF-based Navigation:** Uses a combination of attractive, repulsive, gradient, and checkpoint forces for intelligent path following.
  - **Attractive Force:** Pulls the robot towards a dynamic lookahead point on the global path.
  - **Repulsive Force:** Pushes the robot away from obstacles detected by a perception module.
  - **Gradient Force:** Uses a costmap to guide the robot towards lower-cost areas.
  - **Checkpoint Force:** Prevents the robot from skipping critical points on the global path.
  - **Parameterized:** All force coefficients and other parameters can be easily configured via a YAML file.

-----

## Nodes

### `apfPathPlanner`

This is the main node of the `apf_trials` package. It performs the following functions:

  - Initializes an ROS node named `apfPathPlanner`.
  - Subscribes to odometry, obstacle, and global path topics.
  - Continuously calculates the net force on the robot based on the APF algorithm.
  - Generates a local path as a sequence of waypoints.
  - Publishes the final path to a controller node for execution.

### `Obstacle_publisher.py`

This is a utility node used for testing obstacle avoidance logic. It publishes a static or dynamic array of obstacles for the `apfPathPlanner` to react to.

-----

## Parameters

The `apfPathPlanner` node is configured using parameters from a YAML file.

| Parameter | Data Type | Default Value | Description |
| :--- | :--- | :--- | :--- |
| **`KATT`** | `double` | `8` | The gain for the attractive force towards the lookahead point. |
| **`KREP`** | `double` | `100` | The gain for the repulsive force from obstacles. |
| **`KENHANCED`** | `double` | `80` | The gain for the enhanced attraction force, used to improve path-following. |
| **`KGRADIENT`** | `double` | `0.1` | The coefficient for the gradient force derived from the costmap. |
| **`QSTAR`** | `double` | `0.8` | The influence range threshold for obstacle repulsion (in meters). |
| **`obstacle_topic`** | `string` | `"obstacleArrayTopic"` | The ROS topic name for incoming obstacle data. |
| **`OBSTACLE_CHECKPOINT_DIST`** | `double` | `0.5` | Distance to ignore checkpoints if they are too close to an obstacle. |
| **`M`** | `int` | `1` | The exponent for the potential field calculation. |
| **`CHECKPOINTDIST`** | `double` | `1.2` | The distance at which a checkpoint is considered reached (in meters). |
| **`LOOKAHEAD_STEPS`** | `int` | `9` | The number of steps to look ahead on the global path to find the target point. |
| **`pathFile`** | `string` | `"~"` | The file path to the global path CSV file. |
| **`costmapFile`** | `string` | `"~"` | The file path to the costmap CSV file. |
| **`TAU`** | `double` | `0.1` | The time step for the potential field simulation (in seconds). |
| **`PIXEL_SCALE`** | `int` | `20` | The scale factor for converting costmap pixels to meters. |
| **`GRADIENT_RADIUS`** | `int` | `1` | The radius for calculating the gradient force on the costmap. |

-----

## Published Topics

  - `/Path` (`nav_msgs/Path`): The final, dynamically generated local path for the pure pursuit controller to follow.

-----

## Subscribed Topics

  - `/filtered_state` (`nav_msgs/Odometry`): The robot's current pose (position and orientation) from the localization module.
  - `/zed_obstacle/obstacle_array` (`ObstacleArray`): An array of detected obstacles from the perception module.

-----

## Usage

To run the `apf_trials` package, it's recommended to use the provided launch file, which also handles the parameters:

```bash
roslaunch apf_trials APF.launch
```

You can also run the nodes individually and set parameters on the command line:

```bash
rosrun apf_trials APF_update.py _KATT:=10 _KREP:=150
```