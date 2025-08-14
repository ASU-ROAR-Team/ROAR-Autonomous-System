### README.md

# Drilling State Machine ROS Package

This package implements the autonomous and manual control logic for a drilling mechanism using a state machine. It orchestrates the entire drilling sequence, from initial positioning to sample collection and ascent, while also providing robust error handling and manual override capabilities.

-----

## Features

  * **State Machine Based Control:** Utilizes the `smach` library for a clear, structured control of the drilling process.
  * **Autonomous Drilling Sequence:** Automatically executes a predefined sequence: descending to surface, descending to sampling depth, collecting samples, and ascending.
  * **Manual Control Interface:** Allows direct human control over platform movement (up/down), auger activation, and gate operation.
  * **Parameterized Configuration:** Key operational parameters (heights, delays, timeouts) are configurable via a YAML file.
  * **Robustness:** Incorporates **timeouts** for critical operations to prevent indefinite hangs and uses **continuous command publishing** for improved communication reliability with low-level controllers.
  * **Emergency Stop:** Features a dedicated emergency stop state for immediate, safe shutdown of all actuators in case of critical errors or timeouts.

-----

## Nodes

### `drilling_state_machine.py`

This is the main node of the drilling package. It performs the following functions:

  * Initializes an ROS node named `drilling_state_machine`.
  * Manages the high-level states (Manual Control, Idle, Descending, Collecting, Ascending, Emergency Stop).
  * Publishes drilling commands (height targets, auger state, gate state, manual directions) via the `CanCommander` interface, which then formats them into CAN messages.
  * Offers the `/start_module` service to initiate the autonomous drilling sequence.

### `low_level_sim.py`

This is a utility node used for simulating the low-level hardware of the drilling mechanism.

  * Initializes an ROS node named `low_level_sim`.
  * Subscribes to drilling command topics from `drilling_state_machine.py`.
  * Simulates platform movement, auger operation, and gate actuation based on received commands.
  * Publishes simulated sensor data (`/height`, `/load_cell`, `/platform_motor/status`) back to the `drilling_state_machine.py`.

-----

## Parameters

The `drilling_state_machine` node is configured using parameters loaded from a YAML file (`drilling_params.yaml`).

| Parameter | Data Type | Default Value | Description |
| :--- | :--- | :--- | :--- |
| **`surface_height`** | `double` | `-11.0` | Target height in cm for initial descent. |
| **`sampling_height`** | `double` | `-40.0` | Target height in cm for sample collection. |
| **`gate_open_delay_time`** | `double` | `5.0` | Time (s) to wait at sampling height before closing the gate. |
| **`ascent_delay_time`** | `double` | `2.0` | Time (s) to wait after collecting a sample before ascending. |
| **`fsm_timeout`** | `double` | `60.0` | General timeout (s) for critical operations in states. |

-----

## Published Topics

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/platform_motor/target_height` | `std_msgs/Float64` | Target height in cm (for autonomous mode). Sent continuously by `CanCommander`. |
| `/auger_motor/cmd` | `std_msgs/Float64` | Auger motor command (`1.0` for ON, `0.0` for OFF). Sent continuously by `CanCommander`. |
| `/servo_gate/cmd` | `std_msgs/Float64` | Servo gate command (`1.0` for OPEN, `-1.0` for CLOSED). Sent continuously by `CanCommander`. |
| `/collect_sample_cmd` | `std_msgs/Float64` | Internal signal (`1.0`/`0.0`) to trigger load cell simulation. |

-----

## Services

### Offered

| Service | Type | Description |
| :--- | :--- | :--- |
| `/start_module` | `roar_msgs/StartModule.srv` | Initiates the automatic drilling sequence (used to transition from `MANUAL_CONTROL` to `IDLE`). |

### Called

This node does not directly call other ROS services.

-----

## Actions

This package does not currently implement any ROS Actions.

-----

## Custom Message

### Message: `roar_msgs/Drilling.msg`

```
# Commands for the drilling mechanism
float64 target_height_cm  # Target height in cm (for autonomous mode)
bool gate_open            # True for open, False for closed
bool auger_on             # True for on, False for off
bool manual_up            # True for manual up, False for off
bool manual_down          # True for manual down, False for off
```

This message is used internally by the `CanCommander` to encapsulate all drilling commands sent to the low-level system via the CAN bus.

-----

## Usage

To run the drilling state machine and its simulation, use the provided launch file:

```bash
roslaunch fsm_pkg Drilling.launch
```

Once launched, the state machine will be in `MANUAL_CONTROL`. You can then interact with it:

1.  **In the launch terminal:** Type `auto` and press Enter to transition to `IDLE`.
2.  **In a new terminal:** Call the service to start the autonomous sequence:
    ```bash
    rosservice call /start_module "{}"
    ```