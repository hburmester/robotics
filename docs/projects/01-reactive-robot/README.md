# Project 01 — Reactive Robot

A ROS 2 project implementing a simulated one-dimensional mobile robot with closed-loop distance control.

## Objective

The objective of this project is to build a small but complete closed-loop robotic system while learning and applying fundamental ROS 2, control engineering, simulation, and software architecture concepts.

The robot moves along a one-dimensional axis toward a fixed obstacle.

A simulated distance sensor measures the remaining distance to the obstacle. A controller processes this measurement and generates a velocity command. The simulated robot then integrates that velocity command to update its position.

This creates a complete feedback loop.

## System Architecture

The system contains three main functional components:

- **Simulation** — models the robot and distance sensor.
- **Control** — calculates the robot velocity from the measured distance.
- **Bringup** — launches and configures the complete system.

The components communicate through ROS 2 topics.

```text
                         /distance
┌─────────────────┐ ─────────────────► ┌─────────────────────┐
│ Distance Sensor │                    │ Obstacle Controller │
└────────▲────────┘                    └──────────┬──────────┘
         │                                        │
         │ /robot_position                        │ /cmd_vel
         │                                        ▼
         │                             ┌─────────────────────┐
         └─────────────────────────────│   Simulated Robot   │
                                       └─────────────────────┘
```

This forms the feedback loop:

```text
Robot Position
      │
      ▼
Distance Sensor
      │
      │ /distance
      ▼
Obstacle Controller
      │
      │ /cmd_vel
      ▼
Simulated Robot
      │
      └──────────────► Robot Position
```

## ROS 2 Package Architecture

The project is separated into packages according to responsibility.

```text
ros2_ws/src/
│
├── robot_basics/
│
├── robot_control/
│   └── obstacle_controller.py
│
├── robot_simulation/
│   ├── distance_sensor.py
│   └── simulated_robot.py
│
└── robot_bringup/
    ├── launch/
    │   └── reactive_robot.launch.py
    │
    └── config/
        └── controller.yaml
```

### `robot_control`

Contains the control logic.

Current node:

- `obstacle_controller`

The package is intentionally separated from the simulation implementation so that the controller can eventually operate with another simulator or real hardware without requiring its control logic to be rewritten.

### `robot_simulation`

Contains the simulated physical system.

Current nodes:

- `distance_sensor`
- `simulated_robot`

The package represents the plant and sensor used by the controller.

### `robot_bringup`

Contains system-level launch and configuration resources.

It is responsible for assembling the control and simulation packages into a complete runnable robotic system.

### `robot_basics`

Contains introductory ROS 2 exercises created while learning the ROS communication model, including publisher/subscriber and QoS experiments.

These examples are intentionally kept separate from the application architecture.

## ROS 2 Interfaces

The primary communication interfaces are:

| Topic | Message Type | Publisher | Subscriber |
|---|---|---|---|
| `/distance` | `std_msgs/msg/Float64` | Distance Sensor | Obstacle Controller |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Obstacle Controller | Simulated Robot |
| `/robot_position` | `std_msgs/msg/Float64` | Simulated Robot | Distance Sensor |

These ROS interfaces decouple the individual components.

The controller therefore depends on the interface presented by the system rather than directly depending on the implementation of the simulated robot.

## Closed-Loop Control

The project implements a closed-loop control system.

The controller calculates the distance error:

```text
e = d - d_desired
```

where:

- `d` is the measured distance.
- `d_desired` is the desired stopping distance.
- `e` is the control error.

The proportional controller calculates velocity using:

```text
v = Kp * e
```

where:

- `v` is the commanded linear velocity.
- `Kp` is the proportional gain.

The velocity is then constrained:

```text
0 <= v <= v_max
```

The current controller intentionally does not command reverse motion.

## Deadband / Position Tolerance

Sensor noise can cause small control errors even when the robot is effectively at its desired position.

A tolerance region is therefore applied:

```text
|e| <= tolerance
```

Inside this region:

```text
v = 0
```

This prevents small measurement fluctuations from continuously producing unnecessary velocity commands near the target.

## Controller Evolution

The controller was intentionally developed incrementally.

```text
Threshold / Bang-Bang Control
            │
            ▼
        Hysteresis
            │
            ▼
   Proportional Control
            │
            ▼
    Velocity Saturation
            │
            ▼
     Deadband / Tolerance
            │
            ▼
ROS 2 Parameter Configuration
            │
            ▼
Runtime Parameter Validation
```

This progression demonstrates how a simple reactive behavior can evolve into a more structured feedback controller.

## Hysteresis vs Deadband

Both techniques were explored during development, but they solve different problems.

### Hysteresis

Hysteresis introduces state-dependent switching thresholds.

For example:

```text
MOVING  → STOPPED when distance <= 0.50 m
STOPPED → MOVING  when distance >= 0.60 m
```

The behavior therefore depends on the previous controller state.

### Deadband

Deadband instead defines a tolerance around the desired control error:

```text
|error| <= tolerance
```

It does not require state memory.

The final proportional controller uses a deadband around the desired distance.

## Robot Simulation

The simulated robot represents a simple one-dimensional kinematic plant.

Its state is the robot position:

```text
x
```

The velocity command is obtained from:

```text
/cmd_vel
```

The robot position is updated using:

```text
x[k+1] = x[k] + v[k] * Δt
```

This is a discrete-time approximation of:

```text
dx/dt = v
```

The simulation uses the Forward Euler numerical integration method.

The current simulation period is:

```text
Δt = 0.1 s
```

corresponding approximately to:

```text
10 Hz
```

## Distance Sensor Simulation

A fixed obstacle is placed ahead of the robot.

The true distance is calculated from:

```text
distance = obstacle_position - robot_position
```

The simulated sensor then adds Gaussian measurement noise.

Conceptually:

```text
measured_distance = true_distance + noise
```

The noise follows approximately:

```text
noise ~ N(0, σ²)
```

with a standard deviation of approximately:

```text
σ = 0.02 m
```

The measured distance is prevented from becoming negative.

This separates the concept of **ground truth** from **sensor measurement**, which becomes increasingly important in more realistic robotics simulations.

## ROS 2 Parameters

Controller tuning values are exposed through ROS 2 parameters rather than being permanently hard-coded into the control algorithm.

Current parameters include:

- `desired_distance`
- `kp`
- `max_speed`
- `tolerance`

Default values are stored in:

```text
robot_bringup/config/controller.yaml
```

The configuration hierarchy is:

```text
Python defaults
      │
      ▼
YAML launch configuration
      │
      ▼
Runtime parameter updates
```

Parameters can therefore be inspected and modified without editing the controller source code.

## Runtime Parameter Validation

Runtime parameter updates are validated before being accepted.

For example, values such as the following are rejected:

```text
kp < 0
max_speed < 0
tolerance < 0
desired_distance < 0
```

Accepted parameter updates also update the internal controller state.

This keeps the ROS parameter state and the actual application configuration synchronized.

## Launch System

The complete system can be started through the bringup package:

```bash
ros2 launch robot_bringup reactive_robot.launch.py
```

The launch system starts:

```text
distance_sensor
obstacle_controller
simulated_robot
```

and loads the controller configuration from YAML.

This replaces manually starting each node in separate terminals and provides a reproducible system startup procedure.

## Build and Run

From the ROS 2 workspace:

```bash
cd ~/Projects/robotics/ros2_ws
colcon build
source install/setup.bash
```

Launch the complete system:

```bash
ros2 launch robot_bringup reactive_robot.launch.py
```

## Inspecting the Running System

Useful ROS 2 commands include:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /distance
ros2 topic echo /robot_position
ros2 topic echo /cmd_vel
ros2 topic info /distance
ros2 topic hz /distance
ros2 param list /obstacle_controller
```

Controller parameters can also be inspected at runtime:

```bash
ros2 param get /obstacle_controller kp
ros2 param get /obstacle_controller max_speed
ros2 param get /obstacle_controller desired_distance
ros2 param get /obstacle_controller tolerance
```

## Engineering Concepts Demonstrated

### Robotics and Mechatronics

- Closed-loop feedback control
- Proportional control
- Bang-bang control
- Hysteresis
- Deadband
- Sensor measurement noise
- Gaussian noise
- Kinematic modeling
- Numerical integration
- Sampling frequency
- Actuator saturation
- Plant modeling

### ROS 2

- Nodes
- Publishers and subscribers
- Topics
- ROS message types
- Callbacks
- Executors
- DDS
- RMW
- QoS
- ROS 2 parameters
- Runtime parameter updates
- Launch files
- Package architecture
- Underlay and overlay workspaces
- `colcon`
- `ament_python`

### Software Engineering

- Separation of concerns
- Modular architecture
- Interface-based communication
- Hardware abstraction
- Configuration separated from source code
- Runtime validation
- Defensive programming
- Git version control
- Incremental development
- Meaningful commits
- Reproducible system startup

## Architecture Evolution

The project initially placed the application nodes inside a single learning package.

As the system became more complex, it was refactored into separate packages:

```text
Single learning package
          │
          ▼
┌──────────────────────┐
│    robot_control     │
├──────────────────────┤
│  robot_simulation    │
├──────────────────────┤
│    robot_bringup     │
└──────────────────────┘
```

This creates clearer responsibility boundaries and prepares the architecture for future replacement of the simulated plant with more realistic simulation or physical hardware.

The control package can remain largely independent of how the robot itself is implemented.

## Project Status

**Completed — Project 01**

The completed milestone demonstrates a one-dimensional closed-loop robotic system with simulated sensing, proportional control, runtime configuration, and modular ROS 2 architecture.

The Git history preserves the incremental development process that led to this architecture.

The next project extends these foundations into a two-dimensional mobile robot with planar pose, differential-drive kinematics, coordinate frames, odometry, URDF, TF, visualization, and eventually physics-based simulation.
