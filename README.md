# TurtlePursuit: An Autonomous Pursuit-Evasion Framework in ROS 2
**Repository:** https://github.com/Shass27/TurtlePursuit

## 1. Abstract
TurtlePursuit is a robotic simulation framework developed within the Robot Operating System 2 (ROS 2) environment. The project simulates a dynamic pursuit-evasion scenario wherein an autonomous agent (the "Hunter") identifies, tracks, and intercepts dynamically generated targets (the "Prey") within a bounded 2D continuous space. This system demonstrates fundamental concepts in autonomous robotics, including node orchestration, coordinate frame transformation, feedback control loops, and service-based interaction for entity management.

## 2. System Architecture
The system is composed of a modular node architecture leveraging the ROS 2 communication graph. The interaction between components is achieved through a combination of asynchronous topics and synchronous service calls.

### 2.1 Component Overview
- `/turtle_spawner`: A stochastic generator that spawns target entities at random coordinates `(x, y, θ)` within the simulation bounds. It maintains a state registry of active targets and broadcasts this manifest to the wider system via custom interfaces.
- `/turtle_controller`: The central processing unit for the autonomous agent. It subscribes to the global target state and the agent's own odometry to compute kinematic control commands at a frequency of 100 Hz.
- `turtlesim`: The simulation backend responsible for rendering the environment and integrating the kinematic equations of motion.

### 2.2 Data Flow Topology

#### Topics

`/turtles`  
Type: `interfaces/msg/TurtleArray`  
Carries the real-time manifest of all active target turtles.

`/turtle1/pose`  
Type: `turtlesim/msg/Pose`  
Provides telemetry (position and orientation) of the Hunter agent.

`/turtle1/cmd_vel`  
Type: `geometry_msgs/msg/Twist`  
Transmits computed velocity commands to the Hunter's actuator interface.

#### Services
- `/spawn`: Invoked by the spawner node to instantiate new targets in the simulator.
- `/kill`: Invoked by the controller node to remove intercepted targets from the simulation.

## 3. Methodology and Control Algorithms
The autonomous behavior is governed by a Nearest-Neighbor selection algorithm coupled with a Proportional (P) Controller for kinematic regulation.

### 3.1 Target Selection Strategy
The agent operates in a continuous control loop where it evaluates the Euclidean distance `d_i` between its current position `(x_h, y_h)` and every `i`-th active target `(x_i, y_i)`:
```
d_i = sqrt((x_i - x_h)^2 + (y_i - y_h)^2)
```
The target `T_target` is selected such that it minimizes the distance metric:
```
T_target = argmin(d_i)
```

### 3.2 Kinematic Control Law
Once a target is locked, the controller generates velocity commands based on the error state.

#### Linear Velocity Control (`v`)
The linear velocity is proportional to the distance error, ensuring the agent decelerates as it approaches the interception point to minimize overshoot.
```
v(t) = K_p_linear · d(t)
```
Parameter: `K_p_linear = 2.0`

#### Angular Velocity Control (`ω`)
The angular velocity is driven by the heading error `α`, defined as the difference between the desired heading angle (towards the target) and the current agent orientation `θ_h`:
```
α = atan2(y_target - y_h, x_target - x_h) - θ_h
```
The error `α` is normalized to the range `[-π, π]` to ensure optimal turning direction. The control law is:
```
ω(t) = K_p_angular · α
```
Parameter: `K_p_angular = 6.0`

### 3.3 Interception Logic
An interception is successfully registered when the distance `d(t)` falls below a critical threshold `ε`:
```
d(t) < 0.1 ⇒ State: Intercepted
```
Upon interception, the agent halts `(v = 0, ω = 0)`, issues a service request to remove the entity from the simulation, and updates the internal state registry to prevent re-engagement.

## 4. System Requirements & Development Environment

### 4.1 Runtime Environment
Operating System: Ubuntu 24.04.3 LTS (Noble Numbat)  
Middleware: ROS 2 Jazzy Jalisco  
Language: Python 3.12+

### 4.2 Development Tools
IDE: Visual Studio Code (VS Code)  
Build System: Colcon (Collective Construction)

## 5. Installation and Execution

### 5.1 Build Procedure
Clone the repository into your ROS 2 workspace `src` directory and build using `colcon`:
```
mkdir -p  ~/ros2_ws/src
git clone https://github.com/Shass27/TurtlePursuit
mv TurtlePursuit/src/* .
rm -rf TurtlePursuit
cd ..
colcon build
source install/setup.bash
```

### 5.2 Usage
The project utilizes a centralized XML launch file to orchestrate the simulation nodes and load configuration parameters:
```
ros2 launch bringup catch_all.launch.xml
```

### 5.3 Configuration
Simulation parameters can be adjusted via `catch_all_config.yaml` or passed as launch arguments:
- `spawn_frequency`: Rate at which new targets appear (Hz)
- `turtle_prefix`: Naming convention for generated targets
