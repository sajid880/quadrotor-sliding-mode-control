ROS2 Drone Control Workspace

A comprehensive ROS2 workspace for autonomous drone control using advanced fractional-order sliding mode control algorithms. This workspace provides a complete system for trajectory generation, position control, and MAVROS-based communication with PX4 autopilots.

OVERVIEW

This workspace implements an Adaptive Fractional-Order Nonsingular Fast Terminal Sliding Mode (AFONFTSM) controller for precise drone position tracking. The system consists of modular packages that work together to generate desired trajectories, compute control commands, and interface with MAVROS for real-time drone control.

The controller is based on fractional calculus principles, utilizing Grunwald-Letnikov (GL) finite-memory approximation for fractional derivative and integral operations. This approach provides enhanced tracking performance and robustness compared to traditional integer-order controllers.

PACKAGES

afonftsm_controller

The core control package implementing the AFONFTSM position controller. This node subscribes to desired trajectory setpoints and current odometry, then publishes velocity commands to MAVROS.

Key Features:
- Fractional-order derivative and integral terms using GL approximation
- Nonsingular fast terminal sliding mode control
- Configurable control parameters for tuning
- Velocity command saturation for safety
- Real-time control loop at configurable rates

Topics:
- Subscribes to /trajectory/desired (geometry_msgs/PoseStamped) for reference positions
- Subscribes to /mavros/local_position/odom (nav_msgs/Odometry) for current state
- Publishes to /mavros/setpoint_velocity/cmd_vel (geometry_msgs/TwistStamped) for velocity commands

Parameters:
- rate: Control loop frequency in Hz (default: 30.0)
- memory_len: Fractional GL memory length in samples (default: 60)
- gamma_d: Fractional derivative order (default: 0.9)
- gamma_i: Fractional integral order (default: 0.1)
- c_d: Coefficient for fractional derivative term (default: 0.18)
- c_i: Coefficient for fractional integral term (default: 0.18)

drone_interface

Interface package for MAVROS communication and offboard mode management. Provides nodes for arming, mode switching, and actuator control.

Components:

offboard_node: Manages offboard mode activation and setpoint publishing
- Automatically arms the drone and switches to OFFBOARD mode
- Publishes position setpoints to /mavros/setpoint_position/local
- Can generate circular trajectories for testing

drone_interface_node: Converts controller commands to actuator control messages
- Subscribes to /controller/commands (std_msgs/Float32MultiArray)
- Publishes to /mavros/actuator_control (mavros_msgs/ActuatorControl)

Launch Files:
- offboard_system.launch.py: Launches the complete system including trajectory generator, controller, and offboard node

trajectory_generator

Generates various 3D trajectory patterns for the drone to follow. Supports multiple trajectory types with configurable parameters.

Trajectory Types:
- helix: Spiral trajectory with circular motion in XY plane and ascending Z
- circle: Circular trajectory at constant height
- line: Linear trajectory in 3D space

Topics:
- Publishes to /trajectory/desired (geometry_msgs/PoseStamped)

Parameters:
- hz: Publication rate in Hz (default: 30.0)
- type: Trajectory type - helix, circle, or line (default: helix)
- radius: Trajectory radius in meters (default: 3.0)
- height: Maximum height for helix trajectory in meters (default: 3.0)
- speed: Trajectory speed parameter (default: 0.5)

mavlink, mavros, mavros_msgs, mavros_examples, mavros_extras

MAVLink and MAVROS packages for communication with PX4 autopilots. These are dependency packages that provide the communication protocol and ROS2 interfaces for drone control.

REQUIREMENTS

System Requirements:
- Ubuntu 20.04 or later (tested on Ubuntu 22.04)
- ROS2 Humble or later
- Python 3.8 or later
- NumPy

Hardware Requirements:
- PX4-compatible autopilot (e.g., Pixhawk series)
- MAVROS connection to autopilot (USB, UART, or network)
- Optional: Gazebo simulator for testing without physical hardware

INSTALLATION

1. Clone or navigate to the workspace directory:
   cd /home/sajeed/ros2_drone_ws

2. Install dependencies:
   sudo apt update
   sudo apt install ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs
   sudo apt install python3-numpy python3-setuptools

3. Build the workspace:
   colcon build --symlink-install

4. Source the workspace:
   source install/setup.bash

Note: Add the source command to your .bashrc for persistent setup:
   echo "source /home/sajeed/ros2_drone_ws/install/setup.bash" >> ~/.bashrc

USAGE

Basic Usage

1. Start MAVROS connection to your autopilot:
   ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14557

   For physical hardware, adjust the fcu_url parameter accordingly (e.g., /dev/ttyUSB0:57600 for USB connection).

2. Launch the complete system:
   ros2 launch drone_interface offboard_system.launch.py

   This will start:
   - Trajectory generator (helix pattern by default)
   - AFONFTSM controller
   - Offboard node for mode management

3. Monitor the system:
   ros2 topic echo /mavros/local_position/odom
   ros2 topic echo /trajectory/desired
   ros2 topic echo /mavros/setpoint_velocity/cmd_vel

Running Individual Nodes

Trajectory Generator:
   ros2 run trajectory_generator trajectory_generator_node --ros-args -p type:=circle -p radius:=2.0

AFONFTSM Controller:
   ros2 run afonftsm_controller afonftsm_controller_node --ros-args -p rate:=50.0 -p gamma_d:=0.95

Offboard Node:
   ros2 run drone_interface offboard_node

Customizing Parameters

You can modify the launch file parameters or pass them via command line:

   ros2 launch drone_interface offboard_system.launch.py \
     trajectory_generator:={
       type:=circle,
       radius:=5.0,
       speed:=0.3
     } \
     afonftsm_controller:={
       rate:=50.0,
       gamma_d:=0.95,
       gamma_i:=0.15,
       v_max:=3.0
     }

CONTROL THEORY

The AFONFTSM controller implements an adaptive fractional-order nonsingular fast terminal sliding mode control strategy. The key components include:

Sliding Surface:
The sliding surface S combines position error, velocity error, fractional derivative, and fractional integral terms:
   S = e2 + c_d * D^gamma_d(e1) + c_i * I^gamma_i(e1)

Where:
- e1 is the position error (current - desired)
- e2 is the velocity error (current - desired)
- D^gamma_d is the fractional derivative of order gamma_d
- I^gamma_i is the fractional integral of order gamma_i
- c_d and c_i are weighting coefficients

Control Law:
The control law uses a terminal sliding mode approach with fractional-order terms:
   v = -k1 * sign(S) * sqrt(|S|) - k2 * S - p_corr * e1

Where k1, k2, and p_corr are control gains.

Fractional Calculus:
Fractional derivatives and integrals are approximated using the Grunwald-Letnikov (GL) method with finite memory:
   D^gamma f(t) ≈ sum_{k=0}^{N} c_k * f(t - k*dt)

The binomial coefficients c_k are precomputed for efficiency.

TUNING GUIDE

Controller Parameters:

rate: Higher rates (50-100 Hz) provide better tracking but require more computational resources. 30 Hz is typically sufficient.

memory_len: Longer memory (60-100 samples) improves fractional approximation accuracy but increases computational cost. Start with 60.

gamma_d: Fractional derivative order. Values between 0.8-0.95 work well. Higher values emphasize recent errors more.

gamma_i: Fractional integral order. Values between 0.05-0.2 are typical. Lower values reduce integral windup.

c_d, c_i: These balance the fractional terms. Start with equal values around 0.15-0.2 and adjust based on tracking performance.

k1, k2: Sliding mode gains. k1 typically ranges from 2.0-3.0, k2 from 0.1-0.3. Increase for faster convergence but watch for oscillations.

v_max: Maximum velocity command. Set based on your drone's capabilities and safety requirements. Typical values: 1.5-3.0 m/s.

Tuning Procedure:
1. Start with default parameters
2. Test with a simple circle trajectory
3. Adjust gamma_d and gamma_i for desired response characteristics
4. Tune k1 and k2 for convergence speed
5. Adjust c_d and c_i to balance tracking accuracy
6. Fine-tune v_max based on physical constraints

TROUBLESHOOTING

Controller not publishing commands:
- Check that odometry is being received: ros2 topic echo /mavros/local_position/odom
- Verify trajectory generator is publishing: ros2 topic echo /trajectory/desired
- Check node logs: ros2 node info /afonftsm_controller

Drone not responding:
- Verify MAVROS connection: ros2 topic list | grep mavros
- Check offboard mode is active: ros2 topic echo /mavros/state
- Ensure drone is armed: ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

Oscillations or instability:
- Reduce control gains (k1, k2)
- Lower v_max
- Increase memory_len for smoother fractional operations
- Check for sensor noise in odometry

Poor tracking performance:
- Increase control rate
- Adjust gamma_d and gamma_i values
- Tune c_d and c_i coefficients
- Verify trajectory generator parameters match desired path

ARCHITECTURE

System Architecture:

The workspace follows a modular architecture with clear separation of concerns:

1. Trajectory Layer: trajectory_generator package generates desired waypoints
2. Control Layer: afonftsm_controller package computes control commands
3. Interface Layer: drone_interface package manages MAVROS communication
4. Communication Layer: mavros packages handle protocol translation

Data Flow:

trajectory_generator → /trajectory/desired → afonftsm_controller
                                                      ↓
/mavros/local_position/odom → afonftsm_controller → /mavros/setpoint_velocity/cmd_vel → MAVROS → PX4

The offboard_node runs in parallel to manage flight mode and arming status.

FILE STRUCTURE

ros2_drone_ws/
├── src/
│   ├── afonftsm_controller/
│   │   ├── afonftsm_controller/
│   │   │   └── afonftsm_controller_node.py
│   │   ├── package.xml
│   │   └── setup.py
│   ├── drone_interface/
│   │   ├── drone_interface/
│   │   │   ├── drone_interface_node.py
│   │   │   └── offboard_node.py
│   │   ├── launch/
│   │   │   └── offboard_system.launch.py
│   │   ├── package.xml
│   │   └── setup.py
│   ├── trajectory_generator/
│   │   ├── trajectory_generator/
│   │   │   └── trajectory_generator_node.py
│   │   ├── package.xml
│   │   └── setup.py
│   └── mavros/ (dependency packages)
├── build/ (build artifacts)
├── install/ (installed packages)
└── log/ (build logs)

DEVELOPMENT

Adding New Trajectory Types:

1. Edit trajectory_generator/trajectory_generator/trajectory_generator_node.py
2. Add new trajectory type in update_traj method
3. Update parameter documentation

Modifying Controller:

1. Edit afonftsm_controller/afonftsm_controller/afonftsm_controller_node.py
2. Adjust control_loop method for algorithm changes
3. Update parameters in package.xml or launch file
4. Rebuild: colcon build --packages-select afonftsm_controller

Testing:

Run linting checks:
   colcon test --packages-select afonftsm_controller
   colcon test-result --verbose

For simulation testing, use Gazebo with PX4 SITL:
   make px4_sitl gazebo
   ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14557

LICENSE

This workspace contains multiple packages with different licenses:
- Custom packages (afonftsm_controller, drone_interface, trajectory_generator): MIT License
- MAVROS packages: Various (BSD, GPLv3, LGPLv3) - see respective package LICENSE files

CONTRIBUTING

When contributing to this project:
1. Follow ROS2 coding standards
2. Use meaningful commit messages
3. Test changes thoroughly before submitting
4. Update documentation for new features
5. Ensure all nodes handle shutdown gracefully

MAINTAINER

Maintained by: sajeed
Email: sjdhsn2396@gmail.com

REFERENCES

The AFONFTSM controller is based on research in adaptive fractional-order sliding mode control. Key concepts include:
- Fractional calculus and Grunwald-Letnikov approximation
- Nonsingular terminal sliding mode control
- Adaptive control for uncertain systems

For MAVROS documentation, visit: https://github.com/mavlink/mavros

VERSION HISTORY

Version 0.0.1:
- Initial implementation of AFONFTSM controller
- Basic trajectory generator with helix, circle, and line patterns
- MAVROS integration for PX4 communication
- Offboard mode management

ACKNOWLEDGMENTS

This project utilizes:
- ROS2 Humble for robotics middleware
- MAVROS for MAVLink communication
- PX4 autopilot firmware
- NumPy for numerical computations

