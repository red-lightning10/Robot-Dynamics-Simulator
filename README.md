# Robot-Dynamics-Simulator README

## Overview
This document outlines the steps and functions used in the Robot-Dynamics-Simulator, a tool for simulating robot dynamics through a graphical user interface (GUI).

## Getting Started
### Steps to Run the Simulator
1. **Start the GUI**: Execute the `GUI.m` file to launch the GUI, which is essential for visualizing robot dynamics.
![Robot Dynamics Simulation](Capture.JPG)
2. Enter a value in any of the field and press `enter` in keyboard and click `animate Robot` button to visulize the movement and profile graphs. Make sure you press Enter right after filling any field so as to register them.


### Functions within `GUI.m`
- **`animate_robot()`**: Initiates the forward dynamics and calculates the required joint motor torques to enable the arm to reach desired configuration/keep the arm steady in current configuration.
- **`externalForceCallback(src, event)`**: Detects user-inputted force and updates the `Ftip` value accordingly.
- **`externalPathCallback(src, event)`**: Captures user-inputted path points and updates the path value.

## Functions in Other Files

### `movementDynamics()`
- **Function**: `[tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc] = movementDynamics(currentQ, curr_vel, curr_acc, S, M, Mlist, Glist, g, n, waypoints, Ftip)`
- **Description**: Calculates the required trajectories and torques to move the robot to desired configuration based on its current state.
- **Inputs**:
  - `Mlist`, `Glist`, `S`, `M`, `waypoints`, `Ftip` - Various robot parameters and dynamics inputs.
  - `g` - Gravitational force vector.
  - `currentQ`, `curr_vel`, `curr_acc` - Current state of the robot.
- **Outputs**:
  - `tau_acc` - Accumulated torque.
  - `jointPos_acc` - Joint positions.
  - `jointVel_acc` - Joint velocities.
  - `jointAcl_acc` - Joint accelerations.
  - `t_acc` - Time intervals.

### `gravity_comp()`
- **Function**: `[tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc] = gravity_comp(curr_Q, S, M, Mlist, Glist, g, n, Ftip)`
- **Description**: Calculates the required torques to hold the robot on standby in its current state.
- **Inputs**:
  - `Mlist`, `Glist`, `S`, `M`, `waypoints`, `Ftip` - Various robot parameters and dynamics inputs.
  - `g` - Gravitational force vector.
- **Outputs**:
  - `tau_acc` - Torque.
  - `jointPos_acc` - Joint positions.
  - `jointVel_acc` - Joint velocities.
  - `jointAcl_acc` - Joint accelerations.
  - `t_acc` - Time intervals.

## Sample Inputs
- 
