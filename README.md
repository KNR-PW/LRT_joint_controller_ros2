# joint_controller_ros2

## Introduction

This project provides a `ros2_control` `ChainableControllerInterface` used to simulate PID controller with position, velocity and feedforward effort inputs, like for example mjbots [moteus](https://github.com/mjbots/moteus) controller controller. It can simulate multiple controllers at the same time (for every actuated joint).
Controller is implementing following control law for every joint:
```math
\boldsymbol{\tau} = \boldsymbol{k}_{p-scale} \boldsymbol{k}_p(\boldsymbol{p}_{des} - \boldsymbol{p}) + \boldsymbol{k}_{d-scale} \boldsymbol{k}_d(\boldsymbol{v}_{des} - \boldsymbol{v}) + \boldsymbol{k}_i\int_{0}^{t} (\boldsymbol{p}_{des} - \boldsymbol{p}) \,dt  + \boldsymbol{\tau}_{ff}
```
where:
 - $\boldsymbol{\tau}$: output joint effort
 - $\boldsymbol{p}_{des}$: desired joint posistion
 - $\boldsymbol{p}$: actual joint position
 - $\boldsymbol{v_{des}}$: desired joint velocity
 - $\boldsymbol{v}$: actual joint velocity
 - $\boldsymbol{\tau}_{ff}$: feedforward joint effort
 - $\boldsymbol{k}_p$, $\boldsymbol{k}_d$, $\boldsymbol{k}_i$: PID controller constant parameters
 - $\boldsymbol{k}_{p-scale}$: PID proportional dynamic parameter
 - $\boldsymbol{k}_{d-scale}$: PID derivative dynamic parameter

Reference (input) interfaces:
- $\boldsymbol{p}_{des}$: `position`
- $\boldsymbol{v_{des}}$: `velocity`
- $\boldsymbol{\tau}_{ff}$: `feedforward_torque`
- $\boldsymbol{k}_{p-scale}$: `kp_scale`
- $\boldsymbol{k}_{d-scale}$: `kd_scale`
  
User can choose what interfaces to use.

#### :warning: IMPORTANT: User don't have to use `velocity` interface for velocity part of PID to work, just add non-zero $\boldsymbol{k}_d$, desired velocity will always be zero, which creates virtual damping. It works similary for other interfaces.

### Software supports:
- :ballot_box_with_check: Working independently via subscriber using `joint_controller/JointCommand` in `joint_controller_msgs` package
- :ballot_box_with_check: Wokring in chain mode with other controllers

## ROS 2 version
- Humble (supports `ros2_control` for Rolling)

## Dependencies (all for humble)
- [ros2_control](https://github.com/ros-controls/ros2_control)
- [ros2_controllers](https://github.com/ros-controls/ros2_controllers)
- [generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library)
- sensor_msgs

## Installation 
1. Clone repo to your workspace:
```bash
git clone https://github.com/KNR-PW/LRT_joint_controller_ros2.git
```
2. Install dependencies in workspace:
```bash
rosdep install --ignore-src --from-paths . -y -r
```
3. Build:
```bash
colcon build --packages-select joint_controller_msgs joint_controller_ros2_control
```

### Controller parameters (example):

```yaml
...
joint_controller:
      type: joint_controller/JointController
...
joint_controller:
  ros__parameters:
    joint_names:
      - body_1_joint

    joint_params:
      body_1_joint: 
        position_max: 3.14
        position_min: -3.14
        position_offset: 0.0
        velocity_max: 10.0
        effort_max: 10.0
    
    reference_interfaces: 
      - "position"

    frequency: 500.0

    pid_gains:
      body_1_joint:
        p: 1.0
        d: 0.5
        i: 0.0
        ilimit: 1.0
...
```

#### `joint_names` - names of joints

#### `joint_params` (map for every joint name):
- `position_offset` - Position offset, will be added to commanded position before sending to controller [`radians`]
- `position_max/min` - Max/min position [`radians`]
- `velocity_max` - Maximal velocity [`radians`]
- `effort_max` - Maximal torque [`Nm`]

#### `reference_interfaces` - reference (input) interfaces for user or other controller

#### `command_interface` - :warning: ALWAYS EFFORT, NOTHING ELSE (it is deafult value so user don't have to write it)

#### `state_interface` - :warning: ALWAYS POSITION AND VELOCITY, NOTHING ELSE (it is deafult value so user don't have to write it)

#### `frequency` - frequency for PID controller (only usefull in integral term), that have to be same as real frequency [Hz]

#### `pid_gains` (map for every joint name):
- `p` - proportional PID term
- `d` - derivative PID term
- `i` - integral PID term
- `ilimit` - integration limit [`Nm`]


## Troubleshooting/New functionality
#### Add new `Issue`. I will try my best to answer. You can also contribute to project via pull requests.

## Contributing
1. Change code.
2. Run all launchfile tests. 
3. Check performance, compliance and other functionality via e.g. [plotjugller](https://plotjuggler.io/) for `ros2`.
4. Add clear description what changes you've made in pull request.
