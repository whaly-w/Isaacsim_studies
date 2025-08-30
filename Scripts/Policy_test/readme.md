# H1 Examples
## Model + Policy import
- ```h1_test.py``` is an example which deploys h1 with built-in rl model
- ```h1_imu.py``` attaches imu sensor to the robot **imu_link**
## ROS2 Implementation
- ```h1_ROS2.py``` publish imu value and listen for command from **/cmd_vel**, this can be use with **h1_pkg/** and **h1_msgs/**
## Custom Policy
### Overview
- ```h1_policy.usd``` uses ROS2 to load and run external rl model 
- the robot's joint properties (stiffness, damping, effort limits, velocity limits) and nvironment in the simulation (physics step) have to be config according to **policy/h1_env.yaml**
- the model file is **h1_locomotion_policy/policy/h1_policy.pt**
- the robot state & imu is published as ROS messages: **/joint_states** and **/imu**
- the joint command is subscribed from **/joint_commands**
- to run this example uses the files in **h1_locomotion_policy/**
### Implementation
to run the model use
```bash
ros2 launch h1_locomotion_policy h1_fullbody_controller.launch.py
```
for robot control use (this would publish the command to **/cmd_vel**)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
