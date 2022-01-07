# robot3_ros
ROS

simulate ball catch by 6DOF manipulater

# library
time

numpy

# ROS nodes and topics
![nodes](./nodes.png)

## nodes
#### visualizer
if subscribe /arm_angv, /shooter_state and /ball_initial_state, update /joint_states.

publish /joint_states to update rviz every 0.1(default) seconds.

if subscribe /hand_close, get hand's position and velocity with GetHandState.srv and check that hand can catch ball.

#### arm_controller
if subscribe /hand_close_reserve, get where and when ball is the highest.

check current time is the time ball is the highest every 0.013(default) seconds.


if subscribe /target_arm_states, calculate inverse kinematics and update joints' target angles.

calculate /arm_angles 
