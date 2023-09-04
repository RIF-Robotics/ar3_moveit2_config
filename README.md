# MoveIt2 Configuration for AR3 Robot Arm

## AR3 MoveIt2 Demo

In this demo configuration, the AR3 will be launched using `ros2_control`'s
`fake_components/GenericSystem` joint loopback simulation. Then, MoveIt2 will
be launched to plan trajectories and send them to the simulated AR3.

1. Start MoveIt2 with the AR3 simulation

        ros2 launch ar3_moveit2_config moveit.launch.py use_fake_hardware:=True

3. In the RVIZ MotionPlanning panel, change the Goal state to "shutdown" and
   click on "Plan & Execute".
