# BoldBot

This repository provides a suite of packages to work with the BoldBot robot:

* `boldbot_description` - resources to describe the BoldBot,
  specifically as a URDF model. This is used to proide the robot's
  physical properties, enabling performing forward kinematics by TF2,
  visualisation in Rviz2, and simulating the robot in Gazebo.
* `boldbot_bringup` - configuration and launch files to bring up the
  basic nodes to interact with the bolbot, or the full stack.
* `boldbot_dummy_sensors` - package with simple node to give fake
  sensor outptut to do some testing without a physical robot (we're
  working towards being able to run a full robot in Gazebo to give a
  nicer test platform).
* `behavior` - collection of behavior nodes.

## Bring up BoldBot

## Real robot

Run the following command on the robot after:

    ros2 launch boldbot_bringup boldbot_bringup.launch.py

This will bring up a node to communicate with the CM730 (and the
motors), and the camera. Any further nodes you can run locally to
communicate through the network, or also on the robot if available.

## Gazebo simulation

To run a simulation of the robot, follow these steps:

1. Set the `GAZEBO_MODEL_PATH` environment variable so Gazebo can find
   all necessary resources:

        export GAZEBO_MODEL_PATH=$(ros2 pkg prefix boldbot_description)/share

2. Run Gazebo with the ROS factory library loaded, which opens
   services to make it possible to load models from ROS:

        gazebo -s libgazebo_ros_factory.so

3. Now spawn the robot in another terminal:

        ros2 run gazebo_ros spawn_entity.py -b \
            -entity boldbot \
            -file $(ros2 pkg prefix boldbot_description)/share/boldbot_description/boldbot.urdf \
            -x 0 -y 0 -z .6

    The `-b` flag keeps the process alive ('bound' to Gazebo) and
    causes the model to be removed when this process is aborted.
    
## Dummy robot

Alternatively, you can launch a dummy boldbot (locally or on the
robot):

`ros2 launch boldbot_bringup boldbot_bringup_dummy.launch.py`

# Check with rviz2

Run `rviz2` to open the Rviz visualisation tool.

## Visualise the transformation

When brought up correctly, you can see the transformation with `Add >
By display type (rviz_default _plugins) > TF`. Switch off `Show Axes`
and `Show Arrows` in the `Display`-view under `TF`. Chose a base_link
in the `Displays`-view under `Global Options > Fixed Frame`. Use
either `torso` or `base_link`.

## Use BoldBot model

To see the actual BoldBot model and not just its links, `Add > By
display type (rviz_default _plugins) > RobotModel`. In the `Displays`
tab then choose the file `boldbot_description/urdf/boldbot.urdf` under
`RobotModel > Description Source`.
