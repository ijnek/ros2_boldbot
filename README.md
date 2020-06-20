# BoldBot

This repository provides a suite of packages to work with the BoldBot robot:

* `boldbot_description` - resources to describe the BoldBot,
  specifically as a URDF model. This is used to proide the robot's
  physical properties, enabling performing forward kinematics by TF2,
  visualisation in Rviz2, and simulating the robot in Gazebo.
* `boldbot_sim` - configuration and launch files to bring up Gazebo
  with the boldbot and the basic nodes to interact with it.
* `boldbot_dummy_sensors` - package with simple node to give fake
  sensor output.

## Bring up Simulation

To run a simulation of the robot, build all packages up to
`boldbot_sim` (and source the workspace), and then run the
following:

    ros2 launch boldbot_sim boldbot_sim_bringup.launch.py

This launches Gazebo and spawns the Boldbot model, including a plugin
that provides the topics to interact with it and a robot state
publisher, so you can see it in Rviz.

## Dummy robot

Alternatively, you can launch a dummy boldbot:

	ros2 launch boldbot_bringup boldbot_bringup_dummy.launch.py

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
display type (rviz_default _plugins) > RobotModel`. You then have two options 
to display the robot using the `Displays` tab under `RobotModel`:

  1. - under `Description Source`: choose `Topic`
     - under `Description Topic` select `/robot_description`, or
  2. - under `Description Source`: choose `File` 
     - under `Description File`: select 
       `install/boldbot_description/share/boldbot_description/urdf/boldbot.urdf` 
