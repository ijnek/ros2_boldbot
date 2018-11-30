# Compile

`colcon build --symlink-install --packages-select boldbot_description boldbot_bringup`


# Bring up BoldBot

`ros2 launch boldbot_bringup boldbot_bringup.launch.py`

# Check with rviz2

When brought up correctly, you can see the transformation with `Add > By display type (rviz_default _plugins) > TF`. Switch off `Show Axes` and `Show Arrows` in the `Display`-view under `TF`. Chose a base_link in the `Displays`-view under `Global Options > Fixed Frame`. Use either `torso` or `base_link`. 

To see the actual BoldBot model and not just its links, `Add > By display type (rviz_default _plugins) > RobotModel`. In the `Displays` tab then choose the file  `boldbot_description/urdf/boldbot.urdf` under `RobotModel > Description Source`.
