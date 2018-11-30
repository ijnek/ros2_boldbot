# Compile

`colcon build --symlink-install --packages-select boldbot_description boldbot_bringup`


# Bring up BoldBot

`ros2 launch boldbot_bringup boldbot_bringup.launch.py`

# Check with rviz2

When brought up, you can see the TF with `Add > By display type > TF`. In `Displays` you can chose `Global Options > Fixed Frame` either `torso` or `base_link`. 

To see the actual BoldBot model and not just its links, `Add > rviz default plugins > RobotModel`. In the `Displays` tab then choose the file  `boldbot_description/urdf/boldbot.urdf`.
