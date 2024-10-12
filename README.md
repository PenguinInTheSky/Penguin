# Penguin
Install colcon

Clone project into for local directory

In the directory, build the project:
colcon build --symlink-install

Source it:
source install/setup.bash

Start map_server node to load the map of the room:
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=./Penguin/maps/small_room/small_room_saved.yaml use_sim_time:=true

Start amcl node to read the map of the room and publish robot's pose:
ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true

Bringup the lifecycles of the twon nodes above:
ros2 run nav2_util lifecycle_bringup map_server
ros2 run nav2_util lifecycle_bringup amcl

Launch the simulation:
ros2 launch Penguin launch_gazebo.launch.py world:=./Penguin/worlds/small_room.world

Watch the robot traversing the floor!

Lu
