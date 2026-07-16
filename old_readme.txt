1. Install ROS 2 Humble for Ubuntu Linux 22.04
https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html

2. Build
first install: 
apt install ros-humble-desktop

source ~/ros2_humble/install/setup.bash

if the project in not in wsl, need to turn on automount in wsl config:
Fix — add an [automount] section to /etc/wsl.conf:
sudo tee -a /etc/wsl.conf > /dev/null <<'EOF'

[automount]
options = "metadata,umask=22,fmask=11"
EOF

then run:
colcon build --symlink-install

3. Launch RViz2
source install/setup.bash
sudo apt install ros-humble-xacro
sudo apt install ros-humble-joint-state-publisher-gui
ros2 launch Penguin launch_rviz.launch.py
hardware acceleration stops rviz from working if our system uses the Mesa graphic drivers, to disable hardware use:
export LIBGL_ALWAYS_SOFTWARE = 1

4. Build Gazebo world

5. Launch Gazebo and use slam_toolbox and teleop to produce map
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros2-control

config.mapper_params_online_async.yaml
mode: mapping

sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-nav2-map-server

sudo apt install ros-humble-ros2-controllers

Don't forget to source:
source /opt/ros/humble/setup.bash
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
Run these in parallel:
Gazebo + robot launch: ros2 launch Penguin launch_gazebo_build_map.launch.py
SLAM: ros2 launch slam_toolbox online_async_launch.py params_file:=$(ros2 pkg prefix Penguin)/share/Penguin/config/mapper_params_online_async.yaml
teleop (drive robot around): ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
launch rviz to track map building: rviz2 -d install/Penguin/share/Penguin/config/view_map_build.rviz

Save the map once complete in a new terminal:
ros2 run nav2_map_server map_saver_cli -f small_room_saved

6. Launch Gazebo, use the map, use the algorithm to move robot around the room
sudo apt install ros-humble-nav2-bringup
That package ships localization_launch.py, which is exactly the right granularity for you: it brings up just map_server + amcl + a lifecycle_manager to auto-activate both 

config.mapper_params_online_async.yaml
mode: localisation

two terminals:
Terminal 1: 
ros2 launch Penguin launch_gazebo.launch.py

Terminal 2:  localization (map_server + amcl)
ros2 launch nav2_bringup localization_launch.py \
  map:=$(ros2 pkg prefix Penguin)/share/Penguin/maps/small_room/small_room_saved.yaml \
  use_sim_time:=true

map_server publishes /map, amcl starts publishing /amcl_pose and the map→odom correction




Q&A:

Q1: why do we have to define inertial in xacro for links?

<visual> — what the renderer (RViz, Gazebo GUI) draws.
<collision> — the geometry the physics engine uses for contact/collision detection.
<inertial> — mass and moment-of-inertia (the 3×3 inertia tensor) used by the dynamics solver.

If you omit <inertial>, most simulators (Gazebo, Bullet, DART, etc.) default the link's mass to 0 (or some engines silently substitute a tiny placeholder value). A massless link is physically meaningless to the solver

Q2: how does the lidar work?

A plugin (.so file) is a shared library, Gazebo loads and runs a plugin inside its own process, a plugin is not part of ROS node graph

For the lidar:
Gazebo parses the <gazebo> block in the URDF/xacro and sees <sensor type="ray">, and so it creates a sensor with the defined properties
Gazebo then sees the plugin defined and dynamically loads that library into its own process (using dlopen), then call its init function with config defined in <ros> and <frame_name>
The plugin code reads the Gazebo's sensor data on every update, publish them as sensor_msgs/msg/LaserScan into /scan topic

Contrast this with a real lidar: a separate ROS driver node runs as its own process talking to the hardware over serial/USB, which also publishs LaserScan on a topic.

Simulation: plugin
Real-life: ROS diver node

Q3: how does ros2_control work?

the gazebo plugin initialises a controller_manager, and this reads <ros2_control> and load the hardware interface plugin (which reads and writes to command and state interfaces and translate commands into robot motions), and take claims of the specified command/state interfaces declared per joint, the controller_manager has a resource manager that hold these in its resource registry, the controller_manager reads the yaml files to figure out the controllers that it controls, at eah time fram,e these controllers read/write from/to the state/commands interfaces, does its own thing, in order to power the robot motion, and the controller_manager makes sure that no two controllers write to the same command interface

Q4: what does robot_state_publisher actually do?

it parses your URDF once at startup, and continuously publishes the TF tree — a transform for every link relative to its parent. For fixed joints (like your lidar_joint), the transform is constant and gets published immediately without needing any joint state. For movable joints (left_wheel_joint, right_wheel_joint), it needs the live angle from /joint_states to compute the current transform — which is exactly why it depends on joint_state_broadcaster.

Q5: is JointStateBroadcaster really needed?

Caveat worth flagging though: dropping joint_state_broadcaster entirely means the wheel joints' TF (visual rotation of the wheel links) never publishes — harmless if nothing hangs off the wheels in your tree (yours don't — wheels are leaf links) and nothing else subscribes to /joint_states. But it's a real gap if you ever add anything downstream of a wheel, or if some other tool/diagnostic node expects /joint_states to include wheel data. diff_drive_controller itself doesn't care either way — as established, it reads wheel state straight from the claimed state interfaces, not via the /joint_states topic.

q6: what is teleop used for?

it is used for manual control of the robot once the world model is set up, so that it can go around the room , use slam to build a map

Q7: How does slam actually work?

Input: subscribes to /scan (your LaserScan from the lidar Gazebo plugin, per scan_topic: /scan) and the TF tree (odom_frame: odom, base_frame: base_footprint) that robot_state_publisher + diff_cont's odometry provide.

Scan matching: each new laser scan is compared against the existing map (or recent scans) to figure out how much the robot has actually moved, correcting for wheel-odometry drift. This uses the "Correlation Parameters" in your config — correlation_search_space_dimension/resolution define how far/finely it searches for the best-fit alignment.

Map building: as scan-matched poses accumulate, it rasterizes them into an occupancy grid (resolution: 0.05 → 5cm per cell) — the classic black/white/gray map (occupied/free/unknown).

Pose graph + loop closure: it maintains a graph of robot poses linked by relative constraints from scan matches. When the robot revisits a previously-mapped area (do_loop_closing: true), it detects the match (loop_search_maximum_distance: 3.0) and runs a graph optimization (via Ceres) to correct accumulated drift across the whole trajectory — this is what prevents the map from becoming a warped mess on long runs.

Output: publishes the map frame and the map → odom TF correction (filling in the top of the TF chain we discussed: map → odom → base_link → lidar), and the occupancy grid on /map.

Q8: why both odom and map?
Why both are kept, not just one:
Local controllers / obstacle avoidance that need smooth, jitter-free, low-latency pose use odom — a discrete correction jump mid-motion would be disruptive.

Long-term navigation goals, global path planning, and "where is the robot really in this building" use map, since odom alone would have the robot's belief silently diverging from reality over a long run.

diff_cont owns and publishes odom → base_link — computed purely from wheel encoder dead reckoning, updated every control cycle (100Hz), smooth but drifting.

slam_toolbox owns and publishes map → odom — computed by matching /scan against the map, updated whenever a scan match lands, jumpy but globally drift-corrected.

Chain them and you get the full pose: map → odom → base_link — a pose that's globally accurate (thanks to the map→odom correction) while still being smooth for control purposes (thanks to odom→base_link never jumping). Each node only ever touches its own link in the chain — diff_cont has zero knowledge of the map, slam_toolbox never touches odom→base_link directly, it only corrects the frame above it. That separation is exactly what makes the REP 105 convention work — every package publishes one link, tf2 composes the whole chain for anyone downstream (like nav2) that needs the full base_link pose in map frame.

Q9: how do joint_state_publisher know to just show control for left and right wheels?

Confirmed straight from robot_core.xacro — it's simply filtering by joint type:

left_wheel_joint   type="continuous"
right_wheel_joint  type="continuous"

footprint_joint          type="fixed"
base_to_chassis          type="fixed"

Q10: what does nav2_map_server do?

nav2_map_server is a small package with two complementary tools — you're only using half of it right now:

map_saver (what we're using — map_saver_cli): subscribes to the live /map topic that slam_toolbox is publishing while you drive around, and serializes it to disk as a .pgm (a grayscale image — white=free, black=occupied, gray=unknown) + a .yaml sidecar file (metadata: resolution, origin coordinates, occupancy thresholds). That's the format your existing maps/small_room/small_room_saved.pgm/.yaml are in.

map_server (the other half, not used yet — this is Phase 2, per what you described earlier): does the reverse — loads a saved .pgm/.yaml pair from disk and republishes it as a live /map topic. This is what you'd run instead of slam_toolbox once you're navigating with a pre-built map: map_server provides the static /map, and AMCL (or slam_toolbox in mode: localches live scans against itto localize, rather than building a new map.