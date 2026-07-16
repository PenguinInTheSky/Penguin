# Deep Dive: Q&A

Notes from building this project — the "why" behind design decisions that aren't obvious just from reading the code. See the [README](../README.md) for setup and usage.

## Robot description (URDF/xacro)

### Why do links need an `<inertial>` block?

A URDF `<link>` has three optional blocks, each for a different consumer:

- `<visual>` — what the renderer (RViz, Gazebo GUI) draws.
- `<collision>` — the geometry the physics engine uses for contact detection.
- `<inertial>` — mass and the 3×3 inertia tensor, used by the dynamics solver.

Omit `<inertial>` and most simulators default the link's mass to 0. A massless link is physically meaningless to the solver: `F = ma` blows up as acceleration approaches infinity, gravity has nothing to act on, and joint damping/PID control becomes unstable. `xacro` macros usually generate this block procedurally (mass + shape → inertia formula) so it doesn't need to be hand-derived per link.

### How does the lidar work?

A Gazebo plugin (`.so` file) is a shared library Gazebo loads and runs *inside its own process* — it is not a ROS node by itself.

1. Gazebo parses `<sensor type="ray">` in the xacro and creates a ray-cast sensor with the configured scan range/resolution.
2. It then `dlopen`s the plugin (`libgazebo_ros_ray_sensor.so`) and calls its init function with the `<ros>`/`<frame_name>` config.
3. On every update, the plugin reads the raw ray-cast distances and publishes them as `sensor_msgs/msg/LaserScan` on `/scan`.

| | Simulation | Real hardware |
|---|---|---|
| Data source | Gazebo plugin (fakes the physics) | Driver node (e.g. `rplidar_ros`) talking to hardware over serial/USB |
| Output | Same `sensor_msgs/LaserScan` on a topic | Same `sensor_msgs/LaserScan` on a topic |

Both converge on the same message type/topic shape, so the rest of the stack (RViz, SLAM, exploration) can't tell the difference.

## ros2_control

### How does the whole ros2_control chain work?

```
/cmd_vel  →  DiffDriveController  →  command interfaces  →  hardware interface (write())  →  actuator (sim or real)
                                                                                                      ↓
/joint_states ←  JointStateBroadcaster ←  state interfaces  ←  hardware interface (read())  ←  encoder/sim state
```

- **`<ros2_control>` block** — declares the abstract contract: which joints exist, and what can be commanded/read on each (e.g. `left_wheel_joint` exposes a `velocity` command interface and `velocity`/`position` state interfaces). This is hardware-agnostic.
- **Hardware interface plugin** (`gazebo_ros2_control/GazeboSystem` here) — the concrete implementation of that contract. Its `write()` takes whatever's in the command interfaces and applies it to the actuator (Gazebo physics, or a real motor driver on hardware); its `read()` pulls actual state back into the state interfaces. Swapping this one plugin line is all that's needed to go from simulation to real hardware.
- **`controller_manager`** — the orchestrator. It loads the hardware interface, loads controllers from a YAML config, and every cycle runs: `read()` → each active controller's `update()` → `write()`. Its `ResourceManager` owns the actual interface registry and guarantees two controllers never claim the same command interface.
- **Gazebo's role**: `libgazebo_ros2_control.so` is a Gazebo plugin that instantiates a real `controller_manager` *inside Gazebo's process*, driven by the simulation step instead of a wall-clock timer — standing in for the standalone `ros2_control_node` you'd run against real hardware.

### What do `DiffDriveController` and `JointStateBroadcaster` actually do?

- **`DiffDriveController`** — subscribes to `cmd_vel_unstamped` (Twist), solves the differential-drive inverse kinematics (using `wheel_separation`/`wheel_radius`) to get per-wheel velocity, writes those to the command interfaces. It also reads wheel state back to compute and publish odometry (`/odom` + the `odom → base_link` TF).
- **`JointStateBroadcaster`** — read-only. Every cycle it reads all available state interfaces and publishes them as `sensor_msgs/JointState` on `/joint_states`. No kinematics, no commanding — purely a state-interfaces-to-topic bridge, consumed by `robot_state_publisher`.

### What does `robot_state_publisher` actually do, and is `JointStateBroadcaster` required?

It parses the URDF once at startup and continuously publishes the TF tree. For **fixed** joints (`lidar_joint`, `base_to_chassis`), the transform is constant and published immediately from the URDF alone — no dependency on `/joint_states`. For **movable** joints (`left_wheel_joint`, `right_wheel_joint`), it needs the live angle from `/joint_states`, which is why it depends on `JointStateBroadcaster`.

Practical implication: dropping `JointStateBroadcaster` only breaks the wheels' *visual* rotation in TF — harmless if nothing is mounted downstream of a wheel and nothing else consumes `/joint_states`. `DiffDriveController` itself never touches `/joint_states`; it reads wheel state directly from the claimed state interfaces.

### How does `joint_state_publisher_gui` know to only show sliders for the wheels?

It's a pure filter on joint `type` in the parsed URDF: `fixed` joints (constant transform, nothing to control) are skipped; `continuous`/`revolute`/`prismatic` joints get a slider. In this robot, only `left_wheel_joint`/`right_wheel_joint` are `type="continuous"` — everything else (`lidar_joint`, casters, chassis mount) is `fixed`.

## Localization & mapping

### How does `slam_toolbox` work?

1. **Input**: `/scan` (from the lidar plugin) + the TF tree (`odom`/`base_link`, provided by `robot_state_publisher` + `DiffDriveController`'s odometry).
2. **Scan matching**: each new scan is compared against the map/recent scans to estimate actual motion, correcting wheel-odometry drift.
3. **Map building**: scan-matched poses are rasterized into an occupancy grid (5cm/cell here).
4. **Loop closure**: when the robot revisits a mapped area, a pose-graph optimization (via Ceres) corrects accumulated drift across the whole trajectory.
5. **Output**: the `map` frame, the `map → odom` TF correction, and the occupancy grid on `/map`.

### Why keep both `odom` and `map` frames instead of just one?

- **`odom → base_link`** (published by `DiffDriveController`): pure wheel-encoder dead reckoning. Smooth and continuous — no jumps — but drifts unboundedly over time/distance with zero self-correction.
- **`map → odom`** (published by `slam_toolbox`/AMCL): corrects that drift by matching live scans against the map. Globally accurate, but can jump discretely whenever a new match/loop-closure lands.

Feeding a jumpy, globally-corrected pose straight into a real-time velocity controller would cause sudden corrective jerks. Keeping the two frames separate means fast local control only ever sees the smooth `odom` half, while anything needing global accuracy (path planning) composes the full `map → odom → base_link` chain via `tf2`. Each node only ever publishes its own link in the chain — `DiffDriveController` has zero knowledge of the map, `slam_toolbox`/AMCL never touches `odom → base_link` directly.

### What does `nav2_map_server` do?

Two complementary tools:

- **`map_saver`** (`map_saver_cli`) — subscribes to the live `/map` topic and serializes it to a `.pgm` (occupancy image) + `.yaml` (resolution, origin, thresholds) pair.
- **`map_server`** — the reverse: loads a saved `.pgm`/`.yaml` pair from disk and republishes it as a live `/map` topic, for AMCL to localize against during autonomous exploration.

## Debugging notes worth remembering

- **WSL2 + `/mnt/c` builds fail with `configure_file: Operation not permitted`.** DrvFs (the Windows-drive mount) doesn't preserve real Linux permissions by default — every file shows up as `root:root` with faked `rwxrwxrwx` bits, which breaks CMake's compiler-detection `chmod` calls. Fix: add `options = "metadata"` to the `[automount]` section of `/etc/wsl.conf`, then `wsl --shutdown` and reopen.
- **`DiffDriveController` doesn't subscribe to plain `/cmd_vel`.** With `use_stamped_vel: false`, it listens on `<controller_name>/cmd_vel_unstamped` (e.g. `diff_cont/cmd_vel_unstamped`) — a relative topic under its own controller name, not the bare topic `teleop_twist_keyboard` publishes to by default. Confirmed via `ros2 topic info /cmd_vel --verbose` showing 0 subscribers. Fixed by remapping: `--ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped`.
- **`ros-desktop`/`controller_manager` don't pull in the actual controller plugins.** `diff_drive_controller` and `joint_state_broadcaster` are separate packages (`ros-humble-ros2-controllers`) from `controller_manager` itself — installing the manager isn't enough; `pluginlib` fails to find the plugin classes at load time otherwise.
