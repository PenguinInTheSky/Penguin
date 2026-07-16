# Deep Dive: Q&A

Notes from building this project, in Q&A form. See the [README](../README.md) for setup and usage.

## Q1: Why do links need an `<inertial>` block defined in xacro?

A URDF `<link>` has three optional blocks, each read by a different consumer:

- `<visual>` — what the renderer (RViz, Gazebo GUI) draws.
- `<collision>` — the geometry the physics engine uses for contact detection.
- `<inertial>` — mass and the 3×3 inertia tensor, used by the dynamics solver.

Omit `<inertial>` and most simulators (Gazebo, Bullet, DART) default the link's mass to 0. A massless link is physically meaningless to the solver: `F = ma` blows up as acceleration approaches infinity, gravity has nothing to act on, and joint damping/PID control becomes unstable. In practice, `xacro` macros generate this block procedurally from mass + shape, so it doesn't have to be hand-derived for every link.

## Q2: How does the simulated lidar actually work?

A Gazebo plugin (a `.so` shared library) is loaded and run *inside Gazebo's own process* — it is not a node in the ROS graph by itself.

For this robot's lidar specifically:

1. Gazebo parses the `<gazebo>` block in the xacro, sees `<sensor type="ray">`, and creates a ray-cast sensor with the configured scan range/resolution.
2. It then `dlopen`s the plugin and calls its init function, passing the config from `<ros>` and `<frame_name>`.
3. On every simulation update, the plugin reads the sensor's raw ray-cast distances and publishes them as `sensor_msgs/msg/LaserScan` on `/scan`.

Contrast this with a real lidar: a separate ROS driver node runs as its own process, talking to the hardware over serial/USB, and also publishes `LaserScan` on a topic.

| | Simulation | Real hardware |
|---|---|---|
| Data source | Gazebo plugin | ROS driver node |

Both converge on the same message type and topic shape, so nothing downstream (RViz, SLAM, the exploration node) can tell the difference.

## Q3: How does `ros2_control` work end-to-end?

The Gazebo plugin (`libgazebo_ros2_control.so`) initializes a `controller_manager` inside Gazebo's process. That `controller_manager`:

1. Reads the `<ros2_control>` block in the URDF and loads the hardware interface plugin it specifies (`gazebo_ros2_control/GazeboSystem` here). This plugin reads and writes the command/state interfaces declared per joint, and is what actually translates commands into robot motion (or, on real hardware, into motor signals).
2. Claims the specific command/state interfaces declared per joint (e.g. `left_wheel_joint`'s `velocity` command interface). A `ResourceManager` inside the `controller_manager` holds all of these in a central registry.
3. Reads a YAML config file to know which controllers it should load (e.g. `DiffDriveController`, `JointStateBroadcaster`).
4. Every control cycle, each active controller reads and writes its assigned interfaces from/to that registry to do its job — computing wheel velocities, broadcasting joint state, etc.
5. The `ResourceManager` guarantees no two controllers ever write to the same command interface at once.

## Q4: What does `robot_state_publisher` actually do?

It parses the URDF once at startup and continuously publishes the TF tree — a transform for every link relative to its parent.

For **fixed** joints (like `lidar_joint`), the transform is constant, so it gets published immediately without needing any joint state. For **movable** joints (`left_wheel_joint`, `right_wheel_joint`), it needs the live angle from `/joint_states` to compute the current transform — which is exactly why it depends on `JointStateBroadcaster` being active.

## Q5: Is `JointStateBroadcaster` actually needed?

Dropping `JointStateBroadcaster` entirely means the wheel joints' TF (the visual rotation of the wheel links) never publishes. That's harmless if nothing is mounted downstream of the wheels in the link tree (nothing is, here — the wheels are leaf links) and nothing else subscribes to `/joint_states`. But it would be a real gap if anything were ever added downstream of a wheel, or if some other tool/diagnostic expected `/joint_states` to include wheel data.

`DiffDriveController` itself doesn't care either way: it reads wheel state directly from the claimed state interfaces, not via the `/joint_states` topic.

## Q6: What is teleop used for?

Manual control of the robot once the world model is set up — driving it around the room so `slam_toolbox` can build a map from the lidar scans as it goes. It's a stand-in for whatever eventually sends velocity commands autonomously (in this project, the custom exploration node in Phase 2).

## Q7: How does SLAM (`slam_toolbox`) actually work?

- **Input**: subscribes to `/scan` (the `LaserScan` from the lidar Gazebo plugin) and the TF tree (`odom`/`base_link`, provided by `robot_state_publisher` + `DiffDriveController`'s odometry).
- **Scan matching**: each new laser scan is compared against the existing map (or recent scans) to figure out how much the robot has actually moved, correcting for wheel-odometry drift. The correlation search-space parameters control how far/finely it searches for the best-fit alignment.
- **Map building**: as scan-matched poses accumulate, they're rasterized into an occupancy grid (5cm per cell here) — the classic black/white/gray map of occupied/free/unknown cells.
- **Pose graph + loop closure**: a graph of robot poses linked by relative constraints from scan matches is maintained. When the robot revisits a previously-mapped area, the match is detected and a graph optimization (via Ceres) corrects accumulated drift across the whole trajectory — this is what stops the map from becoming a warped mess on long runs.
- **Output**: the `map` frame, the `map → odom` TF correction (the top of the chain `map → odom → base_link → lidar`), and the occupancy grid on `/map`.

## Q8: Why do we need both an `odom` and a `map` frame?

- **`odom → base_link`**, published by `DiffDriveController`: pure wheel-encoder dead reckoning, computed every control cycle (100Hz). Smooth and jitter-free, but drifts unboundedly over time with nothing to self-correct it.
- **`map → odom`**, published by `slam_toolbox`: computed by matching `/scan` against the map, updated whenever a scan match lands. Globally drift-corrected, but can jump discretely.

Both are kept rather than just one because they serve different consumers: local controllers and obstacle avoidance need a smooth, jitter-free, low-latency pose — a discrete correction jump mid-motion would be disruptive, so they use `odom`. Long-term navigation goals and global path planning need to know where the robot really is in the building, which `odom` alone can't guarantee since its belief silently diverges from reality over a long run — so they use `map`.

Chaining them gives the full pose `map → odom → base_link`: globally accurate (thanks to the `map → odom` correction) while still smooth for control purposes (thanks to `odom → base_link` never jumping). Each node only ever touches its own link in the chain — `DiffDriveController` has zero knowledge of the map, and `slam_toolbox` never touches `odom → base_link` directly, it only corrects the frame above it. That separation is what makes the REP 105 convention work: every package publishes one link, and `tf2` composes the whole chain for anyone downstream (like the exploration node) that needs the full `base_link` pose in the `map` frame.

## Q9: How does `joint_state_publisher_gui` know to only show sliders for the two wheels?

It's simply filtering by joint `type`, straight from `robot_core.xacro`:

```
left_wheel_joint   type="continuous"
right_wheel_joint  type="continuous"

footprint_joint    type="fixed"
base_to_chassis    type="fixed"
```

`fixed` joints have a permanently constant transform, so there's nothing to control and no slider is generated. `continuous`/`revolute`/`prismatic` joints get one. In this robot, only the two wheel joints are non-fixed.

## Q10: What does `nav2_map_server` do?

It's a small package with two complementary tools:

- **`map_saver`** (used via `map_saver_cli`): subscribes to the live `/map` topic that `slam_toolbox` publishes while driving around, and serializes it to disk as a `.pgm` (a grayscale image — white=free, black=occupied, gray=unknown) plus a `.yaml` sidecar file (resolution, origin coordinates, occupancy thresholds). That's the format `maps/small_room/small_room_saved.pgm`/`.yaml` are in.
- **`map_server`** (the other half, used in Phase 2): does the reverse — loads a saved `.pgm`/`.yaml` pair from disk and republishes it as a live `/map` topic. This is what runs instead of `slam_toolbox` once navigating with a pre-built map: `map_server` provides the static `/map`, and AMCL matches live scans against it to localize, rather than building a new map.
