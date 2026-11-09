# slam_nav: SLAM and navigation in the worlds of day 7
---
Day 7 of the robotics course. The small simulator of day 6 (`mcl_room`) replaces Gazebo,
SLAM and navigation are the real ROS nodes: SLAM Toolbox draws the map, Nav2 drives.
No GPU, starts in a few seconds.

```
/cmd_vel (Nav2 or teleop) --> drive_sim --tf odom->base_footprint--> slam_toolbox
                                  |                                   ^     |
                                  +--/sim/true_pose--> laser_sim --/scan    | /map, tf map->odom
                                                                            v
                                          tour --navigate_to_pose--> Nav2 --> RViz
```

| Node | Package | Job |
|---|---|---|
| `drive_sim` | slam_nav | true motion of the robot, drifting odometry, tf `odom -> base_footprint` |
| `laser_sim` | slam_nav | 2D lidar: rays from the true pose against the walls, `/scan` |
| `robot_viz` | slam_nav | draws the true walls and the robot like on day 6, only for looking |
| `tour` | slam_nav | sends Nav2 one goal after another, the same route in every run |
| `base_scan_tf` | tf2_ros | static tf `base_footprint -> base_scan` |
| `slam_toolbox` | slam_toolbox | SLAM, synchronous like Nav2's `slam:=True`, tf `map -> odom` |
| `controller_server`, `planner_server`, `behavior_server`, `bt_navigator` | Nav2 | navigation with the default behavior tree, NavFn and MPPI |
| `map_server`, `amcl` | Nav2 | only in `nav.launch.py`: the saved map and AMCL of day 6 |
| `lifecycle_manager_*` | nav2_lifecycle_manager | activate SLAM, localization and navigation |

Two worlds, both in `slam_nav/worlds.py`. Simulator, laser and drawing all use them.

* `loop`: the square corridor of slide 8, 4 m sides on the centre line, 1.1 m wide.
* `room`: the room of day 6 (`mcl_room`) with the inner wall and the box.

There is no map file of the worlds. The robot draws it itself.

---
## Setup

Create virtual Environment:
```bash
python3 -m venv .venv --system-site-packages
```
No extra pip packages are needed, everything comes from ROS.

Needed ROS packages (Nav2 is already there from day 6):
```bash
sudo apt install ros-kilted-navigation2 ros-kilted-slam-toolbox ros-kilted-teleop-twist-keyboard
```

---
## Build
1. Source Ros:
```bash
source /opt/ros/kilted/setup.bash
```
2. Build with:
```bash
source .venv/bin/activate
python3 -m colcon build
source install/setup.bash
```
Build again after changing a file in `config/`, the launch files use the installed copy.

---
## 1. Map a world with SLAM (slides 11 and 12)

```bash
ros2 launch slam_nav slam.launch.py                  # loop of slide 8
ros2 launch slam_nav slam.launch.py world:=room      # room of day 6
```
No 2D Pose Estimate needed. The map starts where the robot stands. Nav2 runs too, so you can
explore with **2D Goal Pose** in RViz: click into the grey area just beyond the known part.
Nav2 plans through unknown cells (`allow_unknown: true`) and the map grows while the robot drives.

Or let the tour drive, the same route every time (second terminal, source both setups there too):
```bash
ros2 run slam_nav tour --ros-args -p world:=loop     # once around the loop and 2 m further
ros2 run slam_nav tour --ros-args -p world:=room     # through both halves of the room
```
Or drive by hand. Hold a key to drive, release to stop:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
`i` forward, `,` back, `j` / `l` turn, `k` stop, `x` / `c` slower.

Watch the correction of SLAM (slide 11). It grows slowly and jumps when a loop closes:
```bash
ros2 run tf2_ros tf2_echo map odom
```
Save the map (slide 13):
```bash
ros2 run nav2_map_server map_saver_cli -f ~/loop_map
```
Writes `loop_map.pgm` and `loop_map.yaml` into your home folder. If it fails with
"Failed to spin map subscription", give it more time: add `--ros-args -p save_map_timeout:=10.0`.

---
## 2. Navigate on the saved map (slide 23)

```bash
ros2 launch slam_nav nav.launch.py world:=loop                            # map shipped in maps/
ros2 launch slam_nav nav.launch.py world:=loop map:=$HOME/loop_map.yaml   # your own map
```
The world must be the one you mapped. AMCL starts at the origin of the map, which is where the
robot started while mapping, so no 2D Pose Estimate is needed. Then send a goal with
**2D Goal Pose**, the tour, or from the terminal:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: -4.0}, orientation: {w: 1.0}}}}"
```
`x: 2.0, y: -4.0` is in the bottom corridor of the loop. **Publish Point** in RViz shows the
coordinates of any point you click.

---
## In RViz

| What | Meaning |
|---|---|
| grey robot | where the robot really is (only the simulator knows this) |
| blue robot | where the robot believes it is: tf `map -> odom -> base_footprint` |
| blue beams | the scan, drawn from the believed pose. Right estimate: beams end on the walls |
| thin dark lines | the true walls. The map of SLAM should lie exactly under them |
| black, white, grey cells | the map: occupied, free, unknown |
| red dots, blue lines | SLAM pose graph: one node per scan, one line per constraint (slide 9) |
| orange line | the global plan of NavFn |
| coloured square | the local costmap around the robot |
| Global costmap | off by default, switch it on to see the inflation (slide 15) |
| Particles (AMCL) | only with `nav.launch.py` |

The red dots and blue lines come from the topic `/slam_toolbox/graph_visualization`. A loop
closure is a blue line between the newest node and an old node far back in the chain.

---
## Experiments for the exercise

1. **Loop closure** (slides 8 and 25). With the default lidar (3.5 m, 2 cm noise) scan matching
   is so good that the loop closure has almost nothing left to correct. A shorter and noisier lidar
   makes the drift visible. Map the loop twice, each time with a fresh launch, and run the tour:
   ```bash
   ros2 launch slam_nav slam.launch.py range_max:=2.0 noise_std:=0.05 loop_closing:=false
   ros2 launch slam_nav slam.launch.py range_max:=2.0 noise_std:=0.05 loop_closing:=true
   ros2 run slam_nav tour
   ```
   Without loop closure the map comes out slightly rotated, walls near the start often appear
   twice, and at the end the blue robot is 20 to 40 cm away from the grey one. With loop closure
   a blue line links the newest node with the first ones when the robot is back at the start, the
   map jumps into place, every wall appears once and the robot ends about 10 cm away. Watch
   `tf2_echo map odom` at that moment.
2. **Featureless corridor** (slide 24). Look at the map of experiment 1 next to the true walls:
   the corridors are too short. In the middle of a corridor a 2 m lidar sees only two parallel
   walls, so scan matching cannot tell how far the robot went.
3. **A wrong loop closure** (slide 8). In `config/slam_toolbox.yaml` set
   `loop_search_space_dimension` back to the original 8.0, build again and map the room with the
   tour a few times. In one of our test runs SLAM matched the right half of the room with the left
   half, the blue robot jumped 2.5 m, and the tour stopped.
4. **Worse odometry** (day 6). More drift, then watch SLAM correct it:
   ```bash
   ros2 param set /drive_sim odom_heading_bias 0.1
   ```
5. **Other planners** (slide 21). In `config/nav2.yaml` set the `plugin` of `GridBased` to
   `nav2_smac_planner::SmacPlanner2D` or `nav2_theta_star_planner::ThetaStarPlanner`, build again
   and compare the orange paths. Some planners have their own parameters, see the Nav2
   configuration guide.
6. **Inflation** (slides 15 and 24). In the room the gaps between the box and the walls are only
   0.6 m wide. Switch on the Global costmap display and send a goal into the corner behind the
   box. Does NavFn go through a gap or around the box? Then change `robot_radius` (0.17 m) or
   `inflation_radius` (0.70 m) in `config/nav2.yaml`, build again and try once more.

Tested headless with ROS 2 Kilted, SLAM Toolbox 2.9 and Nav2 1.4. With the default lidar SLAM
stays within about 5 cm of the true pose in the loop and 2 cm in the room. Experiment 1 ended
21 and 43 cm off without loop closure, 8 and 11 cm off with it. Navigation with AMCL on the
shipped maps: about 5 cm error, the tour takes 50 s in the loop and 70 s in the room.

---
## Parameters

`drive_sim`: `world` (loop), `cmd_timeout` (0.6 s), `max_linear` (0.5 m/s), `max_angular`
(1.5 rad/s), `odom_scale_error` (0.02), `odom_heading_bias` (0.03 rad/m), `odom_noise_trans`
(0.05), `odom_noise_rot` (0.05), `stamped` (false, true for TwistStamped).

`laser_sim`: `world` (loop), `beams` (360), `fov_deg` (360), `rate` (10 Hz), `range_max`
(3.5 m), `noise_std` (0.02 m).

`tour`: `world` (loop), `laps` (1).

Launch arguments of `slam.launch.py`: `world`, `loop_closing` (true), `range_max` (3.5),
`noise_std` (0.02), `nav` (true, false starts SLAM without Nav2), `rviz` (true).
`nav.launch.py`: `world`, `map`, `range_max`, `noise_std`, `rviz`.

What changed against the original files of SLAM Toolbox and Nav2 is listed at the top of
`config/slam_toolbox.yaml` and `config/nav2.yaml`.

Recorded your own map and want to ship it? Copy `loop_map.pgm` and `loop_map.yaml` to
`maps/loop.pgm` and `maps/loop.yaml`, change `image:` in the yaml to `loop.pgm` and build again.
