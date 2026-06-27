# Husky A200 — Nav2 Route Following

Clearpath Husky A200 (`a200_1103`) following predefined GeoJSON route graphs
using the Nav2 Route Server (`nav2_route`) under ROS 2 Jazzy / Ubuntu 24.04,
in Gazebo simulation.

## Status

- Localization (AMCL `map -> odom` TF) — working
- Route graph loading & path computation — working
- Path execution (route-to-controller bridge) — working
- Multi-lap / arbitrary-loop evaluation harness — working
- `fullLoopRoute1.geojson`, `fullLoopRoute2.geojson` — format verification pending

## Prerequisites

- Ubuntu 24.04, ROS 2 Jazzy
- `ros-jazzy-navigation2`, `ros-jazzy-nav2-route`, `ros-jazzy-nav2-rviz-plugins`
- Clearpath A200 stack (`ros-jazzy-clearpath-*`, `clearpath_nav2_demos`, `clearpath_gz`, `clearpath_viz`)

## Repository structure
├── nav2_custom.yaml              # Full nav2 params (controller, costmaps, route_server, etc.)

├── nav2_custom.launch.py         # Launches nav2 with nav2_custom.yaml

├── localization_custom.yaml      # AMCL params, includes the set_initial_pose fix

├── localization_custom.launch.py # Launches localization with localization_custom.yaml

└── route/

├── testroute1.geojson        # 4-node square loop — verified, safe to use

├── fullLoopRoute1.geojson    # 9-node loop — verification pending

├── fullLoopRoute2.geojson    # 9-node loop + 3 extra edges (8->0, 8->1, 8->5) — verification pending

├── route_loop_eval.py        # Evaluation harness — drives any loop, N laps, logs everything

├── analyze_laps.py           # Computes tracking / cross-track / corner / velocity / success metrics

├── drive_route.py            # One-shot single-edge drive (manual testing / demo)

├── route_server_params.yaml  # legacy — only used by deprecated route_server.launch.py

├── send_route_goal.py        # legacy — computes/tracks a route but does NOT drive the robot

├── route_eval.py             # legacy — superseded by route_loop_eval.py

├── analyze_runs.py           # legacy — superseded by analyze_laps.py

├── route_server.launch.py    # deprecated — standalone route_server, duplicate-node conflict with nav2

└── nav2_with_route.launch.py # deprecated — same duplicate-node conflict
## Why two custom launch wrappers?

The stock `clearpath_nav2_demos` launch files hardcode their own parameter
files internally and don't expose a `params_file` override. `nav2_custom.launch.py`
and `localization_custom.launch.py` wrap the same underlying `nav2_bringup`
launch files but explicitly inject `nav2_custom.yaml` / `localization_custom.yaml`
instead.

## Root causes fixed (background)

1. **AMCL never published `map -> odom` TF.** It requires a completed
   particle-filter update, which a sub-millisecond scan/TF timestamp mismatch
   was blocking indefinitely. Fix: `set_initial_pose: true` in
   `localization_custom.yaml`, which publishes the transform immediately on
   startup instead of waiting on that cycle.
2. **Nothing was calling `FollowPath`.** The Route Server's
   `ComputeAndTrackRoute` action only computes and tracks a route — it never
   drives the robot. `drive_route.py` / `route_loop_eval.py` bridge this by
   forwarding the computed path from the route server's feedback straight
   into `controller_server`'s `FollowPath` action.

## GeoJSON graph format requirements

The graph-authoring tool used to generate these files exports two things
`nav2_route`'s `GeoJsonGraphFileLoader` cannot handle — both cause it to hang
indefinitely with no error message:

- A `crs` block (`EPSG::3857`) — must be removed entirely.
- Empty `MultiLineString` edge geometry — must contain the real
  `[[start_x, start_y], [end_x, end_y]]` coordinate pair.

Check any new graph file before loading it:
```bash
grep -n "crs\|EPSG" route/your_graph.geojson
grep -A2 "MultiLineString" route/your_graph.geojson | head -20
```
If the CRS block is present, or the `coordinates` array under
`MultiLineString` is empty, regenerate the file before use.

## Launch sequence

```bash
# T1 - Gazebo
source ~/clearpath/setup.bash
ros2 launch clearpath_gz simulation.launch.py

# T2 - Localization (custom, AMCL fix included)
source ~/clearpath/setup.bash
ros2 launch ~/clearpath/localization_custom.launch.py \
  map:=$HOME/clearpath/husky_map.yaml use_sim_time:=true

# T3 - RViz
source ~/clearpath/setup.bash
ros2 launch clearpath_viz view_navigation.launch.py namespace:=a200_1103

# T4 - Set initial pose
source ~/clearpath/setup.bash
ros2 topic pub --once /a200_1103/initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'},
    pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
    covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0,
                 0,0,0,0,0,0, 0,0,0,0,0,0,
                 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}"

# T5 - Verify TF before continuing
source ~/clearpath/setup.bash
ros2 run tf2_ros tf2_echo map odom --ros-args \
  -r /tf:=/a200_1103/tf -r /tf_static:=/a200_1103/tf_static
# Must show a stable repeating transform before moving on.

# T6 - Nav2 (custom)
source ~/clearpath/setup.bash
ros2 launch ~/clearpath/nav2_custom.launch.py use_sim_time:=true

# T7 - Load a route graph
source ~/clearpath/setup.bash
ros2 service call /a200_1103/route_server/set_route_graph \
  nav2_msgs/srv/SetRouteGraph \
  "{graph_filepath: /home/rbt-roeun/clearpath/route/testroute1.geojson}"
```

## Running an evaluation

Drive an arbitrary loop for N laps, logging pose and velocity throughout:

```bash
source ~/clearpath/setup.bash
python3 route/route_loop_eval.py --laps 10 --edge-timeout 45 \
  --loop "0,1,2,3,0" \
  --ros-args -r /tf:=/a200_1103/tf -r /tf_static:=/a200_1103/tf_static \
  -p use_sim_time:=true
```

`--loop` accepts any comma-separated node sequence, e.g. a 9-node loop:
```bash
--loop "0,1,2,3,4,5,6,7,8,0"
```

Then compute the metrics:
```bash
source ~/clearpath/setup.bash
python3 route/analyze_laps.py --logdir ~/clearpath/eval_logs
```

This reports, per lap and aggregated: route tracking accuracy, cross-track
error, corner handling (dwell time / cross-track error / peak angular
velocity per node), velocity stability (vx/wz standard deviation, peak
acceleration, RMS jerk), and route completion success rate — plus a
`summary_laps.json` and a trajectory overlay plot.

## Known limitations

- Acceleration/jerk can briefly spike if velocity log samples have an
  irregular time gap; `analyze_laps.py` filters samples outside a plausible
  0.02-0.15s delta before computing finite differences.
- Corner-handling windows are bounded by both distance (0.5 m
  node-achievement radius) and elapsed time, so a node visited twice in one
  lap (start and end of a loop) isn't conflated into one inflated dwell time.
- File paths inside `nav2_custom.yaml`, `route_server_params.yaml`, and the
  Python scripts are absolute (`/home/rbt-roeun/...`) and machine-specific —
  update them if cloning onto a different user/machine.
