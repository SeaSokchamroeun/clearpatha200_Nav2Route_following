# Husky A200 — Nav2 Route Following & Performance Evaluation

Platform: Clearpath Husky A200 | Namespace: `a200_1103` | ROS 2 Jazzy / Ubuntu 24.04 | Simulation: Gazebo Harmonic

This repository documents and version-controls the Nav2 Route Server integration for the Husky A200 virtual trajectory following project — the custom configuration, the fixes required to get the robot physically moving, and the tooling used to evaluate route-following performance.

> **Path note:** the launch files in this repo hardcode paths under `~/clearpath/` (e.g. `~/clearpath/localization_custom.yaml`). This repo mirrors that directory on the robot workstation — copy these files back into `~/clearpath/` (or update the path constants at the top of each `.py` file) before launching on a new machine.

## Status

- [x] AMCL `map→odom` TF fix — confirmed working
- [x] Nav2 launch with custom route_server config — confirmed working
- [x] Route loaded, robot physically follows the graph — confirmed working
- [x] Performance evaluation (10-lap, 4-node loop) — confirmed, 100% success rate
- [ ] 9-node loop (`fullLoopRoute2.geojson`) — pending file audit, not yet run

## Repository layout

| Path | Status | Purpose |
|---|---|---|
| `localization_custom.yaml` | Use | AMCL config with `set_initial_pose: true` — the fix that gets `map->odom` TF publishing |
| `localization_custom.launch.py` | Use | Wraps `nav2_bringup/localization_launch.py`, injects the AMCL fix above |
| `nav2_custom.yaml` | Use | Full nav2 params; `route_server` section points at the active graph file |
| `nav2_custom.launch.py` | Use | Wraps `nav2_bringup/navigation_launch.py`, injects the params above |
| `route/testroute1.geojson` | Use | 4-node square loop. Verified — no CRS block, valid edge coordinates |
| `route/fullLoopRoute1.geojson` | Unverified | Not yet audited for the CRS / empty-geometry bug — confirm before use |
| `route/fullLoopRoute2.geojson` | Unverified | 9-node loop (0->1->...->8->0) plus shortcut edges 8->1, 8->0, 8->5. Not yet audited |
| `route/route_loop_eval.py` | Use | Drives an arbitrary loop edge-by-edge for N laps, logs pose/velocity for evaluation |
| `route/analyze_laps.py` | Use | Computes tracking accuracy, cross-track error, corner handling, velocity stability, and success rate from lap logs |
| `route/drive_route.py` | Use | Single-lap driver — bridges `ComputeAndTrackRoute` output to `FollowPath` |
| `route/route_server_params.yaml` | Use | Standalone route_server params, points at the active graph |
| `route/send_route_goal.py` | Legacy | Computes/tracks a route only — does not drive the robot. Kept for reference |
| `route/route_eval.py` | Legacy | Superseded by `route_loop_eval.py` — jumped between fixed node IDs instead of driving a real loop, which caused goal/position desync |
| `route/route_server.launch.py` | Do not use | Standalone route_server — duplicates the one nav2 already launches |
| `route/nav2_with_route.launch.py` | Do not use | Same duplicate-route_server problem |
| `route/testLoop.geojson`, `testLoop1.geojson`, `testStraight.geojson` | Do not use | Still contain the EPSG::3857 CRS block and empty edge geometry |

## The two fixes this repo is built around

**1. AMCL won't publish `map->odom` TF on its own.**
AMCL requires a completed particle filter update — which requires a laser scan — before it publishes the `map->odom` transform. A sub-millisecond timestamp mismatch between the scan and the latest odometry TF was blocking that first update indefinitely ("extrapolation into the future"). Setting `set_initial_pose: true` in `localization_custom.yaml` makes AMCL publish the transform immediately on startup instead of waiting on that cycle.

**2. The route server doesn't drive the robot.**
`ComputeAndTrackRoute` (the Route Server's action) only computes and tracks a route — it never sends commands to the controller. In a standard Nav2 setup a Behavior Tree bridges that gap; this project talks to the route server directly, so `route_loop_eval.py` / `drive_route.py` does that bridging explicitly: the moment a path appears in the route server's feedback, it's forwarded as a goal to `controller_server`'s `FollowPath` action.

## Launch sequence

Run each in its own terminal, in order:

```bash
# T1 - Gazebo
source ~/clearpath/setup.bash
ros2 launch clearpath_gz simulation.launch.py

# T2 - Localization (AMCL fix included)
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

# T5 - Verify map->odom TF before continuing
source ~/clearpath/setup.bash
ros2 run tf2_ros tf2_echo map odom --ros-args \
  -r /tf:=/a200_1103/tf -r /tf_static:=/a200_1103/tf_static

# T6 - Nav2 (custom)
source ~/clearpath/setup.bash
ros2 launch ~/clearpath/nav2_custom.launch.py use_sim_time:=true

# T7 - Load the route graph
source ~/clearpath/setup.bash
ros2 service call /a200_1103/route_server/set_route_graph \
  nav2_msgs/srv/SetRouteGraph \
  "{graph_filepath: /home/rbt-roeun/clearpath/route/testroute1.geojson}"
```

Do not proceed past T5 until it shows a stable transform.

## Running an evaluation

`route_loop_eval.py` drives any loop edge-by-edge for N laps and logs continuous pose and velocity telemetry. The loop is given as a comma-separated node sequence:

```bash
source ~/clearpath/setup.bash
python3 ~/clearpath/route/route_loop_eval.py --laps 10 --edge-timeout 45 \
  --loop "0,1,2,3,0" \
  --ros-args -r /tf:=/a200_1103/tf -r /tf_static:=/a200_1103/tf_static \
  -p use_sim_time:=true
```

Then compute metrics:

```bash
source ~/clearpath/setup.bash
python3 ~/clearpath/route/analyze_laps.py --logdir ~/clearpath/eval_logs
```

This reports, per lap and in aggregate:

- **Route tracking accuracy** — mean / RMSE / max perpendicular distance to the planned path
- **Cross-track error** — sideways deviation from the path
- **Corner handling** — dwell time, cross-track error, and peak turn rate at each graph node
- **Velocity stability** — standard deviation of forward/turning speed, peak acceleration, jerk
- **Route completion success rate** — per-edge and per-lap

## Results snapshot — testroute1.geojson, 10 laps

| Metric | Result |
|---|---|
| Lap completion success rate | 10/10 (100%) |
| Per-edge success rate | 100% on all 4 edges |
| Mean tracking error | 0.038 m |
| Tracking RMSE | 0.052 m (range 0.042-0.068 m) |
| Hardest corner | Node 3 — longest dwell, highest cross-track error (route geometry, not a controller fault) |
| Loop closure accuracy | 0.032 m mean |
| Velocity stability | vx std ~ 0.160, wz std ~ 0.169, consistent across all laps |

Full breakdown: `results/summary_laps.json` and `results/loop_trajectory_overlay.png` if included (see setup steps in this repo's history), or the project's Performance Evaluation Report.

## Known root causes (history)

| # | Symptom | Root cause | Fix |
|---|---|---|---|
| 1 | Robot stationary, AMCL never publishes TF | Particle filter blocked by a scan/odom timestamp gap | `set_initial_pose: true` in `localization_custom.yaml` |
| 2 | Route computed and visible in RViz, robot still doesn't move | `controller_server`'s `FollowPath` had zero action clients — nothing was calling it | Bridge script forwards the route server's path to `FollowPath` directly |
| 3 | Repeated single-edge runs (`0->3`) failed with `INVALID_PATH` after the first | Each run requested a fixed start node regardless of the robot's actual position; uncancelled prior goals compounded it | Drive the loop sequentially, edge by edge, so the robot is always physically where the next edge expects it |

