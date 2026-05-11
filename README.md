# Autonomous Industrial Inspection Robot

A ROS2-based autonomous inspection robot simulation that navigates a Gazebo environment, detects ArUco markers at industrial equipment locations, and generates structured inspection reports.

**Authors:** Izzatbek & Boburjon

---

## Overview

The robot simulates a real-world industrial inspection workflow:

1. Navigates autonomously through predefined waypoints with either the simple PID follower or Nav2 `NavigateToPose`
2. Detects ArUco markers mounted on equipment and publishes annotated camera images
3. Checks marker regions for simple visual anomalies and streams events to an operator dashboard
4. Generates timestamped `report.json`, `report.csv`, and `report.pdf` files at the end of each run

**Robot:** TurtleBot3 Waffle Pi  
**Simulator:** Gazebo Classic  
**Language:** Python 3 / ROS2 (ament_python)

---

## Architecture

```
Gazebo Simulation
       |
  [Camera Feed]  ──→  aruco_detector  ──→  /inspection/detections  ──→  report_logger
       │                 │
       │                 └────────────→  /inspection/annotated  ──→  RViz
  [Odometry]     ──→  waypoint_follower ──────────────────────────────→  report_logger
  [LaserScan]    ──→  waypoint_follower
                           |
                      /cmd_vel
                           |
                    TurtleBot3 motion
```

| Node | Role |
|---|---|
| `aruco_detector` | Loads camera calibration YAML, confirms detections across frames, publishes JSON detections + annotated images |
| `waypoint_follower` | Waypoint controller with scan-based obstacle rerouting around the tank obstacles |
| `nav2_waypoint_follower` | Sends each inspection waypoint to Nav2 using the `NavigateToPose` action |
| `anomaly_detector` | Checks marker ROIs for visual anomaly signals and saves ROI snapshots |
| `inspection_scheduler` | Triggers scheduled inspection runs and publishes skip hints for recently nominal markers |
| `dashboard_server` | Serves a live event dashboard over HTTP server-sent events |
| `report_logger` | Deduplicates sightings, merges anomalies, writes `report.json`, `report.csv`, and `report.pdf` |

---

## World Layout

10×10 m arena with 4 boundary walls and 2 cylindrical tank obstacles. Five ArUco markers (DICT_4X4_50, 0.4×0.4 m) are placed at:

| Marker ID | Equipment Label | Position |
|---|---|---|
| 0 | tank_A_inlet | (2.5, 0.0) |
| 1 | pipe_junction_1 | (0.0, 2.5) |
| 2 | tank_B_outlet | (−2.5, 0.0) |
| 3 | control_panel | (0.0, −2.5) |
| 4 | valve_cluster | (1.8, 1.8) |

---

## Prerequisites

- ROS2 Humble (or Foxy)
- Gazebo Classic (11)
- TurtleBot3 packages: `turtlebot3`, `turtlebot3_gazebo`, `turtlebot3_description`, `turtlebot3_bringup`
- Nav2 + SLAM packages: `nav2_bringup`, `nav2_msgs`, `slam_toolbox`
- Python: `opencv-contrib-python`, `numpy`, `pyyaml`
- `ros-humble-cv-bridge`, `ros-humble-gazebo-ros-pkgs`, `ros-humble-v4l2-camera`

---

## Build & Run

```bash
# Clone and build
cd ~/ros2_ws
colcon build --packages-select inspection_robot
source install/setup.bash

# Generate ArUco marker textures (first time only)
python3 src/inspection_robot/scripts/generate_world_assets.py

# Launch everything (Gazebo + nodes)
ros2 launch inspection_robot bringup.launch.py

# Launch with RViz watching /inspection/annotated
ros2 launch inspection_robot bringup.launch.py use_rviz:=true

# Launch simulation with Nav2 + slam_toolbox mapping
ros2 launch inspection_robot bringup_nav2.launch.py use_slam:=true

# Launch simulation with Nav2 localization against a saved map
ros2 launch inspection_robot bringup_nav2.launch.py use_slam:=false map:=/path/to/map.yaml

# Launch intelligence nodes: anomaly detector, scheduler, dashboard
ros2 launch inspection_robot inspection_intelligence.launch.py

# Hardware launch path for a TurtleBot3 + v4l2 camera
ros2 launch inspection_robot bringup_hw.launch.py map:=/path/to/map.yaml

# Live dashboard
xdg-open http://localhost:8080
```

Reports are saved to `~/inspection_reports/run_YYYYMMDD_HHMMSS/`. Anomaly ROI snapshots are saved under `~/inspection_reports/anomalies/` by default.

The hardware launch assumes TurtleBot3 bringup, a `/scan` laser, `/odom`, and a camera driver compatible with `v4l2_camera`. Replace `config/camera_hw.yaml` with a real calibration before hardware runs.

---

## Next Steps

### Phase 1 — Simulation Improvements
- [x] **Obstacle avoidance:** `waypoint_follower.py` now uses `/scan` to sidestep around blocked paths before resuming waypoint tracking
- [x] **Camera calibration file:** `aruco_detector.py` now loads YAML camera calibration from `config/camera_sim.yaml` and can also use the remote-added `config/camera_calibration.yaml` format
- [x] **Annotated image stream:** `aruco_detector.py` publishes a continuous `/inspection/annotated` image stream, and `bringup.launch.py` can start RViz with `use_rviz:=true`
- [x] **Re-detection robustness:** `aruco_detector.py` now applies a confidence gate and requires consecutive detections before publishing a sighting; raw detections still appear in the annotated stream

### Phase 2 — Real Hardware Readiness
- [x] **Navigation2 migration:** `nav2_waypoint_follower.py` uses Nav2 `NavigateToPose`; `bringup_nav2.launch.py` starts Nav2 in simulation
- [x] **SLAM integration:** `bringup_nav2.launch.py` can start `slam_toolbox` for mapping or Nav2 localization against a saved map
- [x] **Hardware bring-up launch:** `bringup_hw.launch.py` swaps Gazebo for TurtleBot3 bringup and a configurable real camera node

### Phase 3 — Inspection Intelligence
- [x] **Anomaly detection:** `anomaly_detector.py` checks marker ROIs for red warning/leak colors and dark stain/corrosion signals; it is structured so a model can replace the heuristic later
- [x] **Inspection scheduling:** `inspection_scheduler.py` triggers scheduled runs and publishes skip hints for markers recently seen as nominal
- [x] **Remote dashboard:** `dashboard_server.py` streams status, detection, and anomaly events to `http://localhost:8080`
- [x] **PDF report generation:** `report_logger.py` now writes a human-readable `report.pdf` with pass/fail summaries and anomaly snapshot paths

### Phase 4 — Deployment & CI
- [x] **Docker image:** `Dockerfile`, `.dockerignore`, and `docker/entrypoint.sh` build the ROS workspace into a reproducible Humble image
- [x] **CI pipeline:** `.github/workflows/colcon.yml` builds/tests the ROS package, and `.github/workflows/ci.yml` keeps the remote's lightweight pytest/lint checks
- [x] **Unit tests:** `pytest` tests cover report deduplication/PDF helper logic, waypoint skip/avoidance helpers, and the remote-added waypoint math coverage
