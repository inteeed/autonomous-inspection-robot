# Autonomous Industrial Inspection Robot

A ROS2-based autonomous inspection robot simulation that navigates a Gazebo environment, detects ArUco markers at industrial equipment locations, and generates structured inspection reports.

**Authors:** Izzatbek & Boburjon

---

## Overview

The robot simulates a real-world industrial inspection workflow:

1. Navigates autonomously through a set of predefined waypoints using odometry-based control
2. Detects ArUco markers mounted on equipment (tanks, valves, panels, pipe junctions)
3. Logs all detections with pose, distance, and robot position data
4. Generates a timestamped `report.json` and `report.csv` at the end of each run

**Robot:** TurtleBot3 Waffle Pi  
**Simulator:** Gazebo Classic  
**Language:** Python 3 / ROS2 (ament_python)

---

## Architecture

```
Gazebo Simulation
       |
  [Camera Feed]  ──→  aruco_detector  ──→  /inspection/detections  ──→  report_logger
  [Odometry]     ──→  waypoint_follower ──────────────────────────────→  report_logger
                           |
                      /cmd_vel
                           |
                    TurtleBot3 motion
```

| Node | Role |
|---|---|
| `aruco_detector` | Detects ArUco markers from camera images, publishes JSON detections |
| `waypoint_follower` | 4-phase turn→drive→align→dwell controller, visits 6 waypoints |
| `report_logger` | Deduplicates sightings, writes `report.json` + `report.csv` |

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
- TurtleBot3 packages: `turtlebot3`, `turtlebot3_gazebo`, `turtlebot3_description`
- Python: `opencv-contrib-python`, `numpy`, `pyyaml`
- `ros-humble-cv-bridge`, `ros-humble-gazebo-ros-pkgs`

---

## Build & Run

```bash
# Clone and build
cd ~/ros2_ws
colcon build --packages-select inspection_robot
source install/setup.bash

# Generate ArUco marker textures (first time only)
python3 src/inspection_robot/scripts/generate_world_assets.py

# Launch everything (Gazebo + all 3 nodes)
ros2 launch inspection_robot bringup.launch.py
```

Reports are saved to `~/inspection_reports/run_YYYYMMDD_HHMMSS/`.

---

## Next Steps

### Phase 1 — Simulation Improvements
- [x] **Obstacle avoidance:** LaserScan subscriber in `waypoint_follower.py` checks the forward arc (±20°) and steers toward the more open side when an obstacle is within `obstacle_distance_m` (default 0.40 m)
- [x] **Camera calibration file:** `config/camera_calibration.yaml` holds TurtleBot3 Waffle Pi intrinsics; `aruco_detector.py` loads it via the `calibration_file` parameter as a fallback before `CameraInfo` arrives (live `CameraInfo` always takes precedence)
- [x] **Annotated image stream:** Publishes the OpenCV-annotated image on `/inspection/annotated` for live RViz visualization during a run
- [x] **Re-detection robustness:** `min_consecutive_detections` parameter (default 2) requires a marker to appear in N consecutive frames before it is published to `/inspection/detections`; raw detections still appear in the annotated stream

### Phase 2 — Real Hardware Readiness
- [ ] **Navigation2 migration:** Replace the custom odometry PID controller with Nav2 (`NavigateToPose` action) so the robot can handle dynamic obstacles and use a proper global costmap
- [ ] **SLAM integration:** Add a SLAM node (e.g. `slam_toolbox`) to build a map on the first run; use the saved map for localization on subsequent runs
- [ ] **Hardware bring-up launch:** Create a separate `bringup_hw.launch.py` that swaps Gazebo nodes for real TurtleBot3 bring-up (`turtlebot3_bringup`) and a real camera node

### Phase 3 — Inspection Intelligence
- [ ] **Anomaly detection:** Add a secondary computer vision node that checks the region around each detected marker for visual anomalies (leaks, corrosion, warning lights) using a pretrained model
- [ ] **Inspection scheduling:** Build a simple scheduler node that triggers runs on a time-based schedule and skips waypoints where markers were recently seen and conditions were nominal
- [ ] **Remote dashboard:** Stream detection events to a web dashboard (e.g. Flask + Socket.IO or Foxglove Studio) so operators can monitor inspections in real time
- [ ] **PDF report generation:** Extend `report_logger` to render a human-readable PDF report with annotated images, a site map, and a pass/fail summary per equipment item

### Phase 4 — Deployment & CI
- [ ] **Docker image:** Package the entire workspace into a Docker image for reproducible builds and easy deployment on edge hardware
- [x] **CI pipeline:** `.github/workflows/ci.yml` runs pytest unit tests + flake8 lint on every push/PR (no ROS2 installation required)
- [x] **Unit tests:** `test/test_waypoint_math.py` covers `quat_to_yaw`, `angle_diff`, and `_load_waypoints`; `test/test_report_logger.py` covers deduplication, best-distance tracking, and flush output — all run without ROS2
