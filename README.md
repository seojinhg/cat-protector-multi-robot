# 🐱 Intelligent Cat Protector — Multi-Robot Monitoring System

> **Doosan Robotics Bootcamp (2025) — Intelligence 2nd Project**
> A multi-robot system using two TurtleBot4s that detects and tracks cats in an indoor environment, identifies sick cats, and dispatches a patrol robot to their location via MQTT-based inter-robot communication.

---

## 📌 Project Overview

Two TurtleBot4 robots collaborate to monitor cats in an indoor environment:

- **Tracking Robot**: Detects and tracks cats in real-time using YOLOv11 + OAK-D camera, and publishes sick cat locations via MQTT
- **Patrol Robot**: Autonomously patrols the environment, receives sick cat location alerts from the tracking robot via MQTT broker, and navigates to the reported location

---

## 🛠 Tech Stack

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.10-green)
![YOLO](https://img.shields.io/badge/YOLOv11-Ultralytics-purple)
![Nav2](https://img.shields.io/badge/Nav2-Navigation-orange)
![MQTT](https://img.shields.io/badge/MQTT-Broker-lightgrey)

| Category | Technology |
|----------|-----------|
| Robot | TurtleBot4 × 2 |
| Camera | OAK-D (Depth + RGB) |
| Object Detection & Tracking | YOLOv11 (custom trained) |
| Navigation | Nav2, TurtleBot4Navigator |
| Sensor Fusion | LiDAR + Depth Camera |
| Map Generation | SLAM Toolbox (Nav2) |
| Inter-robot Communication | MQTT Broker |
| Sensor Synchronization | message_filters.ApproximateTimeSynchronizer |
| Framework | ROS2 Humble |
| Language | Python 3.10 |

---

## 👥 Team & My Contributions

| | |
|--|--|
| **Team** | 2-team joint project |
| **Total Members** | 10 |
| **Duration** | 5 days |

### ⭐ My Role

| Area | Contribution |
|------|-------------|
| 🎯 Object Detection | Custom dataset collection & YOLOv11 model training |
| 🔍 Object Tracking | Real-time cat tracking with YOLO Track ID |
| 🤖 Nav2 Navigation | Goal pose generation & robot movement logic |

---

## 🎯 Implementation Details

### 1. YOLOv11-based Cat Detection ⭐

**Model Selection**

YOLOv11 was selected for its balance between real-time inference speed and detection accuracy, making it suitable for continuous tracking on a mobile robot platform.

**Problem: False Positives with Public Dataset**

The initial model trained on the Roboflow public dataset misclassified people in the background as cats, making it unreliable in real indoor environments.

**Solution: Custom Dataset with Domain Gap Minimization** 💡

To minimize the domain gap between training data and real deployment, photos were captured **directly using the TurtleBot4's onboard camera** in the actual lab environment — ensuring identical viewpoint, resolution, and field of view as the actual inference environment.

1. Captured ~100 photos of cats and sick cats using the TurtleBot4's onboard camera in the actual deployment environment
2. Applied data augmentation to expand to ~2,000 training images
3. Retrained YOLOv11 on this custom dataset

| Dataset | Accuracy | Issue |
|---------|----------|-------|
| Roboflow public dataset | - | People in background detected as cats |
| **Custom dataset (TurtleBot4 camera, self-collected)** | **97%** | ✅ Resolved |

---

### 2. Object Tracking — Real-time Cat Following ⭐

**Problem: Camera Delay & Latency**

Receiving camera frames caused timing delays between detection and robot response, making real-time tracking unreliable.

**Solution:**
- Reduced camera image resolution to lower processing load
- Tuned FPS settings to balance frame rate and processing time
- Used **YOLO Track ID** (`persist=True`) to maintain consistent object identity across frames — without Track ID, the same cat could be assigned a new ID each frame, causing erratic robot behavior
- Addressed QoS settings, camera FPS, callback processing time, and synchronization timing

**Sensor Synchronization**

Used `message_filters.ApproximateTimeSynchronizer` to synchronize three data streams with matching timestamps:
- RGB (`CompressedImage`)
- Depth (`Image`)
- Camera Info (`CameraInfo`)

Synchronization timing was one of the most challenging aspects — ROS2 network instability caused frequent desync issues during multi-robot testing.

---

### 3. 3D Position Estimation & Nav2 Navigation ⭐

**3D Coordinate Calculation**
- Extracts average Z depth value over a 3×3 pixel region at detected object center `(cx, cy)` — averaging reduces noise from single-pixel depth errors
- Uses camera intrinsic matrix **K** to compute full `(X, Y, Z)` 3D coordinates
- Depth range filtered to `0.2m ~ 5.0m` to reject invalid readings

**TF Transform**
- Converts `camera_frame → map` frame using TF to get global position
- Uses `do_transform_point` with accurate timestamps for precise transformation

**Nav2 Navigation**
- Calls `goToPose(goal_pose)` via `TurtleBot4Navigator`
- `goal_interval = 0.3s` and `min_goal_distance = 0.3m` thresholds prevent duplicate/excessive movement commands (see Troubleshooting)
- RViz Marker (red sphere) visualizes the target position

---

### 4. Patrol Robot — Autonomous Patrol & Sick Cat Response

**Autonomous Patrol**
- Follows a predefined set of waypoints built from a custom SLAM map
- Patrol path is automatically reversed to create a continuous loop

**Battery Management**
- Continuously monitors battery level
- Below 30% → automatically docks to recharge
- Above 95% → undocks and resumes patrol
- Below 10% → emergency shutdown

**Sick Cat Response**
- Subscribes to `/mqtt_message` topic to receive sick cat coordinates from tracking robot
- Interrupts current patrol, navigates through intermediate waypoints to avoid obstacles, then moves to the reported location
- Resumes patrol after arriving

---

### 5. Inter-robot Communication via MQTT

Two TurtleBot4s communicate through an **MQTT broker** deployed on the cloud:
- The tracking robot publishes the sick cat's map coordinates as an MQTT message
- The MQTT broker relays the message between robots over the network
- The patrol robot subscribes to `/mqtt_message` and receives the coordinates

This architecture solved the network instability problem caused by multiple teams sharing the same network simultaneously — by routing communication through a cloud broker rather than direct peer-to-peer ROS2 topics.



---

## 🏗️ System Architecture

```
TurtleBot4 #1 (Tracking Robot)
        │
        ├── OAK-D Camera (RGB + Depth)
        ├── LiDAR
        │
        ▼
YOLOv11 Detection & Tracking (Track ID, persist=True)
        │
        ▼
Sensor Synchronization (ApproximateTimeSynchronizer)
        │
        ▼
3D Position Estimation (Camera K matrix + TF transform)
        │
        ▼
Nav2 → goToPose() → Follow cat
        │
        │ [Sick cat detected → publish coordinates]
        ▼
MQTT Broker (Cloud)
        │
        ▼
TurtleBot4 #2 (Patrol Robot)
        │
        ├── Autonomous patrol (SLAM map waypoints)
        ├── Battery monitoring & auto-docking
        │
        ▼
Nav2 → goToPose() → Navigate to sick cat location
```

---

## 📁 Repository Structure

```
intelligent-cat-protector/
├── yolo_object_publisher.py   # Tracking robot: YOLO detection, tracking, Nav2
├── patrol_robot.py            # Patrol robot: autonomous patrol, MQTT, battery
├── SDD.png                    # System Design Diagram
├── SFD.png                    # System Flow Diagram
└── README.md
```

---

## 🚀 Prerequisites

> ⚠️ This project requires physical TurtleBot4 hardware, a custom SLAM map, and a configured MQTT broker. Full reproduction requires the original lab environment.

- ROS2 Humble
- TurtleBot4 × 2 (with OAK-D camera)
- Nav2 + SLAM Toolbox
- MQTT Broker (cloud-based)
- Python dependencies:
```bash
pip install ultralytics opencv-python numpy
```

---

## 📊 Results

| Item | Result |
|------|--------|
| Cat detection accuracy (custom dataset) | **97%** |
| False positive issue (people detected as cats) | ✅ Resolved via custom dataset |
| Real-time tracking with Track ID | ✅ Stable |
| Inter-robot communication via MQTT | ✅ Operational after cloud setup |
| Autonomous patrol + battery management | ✅ Fully operational |
| Sick cat → patrol robot dispatch pipeline | ✅ Fully operational |

---

## 🎬 Demo

> 📹 Demo video coming soon

---

## 🔧 Troubleshooting

### Navigation Goal Optimization
Sending a new Nav2 goal every YOLO detection frame caused excessive load and motion instability — the robot wobbled by constantly switching targets.

**Solution**: Tuned two threshold values through real robot testing:
- `goal_interval = 0.3s` — minimum time between goal updates to prevent Nav2 overload
- `min_goal_distance = 0.3m` — only sends a new goal if the cat moved more than 0.3m, reducing unnecessary robot motion for small movements

### Network Instability
Multiple teams connecting to the same network simultaneously caused severe congestion, making direct ROS2 inter-robot communication unreliable.

**Solution**: Deployed an MQTT broker on the cloud to relay messages between robots. Each robot publishes/subscribes through the broker rather than communicating directly — resolved the connectivity issue. However, time lost to network problems significantly delayed testing.

### Testing Without Full Robot Access
Due to network delays, live robot testing time was limited. Pre-recorded video footage from TurtleBot4 drives was used to test YOLO detection and tracking logic offline before deploying to the real robots.

### ROS2 Sensor Synchronization
Aligning QoS settings, camera FPS, callback processing time, and synchronization timing between two robots running simultaneously was difficult. ROS2 network instability amplified desync issues during multi-robot testing.

### Map & Waypoint Coordinates
All patrol waypoints and the docking station position are hardcoded based on a custom SLAM map:
1. Manually operated TurtleBot4 through the environment (teleoperation)
2. LiDAR scanned surroundings during movement
3. SLAM Toolbox built the map from scan data
4. Waypoint coordinates extracted from the generated map

Future improvement: load waypoints dynamically via ROS2 parameters.

### Model Path
Model path is configurable via ROS2 parameter (`model_path`) to support different deployment environments without modifying source code:
```bash
ros2 run <package> yolo_object_publisher --ros-args -p model_path:=/path/to/best.pt
```

---

## 💡 Key Takeaways

- **Rapid integration is key**: In short-term projects, the most effective strategy is to implement features with minimal errors and repeatedly integrate & test as early as possible — rather than developing each module in isolation
- **Domain gap matters**: Training data must match the real deployment environment. Capturing data with the actual robot camera improved detection accuracy significantly
- **Communication is critical**: Lack of communication among team members delayed integration testing. Mediating conflicts and emphasizing the importance of early integration was essential to completing the project
- **Future improvement**: When a tracked object is lost mid-tracking, there is currently no exception handling. Adding re-identification or lost-track recovery logic would make the system significantly more robust

---

## 📝 Notes

- YOLO model weights (`.pt`) and large zip files are excluded from this repository due to size constraints
- MQTT broker platform details to be updated (TODO)
- All map coordinates are specific to the original lab environment
- Glass was excluded from recyclable categories in a separate project for safety reasons
