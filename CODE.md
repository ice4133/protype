# Prototype - Dual-Arm Robotics Control System

A ROS 2 Humble-based robotics control system for dual robotic arm manipulation with VR teleoperation, data collection for imitation learning, and ML-based inference support.

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Directory Structure](#directory-structure)
- [Core Components](#core-components)
- [Message Types](#message-types)
- [Communication Patterns](#communication-patterns)
- [Configuration](#configuration)
- [Quick Start](#quick-start)
- [Development](#development)

---

## Overview

This project provides a complete software stack for controlling dual robotic arms with:

- **VR Teleoperation**: Quest 2 VR controller support for intuitive arm manipulation
- **Two Arm Platforms**: Support for YAM (Damiao motors) and ARX R5 robotic arms
- **Data Collection**: HDF5-based data collection for imitation learning datasets
- **Inference Pipeline**: ZMQ-based inference bridge for ML model deployment
- **Multi-Camera Support**: Intel RealSense D405/D457/L515 camera integration

### Key Technologies

| Component | Technology |
|-----------|------------|
| Framework | ROS 2 Humble |
| Package Manager | Pixi (conda-based) |
| Languages | C++ (controllers), Python (nodes/scripts) |
| Kinematics | Pinocchio |
| Communication | ROS 2 Topics, ZMQ |
| Data Storage | HDF5 |
| Cameras | Intel RealSense SDK |
| Build System | CMake, colcon |

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              VR Teleoperation                                │
│  ┌─────────────┐    ┌──────────────────┐    ┌─────────────────────────────┐ │
│  │  Quest 2    │───▶│  yam_cmd_pub /   │───▶│   /YAM_VR_L, /YAM_VR_R      │ │
│  │  Controller │    │  arx_cmd_pub     │    │   Command Topics            │ │
│  └─────────────┘    └──────────────────┘    └─────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                            Arm Controllers                                   │
│  ┌───────────────────────┐         ┌───────────────────────┐                │
│  │  YamController (C++)  │         │  R5Controller (C++)   │                │
│  │  - Damiao motors      │         │  - CAN interface      │                │
│  │  - 6-DOF + gripper    │         │  - 6-DOF + gripper    │                │
│  │  - Gravity comp       │         │  - Position/End ctrl  │                │
│  │  - IK/FK via Pinocchio│         │  - Kinematics solver  │                │
│  └───────────────────────┘         └───────────────────────┘                │
│           │                                  │                               │
│           ▼                                  ▼                               │
│  ┌──────────────────┐              ┌──────────────────┐                     │
│  │ /arm_l_status    │              │ /arm_l_status    │                     │
│  │ /arm_r_status    │              │ /arm_r_status    │                     │
│  └──────────────────┘              └──────────────────┘                     │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                    ┌───────────────┴───────────────┐
                    ▼                               ▼
┌───────────────────────────────┐  ┌───────────────────────────────────────────┐
│       Data Collector          │  │           Inference Pipeline              │
│  ┌─────────────────────────┐  │  │  ┌─────────────────┐  ┌────────────────┐  │
│  │ data_collector_node.py  │  │  │  │ joint_base_node │  │  camera_node   │  │
│  │ - HDF5 output           │  │  │  │ (ZMQ bridge)    │  │  (ZMQ pub)     │  │
│  │ - Configurable topics   │  │  │  └─────────────────┘  └────────────────┘  │
│  │ - Image/joint/depth     │  │  │          │                    │           │
│  └─────────────────────────┘  │  │          ▼                    ▼           │
└───────────────────────────────┘  │  ┌────────────────────────────────────┐   │
                                   │  │     External ML Inference Server   │   │
                                   │  │     (via ZMQ ports 6001-6007)      │   │
                                   │  └────────────────────────────────────┘   │
                                   └───────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│                              Camera System                                   │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────────────┐  │
│  │ D405 Left Wrist │  │ D405 Right Wrist│  │ D405/D457/L515 High/Top    │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────────────────┘  │
│           │                    │                        │                    │
│           ▼                    ▼                        ▼                    │
│  /cam_left_wrist/*    /cam_right_wrist/*       /cam_high/*                  │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Directory Structure

```
prototype/
├── pixi.toml                    # Pixi package manager configuration
├── config/                      # Camera JSON configurations
│   └── d405.json, d457.yaml
├── bash/                        # Shell scripts for running components
│   ├── run_yam.sh              # Launch YAM arm controllers
│   ├── run_arx.sh              # Launch ARX arm controllers
│   ├── run_cameras.sh          # Launch camera nodes
│   ├── infer_yam.sh            # Launch YAM inference pipeline
│   ├── infer_arx.sh            # Launch ARX inference pipeline
│   ├── run_data_collector.sh   # Launch data collection
│   └── activate_can.sh         # Activate CAN interfaces
├── scripts/                     # Python launch scripts for cameras
│   ├── d405_wrist.py           # Wrist camera launch
│   ├── d405_high.py            # High/top camera launch
│   ├── d457_high.py            # D457 camera launch
│   └── launch_l515.py          # L515 camera launch
└── src/
    ├── yam/                     # YAM arm system
    │   ├── yam_damiao_controller/  # Main YAM controller (C++)
    │   │   ├── src/
    │   │   │   ├── YamController.cpp       # Main controller node
    │   │   │   ├── Damiao_6dof_node.cpp    # Motor interface
    │   │   │   └── mathematical_model/     # Kinematics (modern_robotics, yam_fun)
    │   │   ├── config/                     # YAML configurations
    │   │   │   └── vr_double_arm.yaml      # VR dual-arm config
    │   │   └── launch/                     # ROS 2 launch files
    │   ├── yam_cmd_pub/         # VR command processor (Python)
    │   │   └── yam_cmd_pub/
    │   │       ├── cmd_processor.py        # VR pose to arm commands
    │   │       └── arm_processor.py        # SE3 transformations
    │   └── yam_description/     # URDF/meshes for YAM arm
    │
    ├── arx/                     # ARX R5 arm system
    │   ├── arx_r5_controller/   # Main ARX controller (C++)
    │   │   ├── src/
    │   │   │   └── R5Controller.cpp        # Main controller node
    │   │   ├── config/                     # YAML configurations
    │   │   └── launch/                     # ROS 2 launch files
    │   └── arx_cmd_pub/         # ARX command processor (Python)
    │
    ├── inference/               # ML inference bridge
    │   └── inference/
    │       ├── joint_base_node.py   # ZMQ bridge for joint commands
    │       └── camera_node.py       # ZMQ bridge for camera images
    │
    ├── data_collector/          # Data collection for imitation learning
    │   ├── data_collector/
    │   │   ├── data_collector_node.py  # Main collector node
    │   │   └── data_processor.py       # Image/joint processors
    │   └── config/
    │       └── collect_config.yaml     # Collection configuration
    │
    ├── msg/                     # Custom ROS 2 messages
    │   ├── yam_arm_msg/         # YAM arm messages
    │   │   └── msg/
    │   │       ├── YamCmd.msg           # Command message
    │   │       └── YamStatus.msg        # Status message
    │   ├── arxmsgros2/          # ARX arm messages
    │   │   ├── arx5_arm_msg/
    │   │   │   └── msg/
    │   │   │       ├── RobotCmd.msg     # Command message
    │   │   │       └── RobotStatus.msg  # Status message
    │   │   └── arm_control/     # VR pose messages
    │   └── quest2_button_msg/   # VR button messages
    │
    └── rs_cameras/              # RealSense camera ROS 2 wrapper
        └── realsense-ros-4.51.1/
```

---

## Core Components

### 1. YAM Controller (`src/yam/yam_damiao_controller`)

**Language**: C++
**Entry Point**: `src/YamController.cpp:567`

The YAM controller manages Damiao servo motors via CAN interface with a state machine architecture:

| State | Description |
|-------|-------------|
| `INIT` | Homing procedure - moves all joints to zero position |
| `SOFT` | Motors disabled (safe state) |
| `JOINT_CONTROL` | Direct joint position control with trajectory filtering |
| `END_CONTROL` | End-effector pose control via IK |
| `G_COMPENSATION` | Gravity compensation mode (backdrivable) |

**Key Features**:
- 6-DOF arm + 1-DOF gripper control
- Pinocchio-based forward/inverse kinematics
- Trajectory smoothing system with configurable bandwidth
- Inverse dynamics for feedforward torque
- 200Hz control loop (configurable)

**Architecture**:
```
Damiao6dofInterfacesThread
└── robotControl (MotorControl)     # Unified controller for all 7 motors
    ├── Motors 0-5: Arm joints (DM4340 x3, DM4310 x3)
    └── Motor 6: Gripper (DM4310)
```

The motor interface uses a **unified MotorControl** design where all 7 motors (6 arm + 1 gripper) share a single CAN socket. This ensures deterministic message routing and avoids race conditions that could occur with separate controllers competing for the same CAN bus.

**Motor Interface API**:
- `setRobotPosition(arm_pos, arm_vel, arm_P, arm_D, arm_tor, gripper_pos, gripper_vel, gripper_P, gripper_D, gripper_tor)` - Sends commands to all 7 motors in a single transaction
- `updateYamStatus()` - Reads state from all motors
- `getJointPositions()` / `getGripperPositions()` - Returns arm/gripper state

**ROS 2 Interface**:
- Subscribes: `/YAM_VR_L` or `/YAM_VR_R` (`YamCmd`)
- Publishes: `/arm_l_status` or `/arm_r_status` (`YamStatus`)

### 2. ARX R5 Controller (`src/arx/arx_r5_controller`)

**Language**: C++
**Entry Point**: `src/R5Controller.cpp:233`

The ARX R5 controller supports multiple operation modes:

| Mode | Description |
|------|-------------|
| `normal` | Standard position/end-effector control |
| `vr_slave` | VR teleoperation follower mode |
| `remote_master` | Gravity compensation for teach mode |
| `remote_slave` | Follow another arm's joint positions |

**Key Features**:
- CAN bus communication
- URDF-based kinematics
- Multiple gripper control modes (position/torque)
- 1kHz status publishing

### 3. VR Teleoperation (`src/yam/yam_cmd_pub` and `src/arx/arx_cmd_pub`)

**Language**: Python
**Entry Point**: `yam_cmd_pub/cmd_processor.py:179`

Processes Quest 2 VR controller input for arm teleoperation:

| State | Description |
|-------|-------------|
| `TELEOP` | Active teleoperation - VR poses sent to arms |
| `PAUSED` | Arms hold position, VR input ignored |
| `HOMING` | Moving arms to home position |

**Controls**:
- **Button A**: Toggle teleop/pause
- **Button B**: Start homing sequence
- **Button X**: Start data recording
- **Button Y**: Stop data recording
- **Triggers**: Gripper control (left/right)
- **Left Joystick + LG**: Base movement (linear X/Y)
- **Right Joystick + LG**: Base rotation (angular Z)

### 4. Data Collector (`src/data_collector`)

**Language**: Python
**Entry Point**: `data_collector/data_collector_node.py:290`

Collects synchronized multi-modal data for imitation learning:

**Supported Data Types**:
- RGB images (resizable)
- Depth images
- Point clouds (downsampled)
- Joint positions/velocities/currents
- End-effector poses
- Base odometry

**Output Format**: HDF5 files with configurable datasets

**Services**:
- `/start_collect` - Begin recording at configured frequency
- `/stop_collect` - Stop and save to HDF5

### 5. Inference Pipeline (`src/inference`)

**Language**: Python

Bridges ROS 2 and external ML inference servers via ZMQ:

| Node | Purpose | ZMQ Ports |
|------|---------|-----------|
| `joint_base_node` | Joint status pub, command sub | 6001 (status), 6006 (cmd) |
| `camera_node` | Camera image publishing | 6003-6005 |

**Data Flow**:
1. Robot status → ZMQ PUB → External inference
2. External inference → ZMQ SUB → Robot commands

---

## Message Types

### YamCmd (Command to YAM arm)

```
std_msgs/Header header
float64[7] end_pose    # [x, y, z, qw, qx, qy, qz] - position + quaternion
float64[6] joint_pos   # Joint positions (radians)
float64 gripper        # Gripper position
int64 mode             # Control mode (0=SOFT, 3=G_COMP, 4=END, 5=JOINT)
```

### YamStatus (Status from YAM arm)

```
std_msgs/Header header
float64[7] end_pose    # [x, y, z, qw, qx, qy, qz]
float64[7] joint_pos   # 6 joints + gripper
float64[7] joint_vel   # Joint velocities
float64[7] joint_cur   # Joint currents
```

### RobotCmd (Command to ARX arm)

```
std_msgs/Header header
float64[6] end_pos     # [x, y, z, roll, pitch, yaw]
float64[6] joint_pos   # Joint positions
float64 gripper        # Gripper value
int64 mode             # Control mode
```

### RobotStatus (Status from ARX arm)

```
std_msgs/Header header
float64[6] end_pos     # End-effector pose (XYZ RPY)
float64[7] joint_pos   # 6 joints + gripper
float64[7] joint_vel   # Joint velocities
float64[7] joint_cur   # Joint currents
```

---

## Communication Patterns

### ROS 2 Topic Graph

```
/vr_pose_left ──────┐
/vr_pose_right ─────┼──▶ yam_cmd_pub ──┬──▶ /YAM_VR_L ──▶ YamController_L ──▶ /arm_l_status
/vr_button ─────────┘                  └──▶ /YAM_VR_R ──▶ YamController_R ──▶ /arm_r_status
                                                                                    │
                                       ┌────────────────────────────────────────────┘
                                       ▼
                              data_collector_node ──▶ HDF5 files
                                       ▲
/cam_left_wrist/color/* ───────────────┤
/cam_right_wrist/color/* ──────────────┤
/cam_high/color/* ─────────────────────┘
```

### ZMQ Ports (Inference Pipeline)

| Port | Direction | Data Type |
|------|-----------|-----------|
| 6001 | PUB | Arm joint status (left + right) |
| 6002 | PUB | Base odometry |
| 6003 | PUB | Left camera image |
| 6004 | PUB | Right camera image |
| 6005 | PUB | Top camera image |
| 6006 | SUB | Arm joint commands |
| 6007 | SUB | Base velocity commands |

---

## Configuration

### Pixi Environment (`pixi.toml`)

Key dependencies:
- ROS 2 Humble Desktop
- librealsense 2.50.0
- Pinocchio (kinematics)
- NumPy, SciPy, OpenCV
- h5py (data storage)
- pynput, pyzmq

### Arm Controller Config (`config/vr_double_arm.yaml`)

```yaml
/yam_l:
  ros__parameters:
    arm_can_id: can_yam_l        # CAN interface
    arm_control_type: normal     # Operation mode
    arm_pub_topic_name: arm_l_status
    arm_sub_topic_name: YAM_VR_L
    arm_control_period: 0.005    # 200Hz control loop
    arm_kp_1_3: 120.0            # Joint gains (1-3)
    arm_kp_4_6: 30.0             # Joint gains (4-6)
    arm_kd_1_3: 2.0
    arm_kd_4_6: 1.0
    gripper_position_gain: 20.0
    gripper_velocity_gain: 1.0
```

### Data Collection Config (`config/collect_config.yaml`)

```yaml
data_dir: "/home/shu/data"
collection_frequency: 50         # Hz
max_queue_size: 5
max_age: 0.5                     # Max data staleness (seconds)

datasets:
  "/observations/images/cam_right_wrist":
    topic: "/cam_right_wrist/color/image_rect_raw"
    processor: "data_collector.data_processor.ImageProcessor"
    processor_config:
      resize: [640, 480]
      output_shape: [480, 640, 3]
      dtype: "uint8"

  "/state/joint_position/left":
    topic: "/arm_l_status"
    processor: "data_collector.data_processor.JointPositionProcessor"
    processor_config:
      output_shape: [7]
      dtype: "float64"
```

---

## Quick Start

### Prerequisites

1. Install [Pixi](https://pixi.sh)
2. CAN interfaces configured for motor controllers

### Setup

```bash
# Clone and enter directory
cd /home/shu/code/prototype

# Install dependencies (handled by pixi)
pixi install

# Activate CAN interfaces (if needed)
./bash/activate_can.sh
```

### Running the System

**Option 1: YAM Arms with VR Teleoperation**
```bash
# Terminal 1: Start cameras
./bash/run_cameras.sh

# Terminal 2: Start YAM arm controllers
./bash/run_yam.sh

# Terminal 3: Start data collector (optional)
./bash/run_data_collector.sh
```

**Option 2: ARX Arms**
```bash
./bash/run_arx.sh
```

**Option 3: Inference Mode**
```bash
# For YAM arms
./bash/infer_yam.sh

# For ARX arms
./bash/infer_arx.sh
```

### Building from Source

```bash
# Build all packages
pixi run colcon build --symlink-install

# Build specific package
pixi run colcon build --packages-select yam_damiao_controller
```

---

## Development

### Adding a New Data Processor

1. Create processor class in `src/data_collector/data_collector/data_processor.py`:

```python
class MyProcessor(DataProcessor):
    def process(self, msg):
        # Transform ROS message to numpy array
        return np.array(...)
```

2. Register in `collect_config.yaml`:

```yaml
datasets:
  "/my_dataset":
    topic: "/my_topic"
    processor: "data_collector.data_processor.MyProcessor"
    processor_config:
      output_shape: [...]
      dtype: "float32"
```

### Adding Support for New Arm

1. Create controller package under `src/`
2. Define message types in `src/msg/`
3. Create command processor in Python
4. Add launch files and configurations
5. Create shell scripts in `bash/`

### Key Files for Modification

| Task | Files |
|------|-------|
| Arm kinematics | `yam_damiao_controller/src/mathematical_model/yam_fun.cpp` |
| Motor interface | `yam_damiao_controller/src/Damiao_6dof_node.cpp` |
| Control state machine | `yam_damiao_controller/src/YamController.cpp` |
| Control gains | `config/vr_double_arm.yaml` |
| VR mapping | `yam_cmd_pub/arm_processor.py` |
| Data collection | `data_collector/config/collect_config.yaml` |
| Camera setup | `scripts/d405_*.py` |

---

## License

Private/Internal Use

## Authors

- Asahel (lichengmeng2001@foxmail.com)
