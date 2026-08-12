# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 Humble dual-arm robotics control system supporting YAM (Damiao motors) and ARX R5 arms, with VR teleoperation (Quest 2), HDF5 data collection for imitation learning, and ZMQ-based ML inference bridge. Managed via Pixi (conda-based environment).

## Build Commands

```bash
# Install dependencies
pixi install

# Build all packages
pixi run colcon build --symlink-install

# Build a single package
pixi run colcon build --packages-select yam_damiao_controller

# Run tests for a package
pixi run colcon test --packages-select data_collector

# Source the workspace (done automatically by pixi via install/setup.bash activation)
source install/setup.bash
```

All ROS 2 commands must be prefixed with `pixi run` (e.g., `pixi run ros2 launch ...`).

## Running the System

```bash
./bash/activate_can.sh          # Activate CAN interfaces (required first)
./bash/run_cameras.sh           # Launch RealSense cameras
./bash/run_yam.sh               # Launch YAM arm controllers (VR teleop, dual arm)
./bash/run_yam_single.sh        # Launch YAM single arm (left only)
./bash/run_arx.sh               # Launch ARX arm controllers
./bash/run_data_collector.sh    # Start data collection
./bash/infer_yam.sh             # YAM inference mode
./bash/infer_arx.sh             # ARX inference mode
```

Data collection is controlled via ROS 2 services: `/start_collect` (begin recording) and `/stop_collect` (stop and save HDF5).

## Architecture

```
Quest 2 VR → yam_cmd_pub (Python) → /YAM_VR_L, /YAM_VR_R topics
                                          ↓
                                   YamController (C++) → /arm_l_status, /arm_r_status
                                          ↓                        ↓
                                   data_collector (HDF5)    joint_base_node (ZMQ → ML inference)
```

### Two Arm Platforms

- **YAM** (`src/yam/`): Damiao servo motors via CAN, Pinocchio IK/FK, C++ controller at 200Hz
- **ARX R5** (`src/arx/`): CAN bus interface, URDF kinematics, C++ controller at 1kHz

### Key Packages

| Package | Language | Location | Purpose |
|---------|----------|----------|---------|
| `yam_damiao_controller` | C++ | `src/yam/yam_damiao_controller/` | YAM arm motor control + state machine |
| `arx_r5_controller` | C++ | `src/arx/arx_r5_controller/` | ARX arm motor control |
| `yam_cmd_pub` | Python | `src/yam/yam_cmd_pub/` | VR input → arm commands (SE3 transforms) |
| `data_collector` | Python | `src/data_collector/` | Multi-modal HDF5 data recording |
| `inference` | Python | `src/inference/` | ZMQ bridge between ROS 2 and ML models |
| `yam_arm_msg` / `arx5_arm_msg` | ROS msg | `src/msg/` | Custom message definitions |

### YAM Controller State Machine (`YamController.cpp`)

States: SOFT(0) → INIT(1) → PROTECT(2) → G_COMPENSATION(3) → END_CONTROL(4) → JOINT_CONTROL(5) → ZERO(6) → PLANNING(7)

The motor interface uses a **unified MotorControl** design — all 7 motors (6 arm + 1 gripper) share a single CAN socket to avoid race conditions.

### VR Button Mapping (Quest 2)

- **A**: Toggle teleop/pause
- **B**: Homing sequence
- **X/Y**: Start/stop data recording
- **Triggers**: Gripper control (left/right)
- **Left Joystick + LG**: Base linear movement (X/Y)
- **Right Joystick + LG**: Base rotation (angular Z)

### ZMQ Inference Ports

PUB: 6001 (joints), 6002 (base odom), 6003-6005 (cameras L/R/top). SUB: 6006 (joint cmds), 6007 (base vel cmds).

## Custom Message Types

- **YamCmd**: `end_pose[7]` (xyz + quaternion), `joint_pos[6]`, `gripper`, `mode`
- **YamStatus**: `end_pose[7]`, `joint_pos[7]` (6 joints + gripper), `joint_vel[7]`, `joint_cur[7]`
- **RobotCmd**: `end_pos[6]` (xyz + rpy), `joint_pos[6]`, `gripper`, `mode`
- **RobotStatus**: `end_pos[6]`, `joint_pos[7]`, `joint_vel[7]`, `joint_cur[7]`

## Configuration Files

- `src/yam/yam_damiao_controller/config/vr_double_arm.yaml` — Arm control gains (Kp/Kd), CAN interface, control period
- `src/data_collector/config/collect_config.yaml` — Data topics, processors, collection frequency, output directory
- `config/d405.json`, `config/d457.yaml` — RealSense camera settings
- `scripts/d405_*.py` — Camera launch scripts (serial numbers hardcoded)

## Development Notes

- C++ packages use C++17 and CMake. Python packages use `setup.py` with ROS 2 `ament_python`.
- Camera serial numbers are hardcoded in `scripts/d405_wrist.py` and `scripts/d405_high.py` — update when swapping hardware.
- The `pixi.toml` activation script sources `install/setup.bash`, so you must build before running.
- RMW implementation is set to `rmw_fastrtps_cpp` via pixi activation env.
- Data processors in `data_collector/data_processor.py` follow a simple class pattern: subclass `DataProcessor`, implement `process(msg) -> np.array`, then register in `collect_config.yaml`.
- See `CODE.md` for detailed architecture documentation, motor interface API, and extended development guides.
