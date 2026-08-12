# Keyboard-Triggered Data Collection for Master-Slave Mode

## Background

Data collection was previously triggered by VR controller buttons (X/Y on Quest 2). In master-slave teleoperation mode, VR is not used, so we need keyboard-based triggering instead.

## Design

### New Node: `KeyboardCollectTrigger`

- File: `src/data_collector/data_collector/keyboard_trigger.py`
- Uses pynput to globally listen for keyboard events (no terminal focus required)
- **S key** → calls `/start_collect` service
- **D key** → calls `/stop_collect` service
- Tracks recording state to prevent duplicate start/stop calls
- Prints status messages to terminal (started/stopped/saved)

### Updated `collect_config.yaml`

Collects slave arm full state (both arms) + 3 cameras:

| HDF5 Path | Topic | Content |
|---|---|---|
| `/observations/images/cam_left_wrist` | `/cam_left_wrist/color/image_rect_raw` | Left wrist camera |
| `/observations/images/cam_right_wrist` | `/cam_right_wrist/color/image_rect_raw` | Right wrist camera |
| `/observations/images/cam_high` | `/cam_high/color/image_rect_raw` | Top camera |
| `/state/joint_position/left` | `/slave_l_status` | Left slave joint positions [7] |
| `/state/joint_velocity/left` | `/slave_l_status` | Left slave joint velocities [7] |
| `/state/joint_current/left` | `/slave_l_status` | Left slave joint currents [7] |
| `/state/joint_position/right` | `/slave_r_status` | Right slave joint positions [7] |
| `/state/joint_velocity/right` | `/slave_r_status` | Right slave joint velocities [7] |
| `/state/joint_current/right` | `/slave_r_status` | Right slave joint currents [7] |

### Launch / Script Updates

- Add `KeyboardCollectTrigger` node to launch
- Update `run_data_collector.sh` to start both data_collector and keyboard_trigger

### Unchanged

- `data_collector_node.py` — still controlled via `/start_collect` and `/stop_collect` ROS services
- `data_processor.py` — existing processors (JointPosition, JointVelocity, JointCurrent, Image) are reused
