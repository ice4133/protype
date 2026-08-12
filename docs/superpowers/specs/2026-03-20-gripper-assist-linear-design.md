# Gripper Assist Linear Fade + Auto-Calibration Design

## Problem

The YAM master arm gripper is difficult to open by hand. The current assist implementation applies a constant torque (`gripper_open_assist`) below a fixed position threshold (`gripper_assist_threshold`), creating an abrupt on/off transition. The user wants:

1. Assist force limited to a configurable percentage of gripper travel (default 40%, from closed end)
2. Smooth linear fade from max assist at closed position to zero at the boundary
3. Auto-calibration at startup to determine the right assist force
4. All parameters configurable via YAML

## Current Implementation

File: `src/yam/yam_damiao_controller/src/YamController.cpp`, lines 591-596

```cpp
float gripper_tau = 0.0f;
if (gripper_positions_[0] < gripper_assist_threshold_) {
    gripper_tau = gripper_open_assist_;
}
```

- `gripper_open_assist_`: constant torque (currently -0.05 Nm in master_slave.yaml, -0.18 Nm in master_slave_infer.yaml)
- `gripper_assist_threshold_`: absolute position cutoff (default 2.0, declared in C++ only, not in YAML configs)
- Gripper position range: 0.0 (closed) to 5.0 (fully open), clamped in `Damiao_6dof_node.cpp:145`
- Gripper torque feedback available via `gripper_torques_[0]` from `getGripperCurrent()`

## Design

### 1. Auto-Calibration at Startup (Non-Blocking State Machine)

Calibration runs as a sub-state within `executeInit()`, after homing completes and before transitioning to G_COMPENSATION. It is implemented as a **non-blocking state machine** — each call from `controlLoop()` performs one incremental step, consistent with the existing homing pattern.

Calibration only runs when `arm_control_type == "master"`. For slave or other control types, calibration is skipped entirely.

**State machine flow:**

```
CALIBRATING_RAMP_UP:
    // Each control cycle: increment torque and check for movement
    calibration_torque_ -= abs(gripper_calibration_step_)   // negative = opening direction
    if abs(calibration_torque_) >= gripper_torque_limit_:
        // Hit safety limit without detecting movement
        log WARNING "Calibration hit torque limit, using limit as peak assist"
        gripper_peak_assist_ = -gripper_torque_limit_ * gripper_calibration_safety_factor_
        transition to CALIBRATING_RETURN

    apply calibration_torque_ to gripper motor (via setRobotPosition with tau feedforward)

    if abs(gripper_positions_[0] - gripper_homes_[0]) > movement_threshold (0.05):
        // Friction overcome — record the torque
        gripper_peak_assist_ = calibration_torque_ * gripper_calibration_safety_factor_
        log INFO "Gripper assist calibrated: friction=X Nm, peak_assist=Y Nm, assist_end=Z"
        transition to CALIBRATING_RETURN

CALIBRATING_RETURN:
    // Return gripper to home using go_home_kp_/go_home_kd_ gains (same as homing)
    apply position control: target=gripper_homes_[0], kp=go_home_kp_, kd=go_home_kd_
    if abs(gripper_positions_[0] - gripper_homes_[0]) < 0.02:
        calibration complete, transition to G_COMPENSATION
```

**Torque sign convention:** `gripper_calibration_step_` is always specified as a positive magnitude in config. The code applies it as negative (opening direction) internally. This avoids user confusion about sign.

**Timeout:** Calibration aborts after 5 seconds (configurable). On timeout, log a warning and use `gripper_torque_limit_ * safety_factor` as fallback, same as hitting the torque limit.

**Priority logic for determining `gripper_peak_assist_`:**
1. If `gripper_open_assist != 0.0` in config → use it directly as peak assist (manual override, skip calibration)
2. Else if `arm_control_type == "master"` → run calibration to determine peak assist
3. Else → `peak_assist = 0.0` (no assist for non-master modes)

The `gripper_assist_calibration` parameter is removed to avoid redundancy with the `gripper_open_assist` override.

### 2. Linear Fade Assist Torque

```
GRIPPER_FULL_RANGE = 5.0  // constexpr, matching clamp in Damiao_6dof_node.cpp:145

assist_end = GRIPPER_FULL_RANGE * gripper_assist_range_ratio_

pos = std::max(0.0f, gripper_positions_[0])   // guard against negative encoder drift

if assist_end <= 0.0f:
    gripper_tau = 0.0    // guard: ratio=0 means no assist
elif pos < assist_end:
    ratio = 1.0 - pos / assist_end      // 1.0 at closed, 0.0 at boundary
    gripper_tau = gripper_peak_assist_ * ratio
else:
    gripper_tau = 0.0
```

With default values (`GRIPPER_FULL_RANGE=5.0`, `gripper_assist_range_ratio=0.4`, calibrated `peak_assist`):
- pos = 0.0 (closed): tau = peak_assist (100% assist)
- pos = 1.0 (20%): tau = 50% of peak_assist
- pos = 2.0 (40%): tau = 0.0
- pos > 2.0: tau = 0.0

### 3. Parameter Changes

| Parameter | Old | New | Description |
|-----------|-----|-----|-------------|
| `gripper_open_assist` | -0.05 (constant) | 0.0 (default=auto) | Manual peak assist torque. Non-zero = skip calibration, use this value directly |
| `gripper_assist_threshold` | 2.0 | **removed** | Replaced by ratio-based calculation (C++ declaration removed) |
| `gripper_assist_range_ratio` | *new* | 0.4 | Fraction of travel with assist (0.0-1.0), configurable in YAML |
| `gripper_calibration_step` | *new* | 0.001 | Torque magnitude increment per control cycle during calibration (Nm, always positive) |
| `gripper_calibration_safety_factor` | *new* | 1.2 | Multiplier on measured friction to get peak assist |

`gripper_full_range` is a `constexpr float GRIPPER_FULL_RANGE = 5.0f` in the header, matching the clamp in `Damiao_6dof_node.cpp:145`. Not a YAML parameter — single source of truth.

### 4. Files to Modify

1. **`src/yam/yam_damiao_controller/src/YamController.cpp`**
   - Add non-blocking calibration logic in `executeInit()` (after homing, before G_COMPENSATION transition)
   - Replace step-function assist logic in `executeMasterControl()` with linear fade (update comments at line 591-592)
   - Remove `gripper_assist_threshold_` parameter declaration, add new parameter declarations
   - Store calibrated peak assist in `gripper_peak_assist_`

2. **`src/yam/yam_damiao_controller/include/yam_damiao_controller/YamController.hpp`**
   - Remove `gripper_assist_threshold_` member
   - Add `constexpr float GRIPPER_FULL_RANGE = 5.0f`
   - Add new members: `gripper_assist_range_ratio_`, `gripper_calibration_step_`, `gripper_calibration_safety_factor_`, `gripper_peak_assist_` (runtime)
   - Add calibration state tracking: `bool calibration_in_progress_`, `float calibration_torque_`, `rclcpp::Time calibration_start_time_`
   - Add calibration sub-state enum or bool for RAMP_UP vs RETURN phases

3. **`src/yam/yam_damiao_controller/config/master_slave.yaml`**
   - master_l and master_r: add `gripper_assist_range_ratio: 0.4`, `gripper_calibration_step: 0.001`, `gripper_calibration_safety_factor: 1.2`
   - Change `gripper_open_assist: 0.0` (enable auto-calibration by default)
   - Slave node configs unchanged — gripper assist is master-mode only

4. **`src/yam/yam_damiao_controller/config/master_slave_infer.yaml`**
   - Same changes as master_slave.yaml for master nodes
   - Change `gripper_open_assist` from -0.18 to 0.0 for auto-calibration
   - Slave node configs unchanged

## Risks

- **Calibration time**: Adds ~1-2 seconds to startup. Acceptable for a one-time procedure.
- **Mechanical variance**: Friction may differ between left and right grippers — calibration handles this automatically since each arm calibrates independently.
- **Negative encoder values**: Guarded by `std::max(0.0f, pos)` before ratio calculation to prevent assist torque exceeding peak value.
- **Division by zero**: Guarded by checking `assist_end <= 0.0f` before division.
- **Calibration safety**: Torque is bounded by `gripper_torque_limit_` during calibration. If limit is reached without movement, a warning is logged and the limit value is used as fallback.
- **Calibration timeout**: 5-second timeout prevents indefinite calibration if gripper is stuck. Falls back to torque limit value.
- **Gravity bias**: Calibration measures friction at the home position orientation. If gravity bias on the gripper varies significantly with arm pose during teleoperation, the calibrated value may be inaccurate. The safety_factor (1.2x) provides partial compensation.
- **master_slave_infer.yaml discrepancy**: Current value is -0.18 Nm vs -0.05 Nm in master_slave.yaml. With auto-calibration, both will be determined automatically per-arm.
