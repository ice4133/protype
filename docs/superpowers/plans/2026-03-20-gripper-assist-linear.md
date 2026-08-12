# Gripper Assist Linear Fade + Auto-Calibration Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the step-function gripper assist in YAM master arms with a linear fade limited to a configurable travel percentage, plus auto-calibration at startup to determine the correct assist torque.

**Architecture:** The calibration runs as a non-blocking sub-state within `executeInit()` after homing. Once calibrated, `executeMasterControl()` uses a linear interpolation from peak assist at closed position to zero at the configured boundary. All parameters are YAML-configurable.

**Tech Stack:** C++17, ROS 2 Humble, Damiao motor CAN interface

**Spec:** `docs/superpowers/specs/2026-03-20-gripper-assist-linear-design.md`

---

## Chunk 1: Header Changes + Parameter Declarations

### Task 1: Update YamController.hpp — replace old members with new ones

**Files:**
- Modify: `src/yam/yam_damiao_controller/include/yam_damiao_controller/YamController.hpp:109-111` (gripper assist members)
- Modify: `src/yam/yam_damiao_controller/include/yam_damiao_controller/YamController.hpp:115-121` (add calibration state)

- [ ] **Step 1: Add constexpr and replace gripper_assist_threshold_ with new members**

In `YamController.hpp`, inside the `YamController` class, replace:

```cpp
    float gripper_torque_limit_;      // max torque for gripper (Nm)
    float gripper_open_assist_;       // constant opening assist torque for master mode (Nm)
    float gripper_assist_threshold_;  // gripper position above which assist is disabled
```

With:

```cpp
    float gripper_torque_limit_;      // max torque for gripper (Nm)
    float gripper_open_assist_;       // manual peak assist torque (Nm). 0 = use auto-calibration
    float gripper_assist_range_ratio_; // fraction of travel with assist (0.0-1.0)
    float gripper_calibration_step_;   // torque increment per cycle during calibration (Nm, positive)
    float gripper_calibration_safety_factor_; // multiplier on measured friction
    static constexpr float GRIPPER_FULL_RANGE = 5.0f; // must match clamp in Damiao_6dof_node.cpp:145

    // Runtime calibration state
    float gripper_peak_assist_ = 0.0f;       // determined by calibration or config
    bool calibration_in_progress_ = false;
    bool calibration_return_phase_ = false;
    float calibration_torque_ = 0.0f;
    rclcpp::Time calibration_start_time_;
    static constexpr float CALIBRATION_TIMEOUT = 5.0f;        // seconds
    static constexpr float CALIBRATION_MOVE_THRESHOLD = 0.05f; // position units
```

- [ ] **Step 2: Verify header compiles**

Run:
```bash
pixi run colcon build --packages-select yam_damiao_controller 2>&1 | tail -20
```

Expected: Build succeeds (errors about removed `gripper_assist_threshold_` usage in .cpp are OK at this stage, we fix those next).

- [ ] **Step 3: Commit**

```bash
git add src/yam/yam_damiao_controller/include/yam_damiao_controller/YamController.hpp
git commit -m "refactor: replace gripper_assist_threshold with calibration members in header"
```

### Task 2: Update parameter declarations in YamController.cpp constructor

**Files:**
- Modify: `src/yam/yam_damiao_controller/src/YamController.cpp:49-51` (parameter declarations)

- [ ] **Step 1: Replace parameter declarations**

In `YamController.cpp` constructor, replace:

```cpp
		gripper_torque_limit_ = this->declare_parameter("gripper_torque_limit", 3.0f);
		gripper_open_assist_ = this->declare_parameter("gripper_open_assist", 0.0f);
		gripper_assist_threshold_ = this->declare_parameter("gripper_assist_threshold", 2.0f);
```

With:

```cpp
		gripper_torque_limit_ = this->declare_parameter("gripper_torque_limit", 3.0f);
		gripper_open_assist_ = this->declare_parameter("gripper_open_assist", 0.0f);
		gripper_assist_range_ratio_ = this->declare_parameter("gripper_assist_range_ratio", 0.4f);
		gripper_calibration_step_ = this->declare_parameter("gripper_calibration_step", 0.001f);
		gripper_calibration_safety_factor_ = this->declare_parameter("gripper_calibration_safety_factor", 1.2f);
```

- [ ] **Step 2: Build to verify**

Run:
```bash
pixi run colcon build --packages-select yam_damiao_controller 2>&1 | tail -20
```

Expected: Build may still fail due to `gripper_assist_threshold_` usage in `executeMasterControl()`. That's fixed in Chunk 2.

- [ ] **Step 3: Commit**

```bash
git add src/yam/yam_damiao_controller/src/YamController.cpp
git commit -m "refactor: replace gripper_assist_threshold parameter with calibration params"
```

---

## Chunk 2: Linear Fade Logic in executeMasterControl()

### Task 3: Replace step-function assist with linear fade

**Files:**
- Modify: `src/yam/yam_damiao_controller/src/YamController.cpp:591-596` (executeMasterControl gripper assist)

- [ ] **Step 1: Replace the assist torque calculation**

In `executeMasterControl()`, replace:

```cpp
        // Gripper in master mode: assist torque only in the hard-to-open range
        // Beyond gripper_assist_threshold_, no assist (fully passive)
        float gripper_tau = 0.0f;
        if (gripper_positions_[0] < gripper_assist_threshold_) {
            gripper_tau = gripper_open_assist_;
        }
```

With:

```cpp
        // Gripper in master mode: linear fade assist from peak at closed to zero at assist boundary
        float gripper_tau = 0.0f;
        float assist_end = GRIPPER_FULL_RANGE * gripper_assist_range_ratio_;
        if (assist_end > 0.0f) {
            float pos = std::max(0.0f, gripper_positions_[0]);
            if (pos < assist_end) {
                float ratio = 1.0f - pos / assist_end;
                gripper_tau = gripper_peak_assist_ * ratio;
            }
        }
```

- [ ] **Step 2: Build and verify**

Run:
```bash
pixi run colcon build --packages-select yam_damiao_controller 2>&1 | tail -20
```

Expected: Build succeeds. No references to `gripper_assist_threshold_` remain.

- [ ] **Step 3: Commit**

```bash
git add src/yam/yam_damiao_controller/src/YamController.cpp
git commit -m "feat: replace step-function gripper assist with linear fade"
```

---

## Chunk 3: Auto-Calibration State Machine in executeInit()

### Task 4: Add calibration logic after homing in executeInit()

**Files:**
- Modify: `src/yam/yam_damiao_controller/src/YamController.cpp:360-394` (post-homing transition in executeInit)

- [ ] **Step 1: Add calibration sub-state after homing completes**

In `executeInit()`, the current code after homing is:

```cpp
                homing_in_progress_ = false;
                RCLCPP_INFO(this->get_logger(), "Homing complete - all joints at home position");
            }
            // ... (homing motor commands in else block)
        }
        else {
            // Init finished — route based on control type
            if (arm_control_type_ == "master") {
                transitionToState(ArmState::G_COMPENSATION);
            } else {
                transitionToState(ArmState::JOINT_CONTROL);
            }
            return;
        }
```

Replace the `else` block (the `homing_in_progress_ == false` path, lines 387-395) with:

```cpp
        else {
            // Init finished — run calibration for master, then transition
            if (arm_control_type_ == "master" && std::abs(gripper_open_assist_) < 1e-6f) {
                // Auto-calibration: gripper_open_assist is zero, need to measure friction
                if (!calibration_in_progress_) {
                    // Start calibration
                    calibration_in_progress_ = true;
                    calibration_return_phase_ = false;
                    calibration_torque_ = 0.0f;
                    calibration_start_time_ = this->now();
                    RCLCPP_INFO(this->get_logger(),
                        "Starting gripper assist calibration (step=%.4f, safety=%.1f, range=%.0f%%, limit=%.2f Nm)",
                        gripper_calibration_step_, gripper_calibration_safety_factor_,
                        gripper_assist_range_ratio_ * 100.0f, gripper_torque_limit_);
                }

                float elapsed = (this->now() - calibration_start_time_).seconds();

                if (!calibration_return_phase_) {
                    // RAMP_UP phase: increment torque each cycle until gripper moves
                    if (elapsed > CALIBRATION_TIMEOUT) {
                        RCLCPP_WARN(this->get_logger(),
                            "Gripper calibration timeout (%.1fs), using torque limit as fallback", CALIBRATION_TIMEOUT);
                        gripper_peak_assist_ = -gripper_torque_limit_ * gripper_calibration_safety_factor_;
                        calibration_return_phase_ = true;
                    } else {
                        calibration_torque_ -= std::abs(gripper_calibration_step_); // negative = opening direction

                        if (std::abs(calibration_torque_) >= gripper_torque_limit_) {
                            RCLCPP_WARN(this->get_logger(),
                                "Gripper calibration hit torque limit (%.3f Nm), using as fallback",
                                gripper_torque_limit_);
                            gripper_peak_assist_ = -gripper_torque_limit_ * gripper_calibration_safety_factor_;
                            calibration_return_phase_ = true;
                        } else if (std::abs(gripper_positions_[0] - gripper_homes_[0]) > CALIBRATION_MOVE_THRESHOLD) {
                            // Friction overcome
                            gripper_peak_assist_ = calibration_torque_ * gripper_calibration_safety_factor_;
                            float assist_end = GRIPPER_FULL_RANGE * gripper_assist_range_ratio_;
                            RCLCPP_INFO(this->get_logger(),
                                "Gripper assist calibrated: friction=%.4f Nm, peak_assist=%.4f Nm, assist_end=%.2f",
                                calibration_torque_, gripper_peak_assist_, assist_end);
                            calibration_return_phase_ = true;
                        } else {
                            // Apply calibration torque: hold arm joints at home, apply torque to gripper
                            interfaces_ptr_damiao_->setRobotPosition(
                                joint_homes_,
                                std::vector<float>(arm_dof, 0.0f),
                                std::vector<float>(arm_dof, go_home_kp_),
                                std::vector<float>(arm_dof, go_home_kd_),
                                std::vector<float>(arm_dof, 0.0f),
                                {gripper_positions_[0]},  // hold current pos
                                std::vector<float>(gripper_dof, 0.0f),
                                std::vector<float>(gripper_dof, 0.0f),  // kp=0 (torque mode)
                                std::vector<float>(gripper_dof, gripper_velocity_gain_),
                                {calibration_torque_}  // feedforward torque
                            );
                            return;
                        }
                    }
                }

                if (calibration_return_phase_) {
                    // RETURN phase: move gripper back to home
                    float return_error = std::abs(gripper_positions_[0] - gripper_homes_[0]);
                    if (return_error < 0.02f) {
                        // Calibration complete
                        calibration_in_progress_ = false;
                        RCLCPP_INFO(this->get_logger(), "Gripper calibration complete, transitioning to master mode");
                        transitionToState(ArmState::G_COMPENSATION);
                        return;
                    }

                    // Return to home using go_home gains
                    float return_kp = go_home_kp_;
                    if (return_error > 0.01f && return_kp * return_error > gripper_torque_limit_) {
                        return_kp = gripper_torque_limit_ / return_error;
                    }
                    interfaces_ptr_damiao_->setRobotPosition(
                        joint_homes_,
                        std::vector<float>(arm_dof, 0.0f),
                        std::vector<float>(arm_dof, go_home_kp_),
                        std::vector<float>(arm_dof, go_home_kd_),
                        std::vector<float>(arm_dof, 0.0f),
                        gripper_homes_,
                        std::vector<float>(gripper_dof, 0.0f),
                        std::vector<float>(gripper_dof, return_kp),
                        std::vector<float>(gripper_dof, gripper_velocity_gain_),
                        std::vector<float>(gripper_dof, 0.0f)
                    );
                    return;
                }
            } else {
                // Manual override or non-master: use config value directly
                if (arm_control_type_ == "master") {
                    gripper_peak_assist_ = gripper_open_assist_;
                    float assist_end = GRIPPER_FULL_RANGE * gripper_assist_range_ratio_;
                    RCLCPP_INFO(this->get_logger(),
                        "Gripper assist manual override: peak_assist=%.4f Nm, assist_end=%.2f",
                        gripper_peak_assist_, assist_end);
                }

                if (arm_control_type_ == "master") {
                    transitionToState(ArmState::G_COMPENSATION);
                } else {
                    transitionToState(ArmState::JOINT_CONTROL);
                }
                return;
            }
        }
```

- [ ] **Step 2: Build and verify**

Run:
```bash
pixi run colcon build --packages-select yam_damiao_controller 2>&1 | tail -20
```

Expected: Build succeeds with no errors.

- [ ] **Step 3: Commit**

```bash
git add src/yam/yam_damiao_controller/src/YamController.cpp
git commit -m "feat: add non-blocking gripper assist auto-calibration in executeInit"
```

---

## Chunk 4: YAML Config Updates

### Task 5: Update master_slave.yaml

**Files:**
- Modify: `src/yam/yam_damiao_controller/config/master_slave.yaml:18-19` (master_l gripper assist)
- Modify: `src/yam/yam_damiao_controller/config/master_slave.yaml:46-47` (master_r gripper assist)

- [ ] **Step 1: Update master_l config**

In `master_slave.yaml`, replace the master_l gripper_open_assist line:

```yaml
    gripper_open_assist: -0.05
```

With:

```yaml
    gripper_open_assist: 0.0
    gripper_assist_range_ratio: 0.4
    gripper_calibration_step: 0.001
    gripper_calibration_safety_factor: 1.2
```

- [ ] **Step 2: Update master_r config**

Same change for master_r section — replace:

```yaml
    gripper_open_assist: -0.05
```

With:

```yaml
    gripper_open_assist: 0.0
    gripper_assist_range_ratio: 0.4
    gripper_calibration_step: 0.001
    gripper_calibration_safety_factor: 1.2
```

- [ ] **Step 3: Remove old comment about gripper_open_assist sign**

Remove the comment block between master_l and master_r (lines 29-31):

```yaml
# 注意: gripper_open_assist 的正负号控制方向
# 如果爪子自动张开说明方向对了，调小数值
# 如果爪子更难打开说明方向反了，改成相反的正负号
```

Replace with:

```yaml
# gripper_open_assist: 0.0 = auto-calibration at startup
# gripper_open_assist: non-zero = manual override (negative = opening direction)
# gripper_assist_range_ratio: fraction of travel with assist (0.4 = first 40%)
```

- [ ] **Step 4: Commit**

```bash
git add src/yam/yam_damiao_controller/config/master_slave.yaml
git commit -m "config: update master_slave.yaml for auto-calibration gripper assist"
```

### Task 6: Update master_slave_infer.yaml

**Files:**
- Modify: `src/yam/yam_damiao_controller/config/master_slave_infer.yaml` (master_l and master_r sections)

- [ ] **Step 1: Update master_l config**

Replace:

```yaml
    gripper_open_assist: -0.18
```

With:

```yaml
    gripper_open_assist: 0.0
    gripper_assist_range_ratio: 0.4
    gripper_calibration_step: 0.001
    gripper_calibration_safety_factor: 1.2
```

- [ ] **Step 2: Update master_r config**

Same change for master_r section.

- [ ] **Step 3: Commit**

```bash
git add src/yam/yam_damiao_controller/config/master_slave_infer.yaml
git commit -m "config: update master_slave_infer.yaml for auto-calibration gripper assist"
```

---

## Chunk 5: Final Build + Verification

### Task 7: Full build and verification

**Files:** None (verification only)

- [ ] **Step 1: Clean build**

Run:
```bash
pixi run colcon build --packages-select yam_damiao_controller 2>&1 | tail -30
```

Expected: Build succeeds with no errors or warnings related to gripper assist.

- [ ] **Step 2: Grep for any remaining references to gripper_assist_threshold**

Run:
```bash
grep -r "gripper_assist_threshold" src/yam/
```

Expected: No matches found. All references to the old parameter have been removed.

- [ ] **Step 3: Verify parameter consistency**

Check that all YAML configs and C++ defaults are consistent:
- `gripper_open_assist` defaults to 0.0 in C++ and all master YAML configs
- `gripper_assist_range_ratio` defaults to 0.4
- `gripper_calibration_step` defaults to 0.001
- `gripper_calibration_safety_factor` defaults to 1.2
