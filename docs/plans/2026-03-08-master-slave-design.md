# 4-Arm Master-Slave Teleoperation Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add master-slave teleoperation to YAM arms — 2 master arms (gravity compensation, hand-guided) control 2 slave arms (joint following), with configurable bilateral force feedback.

**Architecture:** Each arm runs the same `YamController` executable with a different `arm_control_type` parameter (`master` or `slave`). Master publishes YamStatus; slave subscribes to it and follows joint positions. Bilateral force feedback is optional — when enabled, master subscribes to slave's YamStatus and adds `Kp*(q_slave - q_master)` torque on top of gravity compensation. All changes are additive branches alongside the existing `normal` control type.

**Tech Stack:** C++17, ROS 2 Humble, Pinocchio (gravity compensation), colcon/CMake build

**Existing code constraint:** Do NOT modify any existing logic in the `normal` control type branch. All new code goes in new `else if` branches or new methods.

---

### Task 1: Create config file `config/master_slave.yaml`

**Files:**
- Create: `src/yam/yam_damiao_controller/config/master_slave.yaml`

**Step 1: Create the config file**

```yaml
# Master-Slave 4-arm configuration
# Master arms: gravity compensation (hand-guided), optional bilateral force feedback
# Slave arms: joint position following from master status

/master_l:
  ros__parameters:
    arm_can_id: can_master_l
    arm_control_type: master
    arm_pub_topic_name: master_l_status
    arm_sub_topic_name: master_l_cmd
    arm_control_period: 0.004
    arm_kp_1_3: 0.0
    arm_kp_4_6: 0.0
    arm_kd_1_3: 0.0
    arm_kd_4_6: 0.0
    gripper_position_gain: 20.0
    gripper_velocity_gain: 1.0
    gripper_torque_limit: 3.0
    go_home_kp: 25.0
    go_home_kd: 0.5
    bilateral_enabled: true
    bilateral_kp_1_3: 5.0
    bilateral_kp_4_6: 2.0
    bilateral_kd_1_3: 0.2
    bilateral_kd_4_6: 0.1
    bilateral_slave_topic: slave_l_status

/master_r:
  ros__parameters:
    arm_can_id: can_master_r
    arm_control_type: master
    arm_pub_topic_name: master_r_status
    arm_sub_topic_name: master_r_cmd
    arm_control_period: 0.004
    arm_kp_1_3: 0.0
    arm_kp_4_6: 0.0
    arm_kd_1_3: 0.0
    arm_kd_4_6: 0.0
    gripper_position_gain: 20.0
    gripper_velocity_gain: 1.0
    gripper_torque_limit: 3.0
    go_home_kp: 25.0
    go_home_kd: 0.5
    bilateral_enabled: true
    bilateral_kp_1_3: 5.0
    bilateral_kp_4_6: 2.0
    bilateral_kd_1_3: 0.2
    bilateral_kd_4_6: 0.1
    bilateral_slave_topic: slave_r_status

/slave_l:
  ros__parameters:
    arm_can_id: can_slave_l
    arm_control_type: slave
    arm_pub_topic_name: slave_l_status
    arm_sub_topic_name: master_l_status
    arm_control_period: 0.004
    arm_kp_1_3: 120.0
    arm_kp_4_6: 30.0
    arm_kd_1_3: 2.0
    arm_kd_4_6: 1.0
    gripper_position_gain: 20.0
    gripper_velocity_gain: 1.0
    gripper_torque_limit: 3.0
    go_home_kp: 25.0
    go_home_kd: 0.5

/slave_r:
  ros__parameters:
    arm_can_id: can_slave_r
    arm_control_type: slave
    arm_pub_topic_name: slave_r_status
    arm_sub_topic_name: master_r_status
    arm_control_period: 0.004
    arm_kp_1_3: 120.0
    arm_kp_4_6: 30.0
    arm_kd_1_3: 2.0
    arm_kd_4_6: 1.0
    gripper_position_gain: 20.0
    gripper_velocity_gain: 1.0
    gripper_torque_limit: 3.0
    go_home_kp: 25.0
    go_home_kd: 0.5
```

**Step 2: Verify the file is in the right directory**

Run: `ls src/yam/yam_damiao_controller/config/`
Expected: `master_slave.yaml` alongside `vr_double_arm.yaml`

---

### Task 2: Add master/slave member variables to `YamController.hpp`

**Files:**
- Modify: `src/yam/yam_damiao_controller/include/yam_damiao_controller/YamController.hpp`

**Step 1: Add new public method declarations**

After `void executeGravityCompensation();` (line 52), add:

```cpp
    void executeMasterControl();
    void slaveStatusCallback(const yam_arm_msg::msg::YamStatus::SharedPtr msg);
```

**Step 2: Add new private member variables**

After line 142 (`rclcpp::TimerBase::SharedPtr state_timer_;`), add:

```cpp

    // Master-slave members
    std::string arm_control_type_;
    rclcpp::Subscription<yam_arm_msg::msg::YamStatus>::SharedPtr slave_status_subscriber_;

    // Bilateral force feedback
    bool bilateral_enabled_ = false;
    float bilateral_kp_1_3_ = 0.0f;
    float bilateral_kp_4_6_ = 0.0f;
    float bilateral_kd_1_3_ = 0.0f;
    float bilateral_kd_4_6_ = 0.0f;
    std::vector<float> slave_joint_positions_;
    std::vector<float> slave_joint_velocities_;
    rclcpp::Time slave_last_received_time_;
    bool slave_data_valid_ = false;
    static constexpr double SLAVE_DATA_TIMEOUT = 0.1;  // seconds
```

**Step 3: Build to verify header compiles**

Run: `pixi run colcon build --packages-select yam_damiao_controller 2>&1 | tail -5`
Expected: build succeeds (new members declared but unused is OK)

**Step 4: Commit**

```bash
git add src/yam/yam_damiao_controller/include/yam_damiao_controller/YamController.hpp
git commit -m "feat: add master/slave member declarations to YamController.hpp"
```

---

### Task 3: Add slave mode to `YamController.cpp`

**Files:**
- Modify: `src/yam/yam_damiao_controller/src/YamController.cpp`

Slave mode is simpler — do it first.

**Step 1: Store `arm_control_type` and add slave branch in constructor**

In the constructor, change line 12 from:
```cpp
		std::string arm_control_type = this->declare_parameter("arm_control_type", "normal");
```
to:
```cpp
		arm_control_type_ = this->declare_parameter("arm_control_type", "normal");
		std::string arm_control_type = arm_control_type_;
```

Then, after the closing `}` of `if (arm_control_type == "normal")` block (line 92) and before the `else {` on line 93, add:

```cpp
		else if (arm_control_type == "slave") {
			// Slave mode: subscribe to master YamStatus, follow joint positions
			robot_status_publisher_ = this->create_publisher<yam_arm_msg::msg::YamStatus>(
				this->declare_parameter("arm_pub_topic_name", "arm_status"), 1);

			state_timer_ = this->create_wall_timer(std::chrono::milliseconds(3),
				std::bind(&YamController::publishState, this));

			transitionToState(ArmState::INIT);
			control_timer_ = this->create_wall_timer(
				std::chrono::duration<double>(dt_),
				std::bind(&YamController::controlLoop, this));

			// Subscribe to master's YamStatus (not YamCmd)
			auto master_topic = this->declare_parameter("arm_sub_topic_name", "master_status");
			slave_status_subscriber_ = this->create_subscription<yam_arm_msg::msg::YamStatus>(
				master_topic, 1,
				[this](const yam_arm_msg::msg::YamStatus::SharedPtr msg) {
					if (current_state_ == ArmState::SOFT || current_state_ == ArmState::INIT) return;

					if (current_state_ != ArmState::JOINT_CONTROL) {
						initialization_flag_ = false;
						transitionToState(ArmState::JOINT_CONTROL);
					}
					for (int i = 0; i < 6; i++) {
						target_joint_positions_[i] = static_cast<float>(msg->joint_pos[i]);
					}
					target_gripper_position_ = static_cast<float>(msg->joint_pos[6]);
				});

			RCLCPP_INFO(this->get_logger(), "Slave mode: following master on topic '%s'", master_topic.c_str());
		}
```

**Step 2: Modify `executeInit()` to auto-transition slave to JOINT_CONTROL**

In `executeInit()`, find the `else` block at line 308-312:
```cpp
        else {
            // Init finished
            transitionToState(ArmState::JOINT_CONTROL);
            return;
        }
```

This already transitions to `JOINT_CONTROL` after homing, which is what slave needs. No change needed — slave will wait in JOINT_CONTROL for master data to arrive via the subscription callback above.

**Step 3: Build and verify**

Run: `pixi run colcon build --packages-select yam_damiao_controller 2>&1 | tail -5`
Expected: build succeeds

**Step 4: Commit**

```bash
git add src/yam/yam_damiao_controller/src/YamController.cpp
git commit -m "feat: add slave control type to YamController"
```

---

### Task 4: Add master mode to `YamController.cpp`

**Files:**
- Modify: `src/yam/yam_damiao_controller/src/YamController.cpp`

**Step 1: Add master branch in constructor**

After the slave `else if` block added in Task 3, before the final `else {` error block, add:

```cpp
		else if (arm_control_type == "master") {
			// Master mode: gravity compensation + optional bilateral force feedback
			robot_status_publisher_ = this->create_publisher<yam_arm_msg::msg::YamStatus>(
				this->declare_parameter("arm_pub_topic_name", "arm_status"), 1);

			state_timer_ = this->create_wall_timer(std::chrono::milliseconds(3),
				std::bind(&YamController::publishState, this));

			transitionToState(ArmState::INIT);
			control_timer_ = this->create_wall_timer(
				std::chrono::duration<double>(dt_),
				std::bind(&YamController::controlLoop, this));

			// Bilateral force feedback (optional)
			bilateral_enabled_ = this->declare_parameter("bilateral_enabled", false);
			if (bilateral_enabled_) {
				bilateral_kp_1_3_ = this->declare_parameter("bilateral_kp_1_3", 5.0f);
				bilateral_kp_4_6_ = this->declare_parameter("bilateral_kp_4_6", 2.0f);
				bilateral_kd_1_3_ = this->declare_parameter("bilateral_kd_1_3", 0.2f);
				bilateral_kd_4_6_ = this->declare_parameter("bilateral_kd_4_6", 0.1f);

				slave_joint_positions_ = std::vector<float>(arm_dof, 0.0f);
				slave_joint_velocities_ = std::vector<float>(arm_dof, 0.0f);
				slave_last_received_time_ = this->get_clock()->now();

				auto slave_topic = this->declare_parameter("bilateral_slave_topic", "slave_status");
				slave_status_subscriber_ = this->create_subscription<yam_arm_msg::msg::YamStatus>(
					slave_topic, 1,
					std::bind(&YamController::slaveStatusCallback, this, std::placeholders::_1));

				RCLCPP_INFO(this->get_logger(),
					"Master mode: bilateral feedback ON, subscribing to '%s'", slave_topic.c_str());
			} else {
				RCLCPP_INFO(this->get_logger(), "Master mode: bilateral feedback OFF");
			}

			// Declare but don't use arm_sub_topic_name (needed for parameter consistency)
			this->declare_parameter("arm_sub_topic_name", "unused");
		}
```

**Step 2: Add `slaveStatusCallback` method**

After `executeGravityCompensation()` (after line 471), add:

```cpp
    void YamController::slaveStatusCallback(const yam_arm_msg::msg::YamStatus::SharedPtr msg) {
        for (int i = 0; i < 6; i++) {
            slave_joint_positions_[i] = static_cast<float>(msg->joint_pos[i]);
            slave_joint_velocities_[i] = static_cast<float>(msg->joint_vel[i]);
        }
        slave_last_received_time_ = this->get_clock()->now();
        slave_data_valid_ = true;
    }
```

**Step 3: Add `executeMasterControl` method**

After `slaveStatusCallback`, add:

```cpp
    void YamController::executeMasterControl() {
        Eigen::VectorXd current_q = Eigen::Map<Eigen::VectorXf>(
            joint_positions_.data(), joint_positions_.size()
        ).cast<double>();

        yam_solver_ptr->setCurrentConfig(current_q);
        Eigen::VectorXf gravity_tau = yam_solver_ptr->gravityCompensationCurrent().cast<float>();
        std::vector<float> target_tau(gravity_tau.data(), gravity_tau.data() + gravity_tau.size());

        // Add bilateral force feedback if enabled and data is fresh
        if (bilateral_enabled_ && slave_data_valid_) {
            double age = (this->get_clock()->now() - slave_last_received_time_).seconds();
            if (age < SLAVE_DATA_TIMEOUT) {
                for (size_t i = 0; i < arm_dof; i++) {
                    float kp = (i < 3) ? bilateral_kp_1_3_ : bilateral_kp_4_6_;
                    float kd = (i < 3) ? bilateral_kd_1_3_ : bilateral_kd_4_6_;
                    float pos_error = slave_joint_positions_[i] - joint_positions_[i];
                    float vel_error = slave_joint_velocities_[i] - joint_velocities_[i];
                    target_tau[i] += kp * pos_error + kd * vel_error;
                }
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Bilateral feedback: slave data stale (%.3f s), using gravity comp only", age);
            }
        }

        interfaces_ptr_damiao_->setRobotPosition(
            {joint_positions_[0], joint_positions_[1], joint_positions_[2],
             joint_positions_[3], joint_positions_[4], joint_positions_[5]},
            std::vector<float>(arm_dof, 0.0f),
            std::vector<float>(arm_dof, 0.0f),
            std::vector<float>(arm_dof, 0.0f),
            target_tau,
            {gripper_positions_[0]},
            std::vector<float>(gripper_dof, 0.0f),
            std::vector<float>(gripper_dof, gripper_position_gain_),
            std::vector<float>(gripper_dof, gripper_velocity_gain_),
            std::vector<float>(gripper_dof, 0.0f)
        );
    }
```

**Step 4: Modify `executeInit()` to route master to G_COMPENSATION**

In `executeInit()`, change the post-homing transition (lines 308-312) from:

```cpp
        else {
            // Init finished
            transitionToState(ArmState::JOINT_CONTROL);
            return;
        }
```

to:

```cpp
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

**Step 5: Add `executeMasterControl` to state machine**

In `runStateMachine()`, find the `G_COMPENSATION` case (lines 216-219):

```cpp
        case ArmState::G_COMPENSATION:
        checkError();
        executeGravityCompensation();
        break;
```

Change to:

```cpp
        case ArmState::G_COMPENSATION:
        checkError();
        if (arm_control_type_ == "master") {
            executeMasterControl();
        } else {
            executeGravityCompensation();
        }
        break;
```

**Step 6: Build and verify**

Run: `pixi run colcon build --packages-select yam_damiao_controller 2>&1 | tail -5`
Expected: build succeeds

**Step 7: Commit**

```bash
git add src/yam/yam_damiao_controller/src/YamController.cpp
git commit -m "feat: add master control type with optional bilateral force feedback"
```

---

### Task 5: Create launch file and shell script

**Files:**
- Create: `src/yam/yam_damiao_controller/launch/master_slave.launch.py`
- Create: `bash/run_master_slave.sh`

**Step 1: Create the launch file**

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('yam_damiao_controller'),
        'config',
        'master_slave.yaml'
    )

    use_master_l_arg = DeclareLaunchArgument(
        'use_master_l', default_value='true',
        description='Launch left master arm')
    use_master_r_arg = DeclareLaunchArgument(
        'use_master_r', default_value='true',
        description='Launch right master arm')
    use_slave_l_arg = DeclareLaunchArgument(
        'use_slave_l', default_value='true',
        description='Launch left slave arm')
    use_slave_r_arg = DeclareLaunchArgument(
        'use_slave_r', default_value='true',
        description='Launch right slave arm')

    master_l_node = Node(
        package='yam_damiao_controller',
        executable='YamController',
        name='master_l',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(LaunchConfiguration('use_master_l'))
    )

    master_r_node = Node(
        package='yam_damiao_controller',
        executable='YamController',
        name='master_r',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(LaunchConfiguration('use_master_r'))
    )

    slave_l_node = Node(
        package='yam_damiao_controller',
        executable='YamController',
        name='slave_l',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(LaunchConfiguration('use_slave_l'))
    )

    slave_r_node = Node(
        package='yam_damiao_controller',
        executable='YamController',
        name='slave_r',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(LaunchConfiguration('use_slave_r'))
    )

    return LaunchDescription([
        use_master_l_arg,
        use_master_r_arg,
        use_slave_l_arg,
        use_slave_r_arg,
        master_l_node,
        master_r_node,
        slave_l_node,
        slave_r_node,
    ])
```

**Step 2: Create the shell script**

```bash
#!/bin/bash
pixi run ros2 launch yam_damiao_controller master_slave.launch.py
```

**Step 3: Make shell script executable**

Run: `chmod +x bash/run_master_slave.sh`

**Step 4: Build to install new config and launch files**

Run: `pixi run colcon build --packages-select yam_damiao_controller 2>&1 | tail -5`
Expected: build succeeds, new files installed to `install/yam_damiao_controller/share/`

**Step 5: Verify installed files**

Run: `ls install/yam_damiao_controller/share/yam_damiao_controller/config/master_slave.yaml install/yam_damiao_controller/share/yam_damiao_controller/launch/master_slave.launch.py`
Expected: both files exist

**Step 6: Commit**

```bash
git add src/yam/yam_damiao_controller/launch/master_slave.launch.py
git add src/yam/yam_damiao_controller/config/master_slave.yaml
git add bash/run_master_slave.sh
git commit -m "feat: add master-slave launch file and config"
```

---

### Task 6: Verification — dry run without hardware

**Step 1: Check launch file parses correctly**

Run: `pixi run ros2 launch yam_damiao_controller master_slave.launch.py --show-args`
Expected: shows `use_master_l`, `use_master_r`, `use_slave_l`, `use_slave_r` arguments

**Step 2: Check config file loads**

Run: `pixi run ros2 launch yam_damiao_controller master_slave.launch.py use_master_l:=false use_master_r:=false use_slave_l:=false use_slave_r:=false`
Expected: launches and exits cleanly (no nodes to run)

**Step 3: Final commit with all files**

```bash
git add -A
git status
git commit -m "feat: 4-arm master-slave teleoperation with configurable bilateral feedback"
```

---

## File Summary

| File | Action | Description |
|------|--------|-------------|
| `src/yam/yam_damiao_controller/config/master_slave.yaml` | CREATE | 4-arm YAML config |
| `src/yam/yam_damiao_controller/launch/master_slave.launch.py` | CREATE | Launch 4 nodes |
| `bash/run_master_slave.sh` | CREATE | Shell entry point |
| `src/yam/yam_damiao_controller/include/.../YamController.hpp` | MODIFY | Add master/slave members |
| `src/yam/yam_damiao_controller/src/YamController.cpp` | MODIFY | Add master/slave branches |

## Usage

```bash
# Run all 4 arms (bilateral feedback ON by default in config)
./bash/run_master_slave.sh

# Run without bilateral (edit master_slave.yaml: bilateral_enabled: false)

# Run only left pair for testing
pixi run ros2 launch yam_damiao_controller master_slave.launch.py use_master_r:=false use_slave_r:=false
```
