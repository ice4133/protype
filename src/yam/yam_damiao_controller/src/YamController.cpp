#include "yam_damiao_controller/YamController.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <thread>

namespace yam {

	YamController::YamController() : Node("yam_controller_node"), current_state_(ArmState::INIT) {
		rclcpp::on_shutdown(std::bind(&YamController::cleanup, this));

		// Parameters
		arm_control_type_ = this->declare_parameter("arm_control_type", "normal");
		std::string arm_control_type = arm_control_type_;
		std::string package_name = "yam_description";
		std::string package_share_dir = ament_index_cpp::get_package_share_directory(package_name);

		std::string urdf_path = package_share_dir + "/urdf/" + "yam.urdf";
        yam_solver_ptr = std::make_shared<YamFun>(urdf_path);

		// Motor interface setup
		interfaces_ptr_damiao_ = 
		std::make_shared<Damiao6dofInterfacesThread>(this->declare_parameter("arm_can_id", "/dev/ttyACM0"));

		arm_dof = interfaces_ptr_damiao_->ARM_DOF;
		gripper_dof = interfaces_ptr_damiao_->GRIPPER_DOF;

        joint_homes_ = std::vector<float>(arm_dof, 0.0f);
        gripper_homes_ = std::vector<float>(gripper_dof, 0.0f);

        joint_positions_ = std::vector<float>(arm_dof, 0.0f);
        joint_velocities_= std::vector<float>(arm_dof, 0.0f);
        joint_torques_ = std::vector<float>(arm_dof, 0.0f);
        gripper_positions_ = std::vector<float>(gripper_dof, 0.0f);
        gripper_velocities_ = std::vector<float>(gripper_dof, 0.0f);
        gripper_torques_ = std::vector<float>(gripper_dof, 0.0f);

        target_joint_positions_ = std::vector<float>(arm_dof, 0.0f);
        target_joint_vel_ = std::vector<float>(arm_dof, 0.0f);
        target_joint_acc_ = std::vector<float>(arm_dof, 0.0f);
        target_end_effector_pos_ = std::vector<float>{0, 0, 0, 1, 0, 0, 0};    // xyz_wxyz

		// Gain parameters
		arm_kp_1_3_ = this->declare_parameter("arm_kp_1_3", 100.0f);
        arm_kp_4_6_ = this->declare_parameter("arm_kp_4_6", 50.0f);
        arm_kd_1_3_ = this->declare_parameter("arm_kd_1_3", 1.0f);
        arm_kd_4_6_ = this->declare_parameter("arm_kd_4_6", 1.0f);
		gripper_position_gain_ = this->declare_parameter("gripper_position_gain", 60.0f);
		gripper_velocity_gain_ = this->declare_parameter("gripper_velocity_gain", 1.0f);
		gripper_torque_limit_ = this->declare_parameter("gripper_torque_limit", 3.0f);
		gripper_open_assist_ = this->declare_parameter("gripper_open_assist", 0.0f);
		gripper_assist_range_ratio_ = this->declare_parameter("gripper_assist_range_ratio", 0.4f);
		gripper_calibration_step_ = this->declare_parameter("gripper_calibration_step", 0.001f);
		gripper_calibration_safety_factor_ = this->declare_parameter("gripper_calibration_safety_factor", 1.2f);
		go_home_kp_ = this->declare_parameter("go_home_kp", 15.0f);
		go_home_kd_ = this->declare_parameter("go_home_kd", 0.5f);

		// Control setup
		double control_period = this->declare_parameter("arm_control_period", 0.005);
		setdt(control_period);

        previous_time_ = this->get_clock()->now();

        arm_trajectory_system_=std::make_unique<ArmTrajectorySystem>(Ts,
        bandwidths,
        max_velocities,
        max_accelerations,
        max_jerks);
        
		// Initialize motor interface
		interfaces_ptr_damiao_->Disable();
		robot_init();

		// Setup publishers/subscribers/timers
		if (arm_control_type == "normal") {
			robot_status_publisher_ = this->create_publisher<yam_arm_msg::msg::YamStatus>(
				this->declare_parameter("arm_pub_topic_name", "arm_status"), rclcpp::SensorDataQoS());

			state_timer_ = this->create_wall_timer(std::chrono::milliseconds(3),
				std::bind(&YamController::publishState, this));
			RCLCPP_INFO(this->get_logger(), "Robot Status Pub created!");

			// Control timer at specified period
			transitionToState(ArmState::INIT);
			control_timer_ = this->create_wall_timer(
				std::chrono::duration<double>(dt_),
				std::bind(&YamController::controlLoop, this));
			RCLCPP_INFO(this->get_logger(), "Robot ControlLoop created!");

			robot_cmd_subscriber_ = this->create_subscription<yam_arm_msg::msg::YamCmd>(
				this->declare_parameter("arm_sub_topic_name", "arm_cmd"), rclcpp::SensorDataQoS(),
				std::bind(&YamController::cmdCallback, this, std::placeholders::_1));
			RCLCPP_INFO(this->get_logger(), "Robot Cmd Sub created!");

            // rviz_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            //     "/joint_states", 1);
            // RCLCPP_INFO(this->get_logger(), "Rviz Joint State Pub created!");
		}
		else if (arm_control_type == "slave") {
			// Slave mode: subscribe to master YamStatus, follow joint positions
			robot_status_publisher_ = this->create_publisher<yam_arm_msg::msg::YamStatus>(
				this->declare_parameter("arm_pub_topic_name", "arm_status"), rclcpp::SensorDataQoS());

			state_timer_ = this->create_wall_timer(std::chrono::milliseconds(3),
				std::bind(&YamController::publishState, this));

			transitionToState(ArmState::INIT);
			control_timer_ = this->create_wall_timer(
				std::chrono::duration<double>(dt_),
				std::bind(&YamController::controlLoop, this));

			// Subscribe to master's YamStatus (not YamCmd)
			auto master_topic = this->declare_parameter("arm_sub_topic_name", "master_status");
			slave_status_subscriber_ = this->create_subscription<yam_arm_msg::msg::YamStatus>(
				master_topic, rclcpp::SensorDataQoS(),
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
		else if (arm_control_type == "master") {
			// Master mode: gravity compensation + optional bilateral force feedback
			robot_status_publisher_ = this->create_publisher<yam_arm_msg::msg::YamStatus>(
				this->declare_parameter("arm_pub_topic_name", "arm_status"), rclcpp::SensorDataQoS());

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
					slave_topic, rclcpp::SensorDataQoS(),
					std::bind(&YamController::slaveStatusCallback, this, std::placeholders::_1));

				RCLCPP_INFO(this->get_logger(),
					"Master mode: bilateral feedback ON, subscribing to '%s'", slave_topic.c_str());
			} else {
				RCLCPP_INFO(this->get_logger(), "Master mode: bilateral feedback OFF");
			}

			// Declare but don't use arm_sub_topic_name (needed for parameter consistency)
			this->declare_parameter("arm_sub_topic_name", "unused");
		}
		else {
			RCLCPP_ERROR(this->get_logger(), "Unknown arm control type: %s", arm_control_type.c_str());
		}
	}

	void YamController::robot_init(){
		interfaces_ptr_damiao_->Init();
	}

    void YamController::startHoming() {
        RCLCPP_INFO(this->get_logger(), "Homing started");
    }

	void YamController::transitionToState(ArmState new_state) {
	// Handle state transitions
	switch (new_state) {
		case ArmState::SOFT:
			interfaces_ptr_damiao_->Disable();
			RCLCPP_INFO(this->get_logger(), "Transitioning to SOFT state, restart to resume!!!");
			break;
			
		case ArmState::INIT:
			RCLCPP_INFO(this->get_logger(), "Transitioning to INIT state");
			startHoming();
			// Initialize homing procedure
		break;
			
        case ArmState::G_COMPENSATION:
			RCLCPP_INFO(this->get_logger(), "Transitioning to G_COMPENSATION state");
			break;            

		case ArmState::JOINT_CONTROL:
			RCLCPP_INFO(this->get_logger(), "Transitioning to JOINT_CONTROL state");
			break;
			
        case ArmState::END_CONTROL:
        	RCLCPP_INFO(this->get_logger(), "Transitioning to END_CONTROL state");
			break;
			
		// Add other state transitions as needed
		default:
			RCLCPP_WARN(this->get_logger(), "Transitioning to unimplemented state: %d", 
						static_cast<int>(new_state));
		}
		current_state_ = new_state;
	}

  void YamController::cmdCallback(const yam_arm_msg::msg::YamCmd::SharedPtr msg) {

    if (current_state_ != ArmState::SOFT && current_state_ != ArmState::INIT){
      // Map message modes to state machine states
        switch(msg->mode) {
            case 0:  // SOFT (disable motors) 
                transitionToState(ArmState::SOFT);
                break;
                
            // case 1:  // INIT
            //     transitionToState(ArmState::INIT);
            //     break;
                
            case 3:
                transitionToState(ArmState::G_COMPENSATION);
                break;

            case 4:  // END_CONTROL
                if (current_state_ != ArmState::END_CONTROL) {
                    initialization_flag_=false;
                    transitionToState(ArmState::END_CONTROL);
                }
                for (int i = 0; i < 7; i++) {
                    target_end_effector_pos_[i] = msg->end_pose[i];
                }
                target_gripper_position_ = static_cast<float>(msg->gripper);
                break;
                
            case 5:  // JOINT_CONTROL
                
                if (current_state_ != ArmState::JOINT_CONTROL) {
                    initialization_flag_=false;
                    transitionToState(ArmState::JOINT_CONTROL);
                }
                // Store target joint positions
                for (int i = 0; i < 6; i++) {
                    target_joint_positions_[i] = static_cast<float>(msg->joint_pos[i]);
                }
                target_gripper_position_ = static_cast<float>(msg->gripper);
                break;
                
            // case 6:  // ZERO (calibration)
            //     transitionToState(ArmState::ZERO);
            //     break;
                
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown control mode: %ld", msg->mode);
        }
    }
  }

  void YamController::controlLoop() {
    // Run state machine
    runStateMachine();
  }

  void YamController::runStateMachine() {
      // Execute current state behavior
      switch (current_state_) {
        case ArmState::SOFT:
        break;
            
        case ArmState::INIT:
        executeInit();
        break;
            
        case ArmState::END_CONTROL:
        checkError();
        executeEndEffectorControl();
        break;
            
        case ArmState::JOINT_CONTROL:
        checkError();
        executeJointControl();
        break;

        case ArmState::G_COMPENSATION:
        checkError();
        if (arm_control_type_ == "master") {
            executeMasterControl();
        } else {
            executeGravityCompensation();
        }
        break;

        //   case ArmState::ZERO:
        //     checkError();
        //     executeZeroing();
        //     break;
              
          // Add other state behaviors as needed
        default:
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "Unimplemented state behavior: %d", 
                                static_cast<int>(current_state_));
      }
    }

    void YamController::executeInit() {
        if (homing_in_progress_) {

            // Initialize homing ramp if not done yet
            if (!homing_ramp_initialized_) {
                homing_start_time_ = this->now();
                homing_ramp_initialized_ = true;
            }

            auto elapsed = this->now() - homing_start_time_;
            float elapsed_seconds = elapsed.seconds();
            
            if (elapsed_seconds > homing_duration_) {
                RCLCPP_INFO(this->get_logger(), "Homing complete");
                // Update robot status to get current positions
                interfaces_ptr_damiao_->updateYamStatus();
                auto current_joint_positions = interfaces_ptr_damiao_->getJointPositions();
                auto current_gripper_position = interfaces_ptr_damiao_->getGripperPositions();
                
                // Check if all joints are at home position (within tolerance)
                const float POSITION_TOLERANCE = 0.1f; // Adjust tolerance as needed
                bool all_joints_homed = true;
                
                for (size_t i = 0; i < current_joint_positions.size(); i++) {
                    if (fabs(current_joint_positions[i]) > POSITION_TOLERANCE) {
                        RCLCPP_ERROR(this->get_logger(),
                                    "Joint %zu not at home position: %.6f",
                                    i, (double)current_joint_positions[i]);
                        all_joints_homed = false;
                    }
                }
                
                for (size_t i = 0; i < current_gripper_position.size(); i++) {
                    if (fabs(current_gripper_position[i]) > POSITION_TOLERANCE) {
                        RCLCPP_ERROR(this->get_logger(), 
                                    "Gripper joint %zu not at home position", i);
                        all_joints_homed = false;
                    }
                }
                
                if (!all_joints_homed) {
                    RCLCPP_ERROR(this->get_logger(), "Homing failed - not all joints reached home position");
                    // Transition to error state or take appropriate action
                    transitionToState(ArmState::SOFT);
                    homing_ramp_initialized_ = false;
                    return;
                }
                
                homing_in_progress_ = false;
                RCLCPP_INFO(this->get_logger(), "Homing complete - all joints at home position");
            }
            else {

                // Clamp gripper kp to enforce torque limit during homing
                // Use go_home_kp_ for homing (gripper_position_gain_ may be 0 in master mode)
                float homing_gripper_error = std::abs(gripper_homes_[0] - gripper_positions_[0]);
                float homing_effective_kp = go_home_kp_;
                if (homing_gripper_error > 0.01f && homing_effective_kp * homing_gripper_error > gripper_torque_limit_) {
                    homing_effective_kp = gripper_torque_limit_ / homing_gripper_error;
                }

                interfaces_ptr_damiao_->setRobotPosition(
                    joint_homes_,
                    std::vector<float>(arm_dof, 0.0f),
                    std::vector<float>(arm_dof, go_home_kp_),
                    std::vector<float>(arm_dof, go_home_kd_),
                    std::vector<float>(arm_dof, 0.0f),
                    gripper_homes_,
                    std::vector<float>(gripper_dof, 0.0f),
                    std::vector<float>(gripper_dof, homing_effective_kp),
                    std::vector<float>(gripper_dof, gripper_velocity_gain_),
                    std::vector<float>(gripper_dof, 0.0f)
                );
            }
        }
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
    }

    void YamController::setRobotJoint() {

        bool delta_too_big = false;

        // Iterate through the joint positions to check for excessive delta.
        // delta check for first 4 motors
        for (size_t i = 0; i < target_joint_positions_.size() - 2; ++i) {
            if (std::abs(target_joint_positions_[i] - joint_positions_[i]) > max_joint_delta) {
                delta_too_big = true;
                RCLCPP_ERROR(this->get_logger(), "Joint Delta Too Big!");
                break; // Exit the loop as soon as one large delta is found.
            }
        }

        // If a large delta was found, use the current joint positions to avoid a jump.
        if (delta_too_big) {
            target_joint_positions_ = joint_positions_;
        }

        if (!initialization_flag_) {
            initialization_flag_ = true;
            std::array<double, 6> current_joint{0,0,0,0,0,0};
            for (int i = 0; i < 6; i++) {
            current_joint[i]=joint_positions_[i];
            }
            arm_trajectory_system_->reset(current_joint);
        }

        //joint positions fifter
        std::array<double, 6> target_joint_pos{0,0,0,0,0,0};
        for (int i = 0; i < 6; i++) {
            target_joint_pos[i]=target_joint_positions_[i];
        }
        arm_trajectory_system_->control_cycle(target_joint_pos);
        auto trajectory_goal_pos=arm_trajectory_system_->get_positions();
        auto trajectory_goal_vel=arm_trajectory_system_->get_velocities();
        auto trajectory_goal_acc=arm_trajectory_system_->get_accelerations();
        for (int i = 0; i < 6; i++) {
            target_joint_positions_[i]=static_cast<float>(trajectory_goal_pos[i]);
            target_joint_vel_[i]      =static_cast<float>(trajectory_goal_vel[i]);
            target_joint_acc_[i]      =static_cast<float>(trajectory_goal_acc[i]);
        }

        // Eigen::VectorXd target_q = Eigen::Map<Eigen::VectorXf>(
        //     joint_positions_.data(), joint_positions_.size()
        // ).cast<double>();
        Eigen::VectorXd target_q = Eigen::Map<Eigen::VectorXf>(
            target_joint_positions_.data(), target_joint_positions_.size()
        ).cast<double>();
        Eigen::VectorXd target_dq = Eigen::Map<Eigen::VectorXf>(
            target_joint_vel_.data(), target_joint_vel_.size()
        ).cast<double>();

        Eigen::VectorXd target_ddq = Eigen::Map<Eigen::VectorXf>(
            target_joint_acc_.data(), target_joint_acc_.size()
        ).cast<double>();

        // Eigen::VectorXf target_tau = yam_solver_ptr->gravityCompensation(target_q).cast<float>();
        Eigen::VectorXf target_tau = yam_solver_ptr->InverseDynamics(target_q,target_dq,target_ddq).cast<float>();

        std::vector<float> target_tau_gc(target_tau.data(), target_tau.data() + target_tau.size());

        // Clamp gripper kp to enforce torque limit
        float gripper_error = std::abs(target_gripper_position_ - gripper_positions_[0]);
        float effective_gripper_kp = gripper_position_gain_;
        if (gripper_error > 0.01f && effective_gripper_kp * gripper_error > gripper_torque_limit_) {
            effective_gripper_kp = gripper_torque_limit_ / gripper_error;
        }

        interfaces_ptr_damiao_->setRobotPosition(
            {target_joint_positions_[0], target_joint_positions_[1], target_joint_positions_[2],
            target_joint_positions_[3], target_joint_positions_[4], target_joint_positions_[5]},
            std::vector<float>(arm_dof, 0.0f),
            std::vector<float>{arm_kp_1_3_, arm_kp_1_3_, arm_kp_1_3_, arm_kp_4_6_, arm_kp_4_6_, arm_kp_4_6_},
            std::vector<float>{arm_kd_1_3_, arm_kd_1_3_, arm_kd_1_3_, arm_kd_4_6_, arm_kd_4_6_, arm_kd_4_6_},
            target_tau_gc,
            {target_gripper_position_},
            std::vector<float>(gripper_dof, 0.0f),
            std::vector<float>(gripper_dof, effective_gripper_kp),
            std::vector<float>(gripper_dof, gripper_velocity_gain_),
            std::vector<float>(gripper_dof, 0.0f)
        );


        // auto joint_cmd_msg = sensor_msgs::msg::JointState();

        // // timestamp
        // joint_cmd_msg.header.stamp = this->get_clock()->now();

        // // prepare sizes
        // size_t n = target_joint_positions_.size();
        // joint_cmd_msg.name.resize(n);
        // joint_cmd_msg.position.resize(n);
        // joint_cmd_msg.velocity.assign(n, 0.0); // zero velocities
        // joint_cmd_msg.effort.resize(n);

        // // fill positions (and efforts if available)
        // for (size_t i = 0; i < n; ++i) {
        //     // fallback joint names: "joint1", "joint2", ...
        //     joint_cmd_msg.name[i] = std::string("joint") + std::to_string(i + 1);

        //     // convert float -> double for message
        //     joint_cmd_msg.position[i] = static_cast<double>(target_joint_positions_[i]);

        //     // // efforts: use target_tau_gc if it has matching data, otherwise 0.0
        //     // if (i < target_tau_gc.size()) {
        //     //     joint_cmd_msg.effort[i] = static_cast<double>(target_tau_gc[i]);
        //     // } else {
        //     //     joint_cmd_msg.effort[i] = 0.0;
        //     // }
        // }

        // rviz_joint_state_publisher_->publish(joint_cmd_msg);

        // // Calculate time interval and log
        // rclcpp::Time current_time = this->get_clock()->now();
        // rclcpp::Duration time_interval = current_time - previous_time_;
        // previous_time_ = current_time;  // Update the previous time to the current one

        // // Log the time interval every time the function is called
        // RCLCPP_INFO(this->get_logger(), "setRobotJoint Time interval: %.7f seconds", time_interval.seconds());

    }

    void YamController::executeGravityCompensation() {
        Eigen::VectorXd current_q = Eigen::Map<Eigen::VectorXf>(
            joint_positions_.data(), joint_positions_.size()
        ).cast<double>();

        yam_solver_ptr->setCurrentConfig(current_q);

        Eigen::VectorXf current_tau = yam_solver_ptr->gravityCompensationCurrent().cast<float>();

        std::vector<float> target_tau_gc(current_tau.data(), current_tau.data() + current_tau.size());

        // std::cout << "target_tau_gc: [";
        // for (size_t i = 0; i < target_tau_gc.size(); ++i) {
        //     std::cout << target_tau_gc[i];
        //     if (i + 1 < target_tau_gc.size()) std::cout << ", ";
        // }
        // std::cout << "]" << std::endl;

        // In G_COMPENSATION mode, gripper target = current position, so no torque clamp needed
        interfaces_ptr_damiao_->setRobotPosition(
            {joint_positions_[0], joint_positions_[1], joint_positions_[2],
            joint_positions_[3], joint_positions_[4], joint_positions_[5]},
            std::vector<float>(arm_dof, 0.0f),
            std::vector<float>(arm_dof, 0.0f),
            std::vector<float>(arm_dof, 0.0f),
            target_tau_gc,
            {gripper_positions_[0]},
            std::vector<float>(gripper_dof, 0.0f),
            std::vector<float>(gripper_dof, gripper_position_gain_),
            std::vector<float>(gripper_dof, gripper_velocity_gain_),
            std::vector<float>(gripper_dof, 0.0f)
        );
    }

    void YamController::slaveStatusCallback(const yam_arm_msg::msg::YamStatus::SharedPtr msg) {
        for (int i = 0; i < 6; i++) {
            slave_joint_positions_[i] = static_cast<float>(msg->joint_pos[i]);
            slave_joint_velocities_[i] = static_cast<float>(msg->joint_vel[i]);
        }
        slave_last_received_time_ = this->get_clock()->now();
        slave_data_valid_ = true;
    }

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

        interfaces_ptr_damiao_->setRobotPosition(
            {joint_positions_[0], joint_positions_[1], joint_positions_[2],
             joint_positions_[3], joint_positions_[4], joint_positions_[5]},
            std::vector<float>(arm_dof, 0.0f),
            std::vector<float>(arm_dof, 0.0f),
            std::vector<float>(arm_dof, 0.0f),
            target_tau,
            {gripper_positions_[0]},
            std::vector<float>(gripper_dof, 0.0f),
            std::vector<float>(gripper_dof, 0.0f),
            std::vector<float>(gripper_dof, 0.0f),
            {gripper_tau}
        );
    }

    void YamController::executeJointControl() {
        setRobotJoint();
    }

    void YamController::executeEndEffectorControl() {

        std::vector<float> xyz_wxyz = target_end_effector_pos_;
        // Extract position and quaternion from the vector
        Eigen::Vector3d xyz(xyz_wxyz[0], xyz_wxyz[1], xyz_wxyz[2]);
        Eigen::Quaterniond quat(xyz_wxyz[3], xyz_wxyz[4], xyz_wxyz[5], xyz_wxyz[6]);

        Eigen::VectorXd current_q = Eigen::Map<Eigen::VectorXf>(
            joint_positions_.data(), joint_positions_.size()
        ).cast<double>();
        yam_solver_ptr->setCurrentConfig(current_q);

        auto start_time = std::chrono::high_resolution_clock::now();

        Eigen::VectorXd q;
        bool ik_success = yam_solver_ptr->computeIK(xyz, quat, q);

        // End timing and calculate duration
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

        if (ik_success) {
            target_joint_positions_ = std::vector<float>(q.data(), q.data() + q.size());
        }
        else {
            RCLCPP_WARN(this->get_logger(), "IK computation failed");
            RCLCPP_INFO(
            this->get_logger(), 
            "IK computation time: %.4f milliseconds", 
            duration.count() / 1000.0
        );
        }

        setRobotJoint();
    }

	void YamController::publishState() {
        interfaces_ptr_damiao_->updateYamStatus();
        
        joint_positions_ = interfaces_ptr_damiao_->getJointPositions();
        joint_velocities_ = interfaces_ptr_damiao_->getJointVelocities();
        joint_torques_ = interfaces_ptr_damiao_->getJointCurrent();
        gripper_positions_ = interfaces_ptr_damiao_->getGripperPositions();
        gripper_velocities_ = interfaces_ptr_damiao_->getGripperVelocities();
        gripper_torques_ = interfaces_ptr_damiao_->getGripperCurrent();

        auto robot_status_msg = yam_arm_msg::msg::YamStatus();
        robot_status_msg.header.stamp = this->get_clock()->now();

        Eigen::VectorXd q(6);
        for (size_t i = 0; i < 6; ++i) {
            q(i) = static_cast<double>(joint_positions_[i]);
        }

        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;

        yam_solver_ptr->computeEEFinNorm(q, position, orientation);

        // The message expects [x, y, z, w, qx, qy, qz].
        robot_status_msg.end_pose[0] = position.x();
        robot_status_msg.end_pose[1] = position.y();
        robot_status_msg.end_pose[2] = position.z();
        robot_status_msg.end_pose[3] = orientation.w();
        robot_status_msg.end_pose[4] = orientation.x();
        robot_status_msg.end_pose[5] = orientation.y();
        robot_status_msg.end_pose[6] = orientation.z();

        for (int i = 0; i < 6; i++) {
            robot_status_msg.joint_pos[i] = joint_positions_[i];
            robot_status_msg.joint_vel[i] = joint_velocities_[i];
            robot_status_msg.joint_cur[i] = joint_torques_[i];
        }
        robot_status_msg.joint_pos[6] = gripper_positions_[0];
        robot_status_msg.joint_vel[6] = gripper_velocities_[0];
        robot_status_msg.joint_cur[6] = gripper_torques_[0];

        robot_status_publisher_->publish(robot_status_msg);
	}

	void YamController::checkError() {
		bool robot_error = interfaces_ptr_damiao_->checkRobotErrors();
		std::string error_msg;

		if (robot_error) {
			error_msg = interfaces_ptr_damiao_->getErrorsDescription();
			RCLCPP_ERROR(this->get_logger(), error_msg.c_str());
			transitionToState(ArmState::SOFT);
		}
	}

	void YamController::cleanup() {
		RCLCPP_INFO(this->get_logger(), "Shutting down, disabling motors");
		interfaces_ptr_damiao_->Disable();
	}
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<yam::YamController>();
    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}
