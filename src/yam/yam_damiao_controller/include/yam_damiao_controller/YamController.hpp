#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <array>
#include <vector>

#include "yam_arm_msg/msg/yam_cmd.hpp"
#include "yam_arm_msg/msg/yam_status.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "yam_damiao_controller/Damiao_6dof_node.hpp"
#include "mathematical_model/yam_fun.hpp"

namespace yam {

// State machine states
enum class ArmState {
    SOFT = 0,           // Disabled state
    INIT = 1,        // Homing procedure
    PROTECT = 2,        // Protection mode
    G_COMPENSATION = 3, // Gravity compensation
    END_CONTROL = 4,    // End effector control
    JOINT_CONTROL = 5, // Joint position control
    ZERO = 6,           // Zeroing/calibration
    PLANNING = 7        // Trajectory planning
};

class YamController : public rclcpp::Node {
public:
    YamController();
    void cleanup();

    void robot_init();

    // Command callback
    void cmdCallback(const yam_arm_msg::msg::YamCmd::SharedPtr msg);

    // State publishing
    void publishState();

    // State machine functions
    void transitionToState(ArmState new_state);
    void runStateMachine();
    void executeInit();
    void setRobotJoint();
    void executeJointControl();
    void executeEndEffectorControl();
    void executeGravityCompensation();
    void executeMasterControl();
    void slaveStatusCallback(const yam_arm_msg::msg::YamStatus::SharedPtr msg);

    void controlLoop();
    void checkError();

    void startHoming();

    // Utility functions
    void setdt(double dt) {
    dt_ = dt;
    }

    static void stopArm()
    {
        if (instance) {
            instance->interfaces_ptr_damiao_->Disable();
        }
    }

    static void signalHandler(int signum) {
        if (instance) {
            instance->interfaces_ptr_damiao_->Disable();
            std::this_thread::sleep_for(std::chrono::microseconds(2000));
        }
        std::exit(signum);
    }

    static YamController *instance;
    std::unique_ptr<ArmTrajectorySystem> arm_trajectory_system_;
private:
    // Motor interface
    std::shared_ptr<Damiao6dofInterfacesThread> interfaces_ptr_damiao_;
    std::shared_ptr<YamFun> yam_solver_ptr;

    // State machine state
    ArmState current_state_;

    // Target positions for control states
    std::vector<float> target_joint_positions_;
    std::vector<float> target_joint_vel_;
    std::vector<float> target_joint_acc_;
    std::vector<float> target_end_effector_pos_;
    float target_gripper_position_ = 0.0f;

    size_t arm_dof;
	size_t gripper_dof;

    // Control gains
    float arm_kp_1_3_;         // kp for arm joints
    float arm_kp_4_6_;
    float arm_kd_1_3_;
    float arm_kd_4_6_;
    float arm_velocity_gain_;         // kd for arm joints
    float gripper_position_gain_;     // kp for gripper
    float gripper_velocity_gain_;     // kd for gripper
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
    float go_home_kp_;                // kp for homing
    float go_home_kd_;                // kd for homing

    // Homing control
    bool homing_in_progress_ = true;
    rclcpp::Time homing_start_time_;
    
    // Smooth homing variables
    bool homing_ramp_initialized_ = false;
    bool initialization_flag_=false;
    float homing_duration_ = 3.0f;  // Total time for homing motion

    const float max_joint_delta = 0.7f; // 40 degree

    std::vector<float> joint_homes_;
    std::vector<float> joint_positions_;
    std::vector<float> joint_velocities_;
    std::vector<float> joint_torques_;
    std::vector<float> gripper_positions_;
    std::vector<float> gripper_homes_;
    std::vector<float> gripper_velocities_;
    std::vector<float> gripper_torques_;

    // Other members
    // JointArrayOTG<6> otg{0.005};  
    rclcpp::Time start_time_;
    // std::vector<LPF1st> filter;
    double dt_ = 0.001;  // control period

    // Publishers and subscribers
    rclcpp::Publisher<yam_arm_msg::msg::YamStatus>::SharedPtr robot_status_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr rviz_joint_state_publisher_;
    rclcpp::Subscription<yam_arm_msg::msg::YamCmd>::SharedPtr robot_cmd_subscriber_;
    rclcpp::TimerBase::SharedPtr control_timer_;  // Timer for control loop
    rclcpp::TimerBase::SharedPtr state_timer_;  // Timer for pub state

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

    rclcpp::Time previous_time_; // to store the last call time

    // 系统参数
    double control_rate = 150.0; // Hz
    double Ts = 1.0 / control_rate;

    // 关节参数 (7自由度)
    std::array<double, 6> bandwidths = {
        30, 30,30,  // 前三个关节高带宽
        40, 40,         // 中间关节中等带宽
        40            // 末端关节较低带宽
    };

    std::array<double, 6> max_velocities = {
        3, 3, 3, 4, 4, 4
    };

    std::array<double, 6> max_accelerations = {
        9.0, 9.0, 9.0, 12.0, 12.0, 12.0
    };

    std::array<double, 6> max_jerks = {
        5000.0,5000.0,5000.0,5000.0,5000.0,5000.0
    };

};
}  // namespace yam