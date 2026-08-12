#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <cstdint>
#include <vector>
#include <stdexcept>
#include <math.h>
#include <map>
#include <string>
#include <algorithm>

#include "drivers/damiao_socket.hpp"

class MotorControl {
public:
    // Constructor with motor configuration
    MotorControl(std::vector<damiao::Motor> motor_config, 
                const std::string& serial_port,
                std::vector<float> limit_min,
                std::vector<float> limit_max
            );
    ~MotorControl();

    // std::vector<float> kp;
    // std::vector<float> kd;
    // std::vector<float> pos;
    // std::vector<float> vel;
    // std::vector<float> tau;

    // std::vector<float> desir_motor_pos;
    // std::vector<float> desir_motor_vel;
    // std::vector<float> desir_motor_tau;
    // std::vector<float> desir_motor_acc;    

    std::vector<float> current_motor_pos;
    std::vector<float> current_motor_vel;
    std::vector<float> current_motor_acc;
    std::vector<float> current_motor_tau;

    // Utility functions
    int floatToUint(float value, float min, float max, int bits);
    float uintTofloat(int value, float min, float max, int bits);

    bool CheckMotorErrors();
    std::string CheckErrorsDescription();

    void motor_clamp(std::vector<float>& target_pos);

    // Motor control functions
    void InitMotors(); // Enable motors & set zero position
    void EnableMotors();
    void DisableMotors();
    void SetZeroMotors();
    void switchControlMode(const damiao::Control_Mode mode);
    void updateMotorState();
    void MitCtrl(const std::vector<float>& pos, const std::vector<float>& vel, 
                const std::vector<float>& kp, const std::vector<float>& kd, 
                const std::vector<float>& tau);
    void PosCtrl(const std::vector<float>& pos, const std::vector<float>& vel);
    
    // Filtering functions
    void KalmanFilter(float process_noise, float measurement_noise, 
                     float estimation_error, float initial_value, float limit);
    float update(float measurement);
    void ButterworthFilter(float cutoff_frequency, float sampling_rate);
    float Butt_update(float new_value);
    void compensate_static_friction_through_vel(
        const std::vector<float>& filtered_motor_vel, 
        std::vector<float>& motor_tor);
    
	size_t get_motors_num() {	return motors.size();	}

private:
    std::shared_ptr<SerialPort> serial;
    std::shared_ptr<damiao::Motor_Control> DAMIAO_Control;
    std::vector<damiao::Motor> motors;
    std::string error_report;
    
    std::vector<float> motor_limit_min;
    std::vector<float> motor_limit_max;

    // 物理量范围
    static constexpr float P_MIN = -12.5f;
    static constexpr float P_MAX = 12.5f;

    static constexpr float V_MIN = -30.0f;
    static constexpr float V_MAX = 30.0f;

    static constexpr float KP_MIN = 0.0f;
    static constexpr float KP_MAX = 500.0f;

    static constexpr float KD_MIN = 0.0f;
    static constexpr float KD_MAX = 5.0f;

    static constexpr float T_MIN = -10.0f;
    static constexpr float T_MAX = 10.0f;


    static constexpr float P_MIN_4340 = -12.5f;
    static constexpr float P_MAX_4340 = 12.5f;

    static constexpr float V_MIN_4340 = -10.0f;
    static constexpr float V_MAX_4340 = 10.0f;

    static constexpr float KP_MIN_4340 = 0.0f;
    static constexpr float KP_MAX_4340 = 500.0f;

    static constexpr float KD_MIN_4340 = 0.0f;
    static constexpr float KD_MAX_4340 = 5.0f;

    static constexpr float T_MIN_4340 = -28.0f;
    static constexpr float T_MAX_4340 = 28.0f;

    float Q = 1e-6; // 过程噪声协方差
    float R = 1e-2; // 测量噪声协方差
    float P_kalman = 1; // 估计误差协方差
    float K; // 卡尔曼增益
    float X = 0; // 状态估计
    float threshold= 0.03;//阈值

    // Butterworth filter variables
    float a0, a1, a2, b1, b2;
    float x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0;
};

#endif // MOTOR_CONTROL_H