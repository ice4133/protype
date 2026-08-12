#include "drivers/motor_control.hpp"

MotorControl::MotorControl(std::vector<damiao::Motor> motor_config, 
                          const std::string& serial_port,
                          std::vector<float> limit_min,
                          std::vector<float> limit_max)
    : motors(std::move(motor_config))
{
    // Check if limit vectors match the number of motors
    size_t motor_count = get_motors_num();
    
    if (limit_min.size() != motor_count) {
        throw std::invalid_argument("limit_min size (" + std::to_string(limit_min.size()) + 
                                   ") does not match number of motors (" + 
                                   std::to_string(motor_count) + ")");
    }
    
    if (limit_max.size() != motor_count) {
        throw std::invalid_argument("limit_max size (" + std::to_string(limit_max.size()) + 
                                   ") does not match number of motors (" + 
                                   std::to_string(motor_count) + ")");
    }
    
    // Check that all min values are less than or equal to max values
    for (size_t i = 0; i < motor_count; ++i) {
        if (limit_min[i] > limit_max[i]) {
            throw std::invalid_argument("limit_min[" + std::to_string(i) + 
                                       "] (" + std::to_string(limit_min[i]) + 
                                       ") is greater than limit_max[" + 
                                       std::to_string(i) + "] (" + 
                                       std::to_string(limit_max[i]) + ")");
        }
    }
    
    serial = std::make_shared<SerialPort>(serial_port, B921600);
    DAMIAO_Control = std::make_shared<damiao::Motor_Control>(serial);
    
    // Initialize all vectors based on motor count
    motor_limit_min = std::move(limit_min);
    motor_limit_max = std::move(limit_max);

    current_motor_pos.resize(motor_count, 0.0f);
    current_motor_vel.resize(motor_count, 0.0f);
    current_motor_acc.resize(motor_count, 0.0f);
    current_motor_tau.resize(motor_count, 0.0f);

    for (size_t i = 0; i < motors.size(); i++) {
        DAMIAO_Control->addMotor(&motors[i]);
        DAMIAO_Control->switchControlMode(motors[i], damiao::MIT_MODE);
        DAMIAO_Control->save_motor_param(motors[i]);         
    }
}

MotorControl::~MotorControl() {
    // Cleanup logic if needed
}

/**
* @brief Check for motor errors
* @return true if any motor has error
*/
bool MotorControl::CheckMotorErrors() {
    error_report.clear(); // Clear existing content
    bool any_error = false;
    constexpr size_t buffer_size = 512; // Buffer for formatting lines
    char buffer[buffer_size];

    for (size_t i = 0; i < motors.size(); ++i) {
        if (motors[i].has_error()) {
            any_error = true;
            uint8_t slave_id = motors[i].GetSlaveId();
            uint8_t error_code = motors[i].get_error_code();
            std::string error_desc = motors[i].get_error_description();

            // Format error line safely
            int len = snprintf(buffer, buffer_size,
                "Motor %zu (ID: 0x%02x) - %s (0x%02x)\n",
                i,
                static_cast<unsigned>(slave_id),
                error_desc.c_str(),
                static_cast<unsigned>(error_code));
            
            if (len > 0 && static_cast<size_t>(len) < buffer_size) {
                error_report += buffer;
            } else {
                // Handle truncation/error: Append safe message
                error_report += "Motor error (description truncated)\n";
            }
        }
    }

    if (!any_error) {
        error_report = "No motor errors detected";
    }

    return any_error;
}

/**
* @brief Get motor error descriptions
* @return Error description string
*/
std::string MotorControl::CheckErrorsDescription() {
    return error_report;
}


int MotorControl::floatToUint(float value, float min, float max, int bits) {
    float span = max - min;
    return static_cast<int>((value - min) * ((1 << bits) - 1) / span);
}

float MotorControl::uintTofloat(int value, float min, float max, int bits) {
    float span = max - min;
    return value * span / ((1 << bits) - 1) + min;
}

void MotorControl::InitMotors() {
    size_t dof = motors.size();
    for (size_t i = 0; i < dof; i++) {
        DAMIAO_Control->clear_error(motors[i]);
        DAMIAO_Control->enable(motors[i]);

        for (size_t j = 0; j <= i; j++) {
            DAMIAO_Control->control_mit(
                motors[j],
                1.0,    // kp
                0.5,    // kd                
                0.0f,   // pos
                0.0f,   // vel
                0.0f    // tau
		    );
        }
    }
    std::cout << "Motors Inited (" << motors.size() << " motors)." << std::endl;
}

void MotorControl::EnableMotors() {
    for (size_t i = 0; i < motors.size(); i++) {
        DAMIAO_Control->clear_error(motors[i]);
        DAMIAO_Control->enable(motors[i]);
    }
    std::cout << "Motors enabled (" << motors.size() << " motors)." << std::endl;
}

void MotorControl::DisableMotors() {
    for (size_t i = 0; i < motors.size(); i++) {
        DAMIAO_Control->clear_error(motors[i]);
        DAMIAO_Control->disable(motors[i]);
    }
    std::cout << "Motors disabled (" << motors.size() << " motors)." << std::endl;
}

void MotorControl::SetZeroMotors() {
    for (size_t i = 0; i < motors.size(); i++) {
        DAMIAO_Control->set_zero_position(motors[i]);
    }
    std::cout << "Motors set to zero position (" << motors.size() << " motors)." << std::endl;
}

void MotorControl::switchControlMode(const damiao::Control_Mode mode) {
    for (size_t i = 0; i < motors.size(); i++) {
        DAMIAO_Control->switchControlMode(motors[i], mode);
        DAMIAO_Control->save_motor_param(motors[i]);
    }
    std::cout << "Control mode switched for " << motors.size() << " motors." << std::endl;
}

void MotorControl::motor_clamp(std::vector<float>& target_pos) {

    for (size_t i = 0; i < motors.size(); ++i) {
        target_pos[i] = std::clamp(
            target_pos[i],
            motor_limit_min[i],
            motor_limit_max[i]
        );
    }
}

void MotorControl::MitCtrl(const std::vector<float>& pos, const std::vector<float>& vel, 
                          const std::vector<float>& kp, const std::vector<float>& kd, 
                          const std::vector<float>& tau) {
    if(pos.size() != motors.size() || 
       vel.size() != motors.size() || 
       kp.size() != motors.size() || 
       kd.size() != motors.size() || 
       tau.size() != motors.size()) {
        throw std::invalid_argument("Input vector sizes don't match motor count");
    }

    std::vector<float> target_pos = pos;
    motor_clamp(target_pos);

    for (size_t i = 0; i < motors.size(); i++) {
        DAMIAO_Control->control_mit(motors[i], kp[i], kd[i], target_pos[i], vel[i], tau[i]);
    }
}

void MotorControl::PosCtrl(const std::vector<float>& pos, const std::vector<float>& vel) {
    if(pos.size() != motors.size() || vel.size() != motors.size()) {
        throw std::invalid_argument("Input vector sizes don't match motor count");
    }

    std::vector<float> target_pos = pos;
    motor_clamp(target_pos);

    for (size_t i = 0; i < motors.size(); i++) {
        DAMIAO_Control->control_pos_vel(motors[i], target_pos[i], vel[i]);
    }
}

void MotorControl::updateMotorState() {
    DAMIAO_Control->receive();  // Receive data for all motors at once
    
    for (size_t i = 0; i < motors.size(); i++) {
        DAMIAO_Control->refresh_motor_status(motors[i]);
        current_motor_pos[i] = motors[i].Get_Position();
        current_motor_vel[i] = motors[i].Get_Velocity();
        current_motor_tau[i] = motors[i].Get_tau();
    }
}

void MotorControl::KalmanFilter(float process_noise, float measurement_noise, 
                              float estimation_error, float initial_value, float limit) {
    Q = process_noise;
    R = measurement_noise;
    P_kalman = estimation_error;
    X = initial_value;
    threshold = limit;
}

float MotorControl::update(float measurement) {
    // Predict
    P_kalman = P_kalman + Q;
    // Update gain
    K = P_kalman / (P_kalman + R);
    // Update estimate
    X = X + K * (measurement - X);
    // Update covariance
    P_kalman = (1 - K) * P_kalman;

    if (std::abs(X) < threshold) {
        X = 0.0f;
    }
    return X;
}

void MotorControl::ButterworthFilter(float cutoff_frequency, float sampling_rate) {
    float wc = tan(M_PI * cutoff_frequency / sampling_rate);
    float k1 = sqrt(2.0) * wc;
    float k2 = wc * wc;
    a0 = k2 / (1.0 + k1 + k2);
    a1 = 2.0 * a0;
    a2 = a0;
    b1 = 2.0 * (k2 - 1.0) / (1.0 + k1 + k2);
    b2 = (1.0 - k1 + k2) / (1.0 + k1 + k2);
}

float MotorControl::Butt_update(float new_value) {
    float result = a0 * new_value + a1 * x1 + a2 * x2 - b1 * y1 - b2 * y2;
    x2 = x1;
    x1 = new_value;
    y2 = y1;
    y1 = result;
    return result;
}

void MotorControl::compensate_static_friction_through_vel(
    const std::vector<float>& filtered_motor_vel, 
    std::vector<float>& motor_tor) {
    
    if(filtered_motor_vel.size() != motor_tor.size()) {
        throw std::invalid_argument("Vector sizes must match");
    }
    
    // Implementation depends on your specific friction model
    // Example placeholder:
    /*
    for (size_t i = 0; i < filtered_motor_vel.size(); i++) {
        if (filtered_motor_vel[i] > 0.1f) {
            motor_tor[i] += 0.5f;  // Positive velocity compensation
        } else if (filtered_motor_vel[i] < -0.1f) {
            motor_tor[i] -= 0.5f;  // Negative velocity compensation
        }
    }
    */
}