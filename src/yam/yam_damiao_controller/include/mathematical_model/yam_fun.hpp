#include <iostream>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <string>

namespace pin = pinocchio;

// 高性能约束位置规划器
class RealTimeConstrainedPlanner {
    public:
        RealTimeConstrainedPlanner(double omega_c, double sample_time,
                                  double max_velocity = std::numeric_limits<double>::infinity(),
                                  double max_acceleration = std::numeric_limits<double>::infinity(),
                                  double max_jerk = std::numeric_limits<double>::infinity())
            : omega_c_(omega_c), Ts_(sample_time),
              v_max_(max_velocity), a_max_(max_acceleration), j_max_(max_jerk) {
            
            // 预计算不变系数
            recalculate_coefficients();
            reset(0.0);
        }
    
        // 重置规划器状态
        void reset(double position, double velocity = 0.0, 
                  double acceleration = 0.0) {
            pos_ = position;
            vel_ = velocity;
            acc_ = acceleration;
            target_ = position;
            last_jerk_ = 0.0;
            jerk_ = 0.0;
        }
    
        // 设置新目标位置
        void set_target(double new_target) noexcept {
            target_ = new_target;
        }
    
        // 实时更新状态（返回位置和速度）
        std::pair<double, double> update() {
            // 计算位置误差 (修正符号)
            //const double e = target_ - pos_;
            const double e = pos_ - target_;
            //std::cout<<std::endl;
            //std::cout<<"e: "<<e<<" pos_:"<<pos_<<" target_"<<target_<<" acc_:"<<acc_<<std::endl;

            // 计算理想加加速度
            double unconstrained_jerk = -k0_ * e - k1_ * vel_ - k2_ * acc_ - k3_ * last_jerk_;
            
            // self.x1 = self.x1 + self.sample_time * self.x2
            // self.x2 = self.x2 + self.sample_time * self.x3
            // self.x3 = self.x3 + self.x4*self.sample_time
            // self.x4 = self.x4 +unconstrained_jerk *self.sample_time
            pos_=pos_+vel_*Ts_;
            //std::cout<<"vel_: "<<vel_<<" acc_:"<<acc_<<" last_jerk_:"<<last_jerk_<<std::endl;
            vel_=vel_+acc_*Ts_;
            acc_=acc_+last_jerk_*Ts_;
            // 应用加加速度约束
            jerk_=jerk_+unconstrained_jerk*Ts_;

            jerk_ = std::clamp(jerk_, -j_max_, j_max_);
            vel_ = std::clamp(vel_, -v_max_, v_max_);
            acc_ = std::clamp(acc_, -a_max_, a_max_);

            // // 应用加加速度约束
            // jerk = std::clamp(jerk, -j_max_, j_max_);
            
            // // 半隐式欧拉积分
            // // 先更新加速度（使用当前jerk）
            // double new_acc = acc_ + jerk * Ts_;
            
            // // 应用加速度约束
            // if (new_acc > a_max_) {
            //     new_acc = a_max_;
            //     jerk = (new_acc - acc_) / Ts_;  // 反向计算实际jerk
            // } else if (new_acc < -a_max_) {
            //     new_acc = -a_max_;
            //     jerk = (new_acc - acc_) / Ts_;
            // }
            
            // // 更新速度（使用约束后的加速度）
            // double new_vel = vel_ + new_acc * Ts_;
            
            // // 应用速度约束
            // if (new_vel > v_max_) {
            //     new_vel = v_max_;
            //     new_acc = (new_vel - vel_) / Ts_;  // 反向计算实际加速度
            //     jerk = (new_acc - acc_) / Ts_;     // 反向计算实际jerk
            // } else if (new_vel < -v_max_) {
            //     new_vel = -v_max_;
            //     new_acc = (new_vel - vel_) / Ts_;
            //     jerk = (new_acc - acc_) / Ts_;
            // }
            
            // // 更新位置（使用约束后的速度）
            // double new_pos = pos_ + new_vel * Ts_;
            
            // 保存jerk用于下次计算
            last_jerk_ = jerk_;
            
            // 更新状态
            // pos_ = new_pos;
            // vel_ = new_vel;
            // acc_ = new_acc;
            
            return {pos_, vel_};
        }
    
        // 获取当前状态
        void get_state(double& position, double& velocity, double& acceleration) const {
            position = pos_;
            velocity = vel_;
            acceleration = acc_;
        }
    
        // 设置约束（实时可更新）
        void set_constraints(double max_velocity, double max_acceleration, double max_jerk) {
            v_max_ = max_velocity;
            a_max_ = max_acceleration;
            j_max_ = max_jerk;
        }
    
        // 设置系统带宽（实时可调整）
        void set_bandwidth(double omega_c) {
            omega_c_ = omega_c;
            recalculate_coefficients();
        }
    
    private:
        void recalculate_coefficients() {
            const double w2 = omega_c_ * omega_c_;
            const double w3 = w2 * omega_c_;
            const double w4 = w2 * w2;
            
            k0_ = w4;
            k1_ = 4.0 * w3;
            k2_ = 6.0 * w2;
            k3_ = 4.0 * omega_c_;
        }
    
        // 规划器参数
        double omega_c_;  // 带宽 (rad/s)
        double Ts_;       // 采样时间 (s)
        
        // 约束限制
        double v_max_;    // 最大速度
        double a_max_;    // 最大加速度
        double j_max_;    // 最大加加速度
        
        // 预计算系数
        double k0_, k1_, k2_, k3_;
        
        // 状态变量
        double pos_;      // 当前位置
        double vel_;      // 当前速度
        double acc_;      // 当前加速度
        double last_jerk_ = 0.0; // 上次加加速度
        double jerk_ = 0.0; // 当前加加速度
        
        // 目标状态
        double target_;   // 目标位置
    };
    
    // 机械臂实时轨迹规划系统
    class ArmTrajectorySystem {
        public:
            ArmTrajectorySystem(double sample_time,
                               const std::array<double, 6>& omega_c,
                               const std::array<double, 6>& max_velocities,
                               const std::array<double, 6>& max_accelerations,
                               const std::array<double, 6>& max_jerks)
                : Ts_(sample_time),planners_{  // 直接在初始化列表中构造数组
                                   RealTimeConstrainedPlanner(omega_c[0], Ts_, max_velocities[0], max_accelerations[0], max_jerks[0]),
                                   RealTimeConstrainedPlanner(omega_c[1], Ts_, max_velocities[1], max_accelerations[1], max_jerks[1]),
                                   RealTimeConstrainedPlanner(omega_c[2], Ts_, max_velocities[2], max_accelerations[2], max_jerks[2]),
                                   RealTimeConstrainedPlanner(omega_c[3], Ts_, max_velocities[3], max_accelerations[3], max_jerks[3]),
                                   RealTimeConstrainedPlanner(omega_c[4], Ts_, max_velocities[4], max_accelerations[4], max_jerks[4]),
                                   RealTimeConstrainedPlanner(omega_c[5], Ts_, max_velocities[5], max_accelerations[5], max_jerks[5])
                               } 
                {

                }
        // 设置所有关节的目标位置
        void set_targets(const std::array<double, 6>& targets) {
            for (int i = 0; i < 6; i++) {
                planners_[i].set_target(targets[i]);
            }
        }
    
        // 更新所有关节状态（返回位置和速度）
        std::pair<std::array<double, 6>, std::array<double, 6>> update() {
            std::array<double, 6> positions;
            std::array<double, 6> velocities;
            
            for (int i = 0; i < 6; i++) {
                auto [pos, vel] = planners_[i].update();
                positions[i] = pos;
                velocities[i] = vel;
            }
            
            return {positions, velocities};
        }
    
        // 获取关节位置
        std::array<double, 6> get_positions() const {
            std::array<double, 6> positions;
            for (int i = 0; i < 6; i++) {
                double pos, vel, acc;
                planners_[i].get_state(pos, vel, acc);
                positions[i] = pos;
            }
            return positions;
        }
    
        // 获取关节速度
        std::array<double, 6> get_velocities() const {
            std::array<double, 6> velocities;
            for (int i = 0; i < 6; i++) {
                double pos, vel, acc;
                planners_[i].get_state(pos, vel, acc);
                velocities[i] = vel;
            }
            return velocities;
        }
    
        // 获取acc
        std::array<double, 6> get_accelerations() const {
            std::array<double, 6> positions;
            for (int i = 0; i < 6; i++) {
                double pos, vel, acc;
                planners_[i].get_state(pos, vel, acc);
                positions[i] = acc;
            }
            return positions;
        }

        // 实时控制循环更新
        void control_cycle(const std::array<double, 6>& new_targets) {
            set_targets(new_targets);
            auto [positions, velocities] = update();
            //send_to_controllers(positions, velocities);
        }
    
        // 重置所有关节状态
        void reset(const std::array<double, 6>& start_positions) {
            for (int i = 0; i < 6; i++) {
                planners_[i].reset(start_positions[i]);
            }
        }
    
    private:
        // 发送命令到关节控制器（示例实现）
        double Ts_;  // 系统采样时间
        std::array<RealTimeConstrainedPlanner, 6> planners_;
    };

class YamFun {
public:
    YamFun(const std::string& urdf_path);
    // Forward kinematics - returns position and orientation as quaternion
    void computeEEFinNorm(const Eigen::VectorXd& q, 
                   Eigen::Vector3d& position, 
                   Eigen::Quaterniond& orientation);
    
    // Inverse kinematics - takes position and orientation as quaternion
    bool computeIK(const Eigen::Vector3d& target_pos,
                   const Eigen::Quaterniond& target_quat,
                   Eigen::VectorXd& result, 
                   int max_iter = 3e2,
                   double eps = 5e-3,
                   double dt = 1e-1);

    // Compute gravity compensation torques for configuration q.
    // Returns a vector of size model.nv (generalized torques).
    Eigen::VectorXd gravityCompensation(const Eigen::VectorXd& q);
    Eigen::VectorXd InverseDynamics(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& ddq);
    
    Eigen::VectorXd gravityCompensationCurrent() const;

    Eigen::VectorXd getCurrentConfig() const;
    void setCurrentConfig(const Eigen::VectorXd& q);
    void resetConfig();
    std::pair<Eigen::VectorXd, Eigen::VectorXd> getJointLimits() const;
    int getNumJoints() const;

private:
    pinocchio::Model model;
    pinocchio::Data data;
    pinocchio::FrameIndex end_effector_id;
    Eigen::VectorXd q_init;
    Eigen::VectorXd q_current;
    // const Eigen::Quaterniond base2eef_quat{0.5, 0.5, 0.5, 0.5};   // w x y z in rviz
    // const Eigen::Matrix3d base2eef_mat = base2eef_quat.toRotationMatrix();
    // const Eigen::Vector3d eef_pos{0.110297, 0, 0.164001};    // x y z in rviz

    Eigen::Quaterniond base2eef_quat;   // w x y z in rviz
    Eigen::Matrix3d base2eef_mat;
    Eigen::Vector3d eef_pos;    // x y z in rviz

};
