#pragma once
#ifndef YOUR_PACKAGE__YOUR_HEADER_HPP_
#define YOUR_PACKAGE__YOUR_HEADER_HPP_

#include <iostream>
#include <vector>
#include <cmath>
#include "mathematical_model/modern_robotics.hpp"
#include <tinyxml2.h>
#include <rclcpp/rclcpp.hpp>
#include "mathematical_model/fifter.hpp"
#include "mathematical_model/Robot_math.hpp"
#define ARM_DOF 6

class ARX_R5_fun {
public:
    // Constructor and Destructor
    ARX_R5_fun();
    ~ARX_R5_fun();
    
    // Model configuration
    void isDynamicParamModel(bool isDynamic) {
        isDynamicParamModel_ = isDynamic;
    }

    // Kinematics functions
    Eigen::MatrixXd ARX_R5_FK(const Eigen::VectorXd thetaList);
    Eigen::VectorXd ARX_R5_IK(Eigen::VectorXd thetaList, const Eigen::MatrixXd T);
    Eigen::MatrixXd ARX_R5_JacobianSpace(const Eigen::VectorXd thetaList);
    
    // Dynamics functions
    Eigen::VectorXd NumericalDifferentiation(const Eigen::VectorXd& dthetalist, double dt);
    Eigen::MatrixXd admittanceForceControl(const Eigen::MatrixXd &pm_now, 
                                         const Eigen::MatrixXd &Ft_in_sensor,
                                         const double dt_);
    Eigen::MatrixXd Get_ARX_Hb_fir(const Eigen::VectorXd& thetalist, 
                                  const Eigen::VectorXd& dthetalist, 
                                  const Eigen::VectorXd& ddthetalist);
    void Get_ARX_Hb_Tor(const Eigen::VectorXd& thetalist, 
                       const Eigen::VectorXd& dthetalist, 
                       const Eigen::VectorXd& ddthetalist,
                       const Eigen::VectorXd& joint_fit_tor);
    void Get_ARX_R5_Yparam();
    
    // Utility functions
    void saveVectorToXml(const Eigen::VectorXd& vec, const std::string& filename);
    Eigen::VectorXd loadVectorFromXml(const std::string& filename);
    Eigen::MatrixXd InverseDynamics(const Eigen::VectorXd& thetalist, 
                                  const Eigen::VectorXd& dthetalist, 
                                  const Eigen::VectorXd& ddthetalist);
    Eigen::MatrixXd GravityForces(const Eigen::VectorXd& thetalist);
    Eigen::MatrixXd GravityAndFirForces(const Eigen::VectorXd& thetalist,
                                      const Eigen::VectorXd& dthetalist);
    void cleanVector();
    
    // Force transformations
    void Force_to_Force(const Eigen::MatrixXd &T,
                       const Eigen::MatrixXd &force_in_body, 
                       Eigen::MatrixXd &force_in_forword);
    auto FT_in_world_to_FT_in_sensor(const Eigen::MatrixXd &T,
                                    const Eigen::MatrixXd &FT_in_world, 
                                    Eigen::MatrixXd &FT_in_sensor);
    auto FT_in_sensor_to_FT_in_world(const Eigen::MatrixXd &T,
                                    const Eigen::MatrixXd &FT_in_sensor, 
                                    Eigen::MatrixXd &FT_in_world);
    Eigen::MatrixXd Get_Ft_in_world(const Eigen::VectorXd& thetalist,
                                   const Eigen::VectorXd& tor);
    
    // Updated interface using vectors
    void updatArmState(const std::vector<float>& motor_pos, 
                      const std::vector<float>& motor_vel, 
                      const std::vector<float>& motor_tau);
    
    void joint2motor(const std::vector<float>& q,
                    const std::vector<float>& dq,
                    const std::vector<float>& ddq,
                    const std::vector<float>& tau_joint,
                    std::vector<float>& motor_pos,
                    std::vector<float>& motor_vel,
                    std::vector<float>& motor_acc,
                    std::vector<float>& tau_motor);

    // State vectors
    std::vector<float> desir_joint_pos = std::vector<float>(ARM_DOF, 0.0f);
    std::vector<float> desir_joint_vel = std::vector<float>(ARM_DOF, 0.0f);
    std::vector<float> desir_joint_acc = std::vector<float>(ARM_DOF, 0.0f);
    std::vector<float> desir_joint_tau = std::vector<float>(ARM_DOF, 0.0f);

    std::vector<float> current_joint_pos = std::vector<float>(ARM_DOF, 0.0f);
    std::vector<float> current_joint_vel = std::vector<float>(ARM_DOF, 0.0f);
    std::vector<float> current_joint_acc = std::vector<float>(ARM_DOF, 0.0f);
    std::vector<float> current_joint_tau = std::vector<float>(ARM_DOF, 0.0f);
    
    std::vector<float> motor_direction = {1, 1, 1, 1, 1, 1};

private:
    bool isDynamicParamModel_ = false;
    Eigen::MatrixXd M_;
    Eigen::MatrixXd Slist_;
    Eigen::VectorXd gravity_vector_ = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd Y_;
    std::vector<Eigen::MatrixXd> Hb_vector_;
    std::vector<Eigen::VectorXd> Joint_tor_vector_;
    
    // Parameter estimation methods
    Eigen::VectorXd leastSquaresEstimation(const Eigen::MatrixXd& Y, const Eigen::MatrixXd& tau);
    Eigen::VectorXd lessSquares(const Eigen::MatrixXd& Y, const Eigen::MatrixXd& tau);
    Eigen::VectorXd lessSquaresfor(const Eigen::MatrixXd& Y, const Eigen::MatrixXd& tau);
};

#endif  // YOUR_PACKAGE__YOUR_HEADER_HPP_