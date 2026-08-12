#include "yam_damiao_controller/Damiao_6dof_node.hpp"
namespace yam
{
    Damiao6dofInterfacesThread::Damiao6dofInterfacesThread(const std::string &can_name)
    : robotControl(
        std::vector<damiao::Motor>{
            // Arm motors (indices 0-5)
            damiao::Motor(damiao::DM4340, 0x01, 0x11),
            damiao::Motor(damiao::DM4340, 0x02, 0x12),
            damiao::Motor(damiao::DM4340, 0x03, 0x13),
            damiao::Motor(damiao::DM4310, 0x04, 0x14),
            damiao::Motor(damiao::DM4310, 0x05, 0x15),
            damiao::Motor(damiao::DM4310, 0x06, 0x16),
            // Gripper motor (index 6)
            damiao::Motor(damiao::DM4310, 0x07, 0x17)
        },
        can_name,
        std::vector<float>{-1.57, 0, 0, -1.57, -1.57, -2, -5},       // limits min (arm + gripper)
        std::vector<float>{1.57, 3.65, 3.13, 1.57, 1.57, 2, 0}       // limits max (arm + gripper)
      )
    {
    }

    Damiao6dofInterfacesThread::~Damiao6dofInterfacesThread() {
        robotControl.DisableMotors();
        std::cout << "Damiao6dofInterfacesThread destroy ok" << std::endl;
    }

    void Damiao6dofInterfacesThread::Init() {
        std::cout << "Start Init robotControl (arm + gripper)" << std::endl;
        robotControl.InitMotors();
    }

    void Damiao6dofInterfacesThread::Enable(){
        robotControl.EnableMotors();
    }

    void Damiao6dofInterfacesThread::Disable(){
        robotControl.DisableMotors();
    }

    void Damiao6dofInterfacesThread::SetZeroMotors(){
        robotControl.SetZeroMotors();
    }

    void Damiao6dofInterfacesThread::updateYamStatus()
    {
        robotControl.updateMotorState();
    }

    void Damiao6dofInterfacesThread::updateArmStatus()
    {
        robotControl.updateMotorState();
    }

    void Damiao6dofInterfacesThread::updateGripStatus()
    {
        robotControl.updateMotorState();
    }

    bool Damiao6dofInterfacesThread::checkRobotErrors() {
        return robotControl.CheckMotorErrors();
    }

    std::string Damiao6dofInterfacesThread::getErrorsDescription() {
        robotControl.CheckMotorErrors();
        return robotControl.CheckErrorsDescription();
    }

    std::vector<float> Damiao6dofInterfacesThread::getJointPositions(){
        return std::vector<float>(
            robotControl.current_motor_pos.begin(),
            robotControl.current_motor_pos.begin() + ARM_DOF
        );
    }

    std::vector<float> Damiao6dofInterfacesThread::getJointVelocities(){
        return std::vector<float>(
            robotControl.current_motor_vel.begin(),
            robotControl.current_motor_vel.begin() + ARM_DOF
        );
    }

    std::vector<float> Damiao6dofInterfacesThread::getJointCurrent(){
        return std::vector<float>(
            robotControl.current_motor_tau.begin(),
            robotControl.current_motor_tau.begin() + ARM_DOF
        );
    }

    std::vector<float> Damiao6dofInterfacesThread::getGripperPositions() {
        std::vector<float> result;
        result.reserve(GRIPPER_DOF);
        for (size_t i = 0; i < GRIPPER_DOF; ++i) {
            float pos = robotControl.current_motor_pos[ARM_DOF + i];
            result.push_back(static_cast<float>(gripper_map.theta2cmd(pos)));
        }
        return result;
    }

    std::vector<float> Damiao6dofInterfacesThread::getGripperVelocities() {
        std::vector<float> result;
        result.reserve(GRIPPER_DOF);
        for (size_t i = 0; i < GRIPPER_DOF; ++i) {
            float vel = robotControl.current_motor_vel[ARM_DOF + i];
            result.push_back(-vel);
        }
        return result;
    }

    std::vector<float> Damiao6dofInterfacesThread::getGripperCurrent() {
        std::vector<float> result;
        result.reserve(GRIPPER_DOF);
        for (size_t i = 0; i < GRIPPER_DOF; ++i) {
            float tau = robotControl.current_motor_tau[ARM_DOF + i];
            result.push_back(-tau);
        }
        return result;
    }

    // Unified API - sends all 7 motors in one transaction
    void Damiao6dofInterfacesThread::setRobotPosition(
        const std::vector<float>& arm_pos, const std::vector<float>& arm_vel,
        const std::vector<float>& arm_P, const std::vector<float>& arm_D, const std::vector<float>& arm_tor,
        const std::vector<float>& gripper_pos, const std::vector<float>& gripper_vel,
        const std::vector<float>& gripper_P, const std::vector<float>& gripper_D, const std::vector<float>& gripper_tor)
    {
        std::vector<float> full_pos(ARM_DOF + GRIPPER_DOF);
        std::vector<float> full_vel(ARM_DOF + GRIPPER_DOF);
        std::vector<float> full_P(ARM_DOF + GRIPPER_DOF);
        std::vector<float> full_D(ARM_DOF + GRIPPER_DOF);
        std::vector<float> full_tor(ARM_DOF + GRIPPER_DOF);

        // Arm (indices 0-5)
        for (size_t i = 0; i < ARM_DOF; ++i) {
            full_pos[i] = arm_pos[i];
            full_vel[i] = arm_vel[i];
            full_P[i] = arm_P[i];
            full_D[i] = arm_D[i];
            full_tor[i] = arm_tor[i];
        }

        // Gripper (index 6) with mapping
        for (size_t i = 0; i < GRIPPER_DOF; ++i) {
            float clamped_pos = std::clamp(gripper_pos[i], 0.0f, 5.0f);
            full_pos[ARM_DOF + i] = static_cast<float>(gripper_map.cmd2theta(clamped_pos));
            full_vel[ARM_DOF + i] = gripper_vel[i];
            full_P[ARM_DOF + i] = gripper_P[i];
            full_D[ARM_DOF + i] = gripper_D[i];
            full_tor[ARM_DOF + i] = gripper_tor[i];
        }

        robotControl.MitCtrl(full_pos, full_vel, full_P, full_D, full_tor);
    }

}
