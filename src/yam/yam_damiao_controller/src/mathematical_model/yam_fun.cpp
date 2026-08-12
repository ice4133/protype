#include "mathematical_model/yam_fun.hpp"

YamFun::YamFun(const std::string& urdf_path) {
    try {
        pinocchio::urdf::buildModel(urdf_path, model);
        std::cout << "Successfully loaded URDF model with " << model.nq << " DoF" << std::endl;
        
        data = pinocchio::Data(model);
        end_effector_id = model.getFrameId("link_6");
        std::cout << "End effector frame ID: " << end_effector_id << std::endl;
        
        q_init = pinocchio::neutral(model);
        q_current = q_init;
        std::cout << "q_current: " << q_current << std::endl;

        // Update kinematics with current joint configuration
        pinocchio::forwardKinematics(model, data, q_current);
        pinocchio::updateFramePlacement(model, data, end_effector_id);
        
        // Get end-effector placement
        const pinocchio::SE3& eef_pose = data.oMf[end_effector_id];
        
        // Extract position and orientation
        Eigen::Vector3d position = eef_pose.translation();
        Eigen::Matrix3d rotation = eef_pose.rotation();
        
        base2eef_mat = rotation;
        base2eef_quat = Eigen::Quaterniond(base2eef_mat);
        eef_pos = position;

        // Convert rotation to more readable format (quaternion)
        Eigen::Quaterniond quat(rotation);
        
        // Print the pose information
        std::cout << "End-Effector Pose:" << std::endl;
        std::cout << "Position: [" << eef_pos.x() << ", " 
                  << eef_pos.y() << ", " << eef_pos.z() << "]" << std::endl;
        std::cout << "Orientation (quaternion): [" << base2eef_quat.w() << ", " 
                  << base2eef_quat.x() << ", " << base2eef_quat.y() << ", " << base2eef_quat.z() << "]" << std::endl;
        std::cout << "Orientation (matrix):\n" << base2eef_mat << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error loading URDF: " << e.what() << std::endl;
        throw;
    }
}

// EEF POSE in Norm frame
// Norm frame's position at EEF position and orientation is same with Base frame
void YamFun::computeEEFinNorm(const Eigen::VectorXd& q, 
                       Eigen::Vector3d& position, 
                       Eigen::Quaterniond& orientation) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    
    pinocchio::SE3 oMf = data.oMf[end_effector_id];
    position = oMf.translation() - eef_pos;
    orientation = Eigen::Quaterniond(oMf.rotation() * base2eef_mat.transpose());
}

bool YamFun::computeIK(const Eigen::Vector3d& target_pos,
                       const Eigen::Quaterniond& target_quat,
                       Eigen::VectorXd& result,
                       int max_iter,
                       double eps,
                       double dt) {

    Eigen::VectorXd q = q_current;

    // Build target SE3 from inputs (IMPORTANT)
    pinocchio::SE3 target_SE3(target_quat.toRotationMatrix(), target_pos);

    // Transform Norm frame to Base frame
    // Norm frame's position at EEF position and orientation is same with Base frame
    // The target_SE3 at Norm frame
    target_SE3.translation() = target_pos + eef_pos;
    target_SE3.rotation() = target_quat.toRotationMatrix() * base2eef_mat;    

    // initialize err safely
    pinocchio::Motion err; 
    err.linear().setZero(); err.angular().setZero();

    for (int i = 0; i < max_iter; ++i) {
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);

        const pinocchio::SE3& oMf = data.oMf[end_effector_id];

        // error from current pose to target (oMf^{-1} * target)
        err = pinocchio::log6(oMf.actInv(target_SE3));

        Eigen::Vector3d vel_err = err.linear();
        Eigen::Vector3d angular_err = err.angular();

        if (vel_err.norm() < eps && angular_err.norm() < eps) {
            result = q;
            return true;
        }

        // Properly sized Jacobian
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model.nv);
        pinocchio::computeFrameJacobian(model, data, q, end_effector_id, J);

        // --- Damped least squares (more stable than raw JT) ---
        const double lambda = 1e-3; // tune
        Eigen::MatrixXd JJt = J * J.transpose();
        Eigen::VectorXd v = err.toVector();

        // solve (JJ^T + lambda^2 I) x = v  => x then dq = J^T * x
        Eigen::MatrixXd A = JJt;
        A.diagonal().array() += lambda * lambda;
        Eigen::VectorXd x = A.ldlt().solve(v);        // 6x1
        Eigen::VectorXd dq = J.transpose() * x;      // nv x 1
        // dq *= dt;                                    // step size

        // integrate in configuration manifold
        q = pinocchio::integrate(model, q, dq);

        // normalize quaternion joints (if any)
        pinocchio::normalize(model, q);

        // If *no* quaternion/floating base joints, you can clamp with limits:
        // (Only do this if your model has no quaternion blocks; otherwise skip.)
        if(model.nq == model.nv) {
            for (int j = 0; j < model.nq; ++j) {
                q[j] = std::max(model.lowerPositionLimit[j],
                                std::min(model.upperPositionLimit[j], q[j]));
            }
        }
    }

    // recompute error for printing (safe, guaranteed initialized)
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    err = pinocchio::log6(data.oMf[end_effector_id].actInv(target_SE3));

    std::cout << "err: " << err.toVector().norm() << std::endl;
    std::cout << "Velocity error: " << err.linear().norm() << std::endl;
    std::cout << "Angular error: " << err.angular().norm() << std::endl;

    return false;
}

Eigen::VectorXd YamFun::gravityCompensation(const Eigen::VectorXd& q) {
    // Check input size
    if (q.size() != model.nq) {
        std::cerr << "gravityCompensation: input q size (" << q.size()
                  << ") does not match model.nq (" << model.nq << "). Returning zeros.\n";
        return Eigen::VectorXd::Zero(model.nv);
    }

    // Zero generalized velocities and accelerations for gravity-only term
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

    // Update kinematics (keeps data consistent)
    pinocchio::forwardKinematics(model, data, q);
    // Optionally: pinocchio::computeAllTerms(model, data, q); // not required for rnea

    // Use RNEA: tau = M(q)*a + C(q,v) + g(q)
    // with v = 0, a = 0 -> tau = g(q)
    Eigen::VectorXd tau_g;
    try {
        tau_g = pinocchio::rnea(model, data, q, v, a);
    } catch (const std::exception& e) {
        std::cerr << "pinocchio::rnea failed: " << e.what() << "\n";
        return Eigen::VectorXd::Zero(model.nv);
    }

    // Ensure correct size (safety)
    if (tau_g.size() != model.nv) {
        std::cerr << "gravityCompensation: unexpected tau size " << tau_g.size()
                  << ", expected " << model.nv << ". Resizing to zeros.\n";
        return Eigen::VectorXd::Zero(model.nv);
    }

    return tau_g;
}

Eigen::VectorXd YamFun::InverseDynamics(const Eigen::VectorXd& q,const Eigen::VectorXd& dq,const Eigen::VectorXd& ddq) {
    // Check input size
    if (q.size() != model.nq ||dq.size() != model.nq||ddq.size() != model.nq) {
        std::cerr << "InverseDynamics: input q size (" << q.size()
                  << ") does not match model.nq (" << model.nq << "). Returning zeros.\n";
        return Eigen::VectorXd::Zero(model.nv);
    }

    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

    pinocchio::forwardKinematics(model, data, q);

    // Use RNEA: tau = M(q)*a + C(q,v) + g(q)
    // with v = 0, a = 0 -> tau = g(q)
    Eigen::VectorXd tau;
    try {
        tau = pinocchio::rnea(model, data, q, dq, a);
    } catch (const std::exception& e) {
        std::cerr << "pinocchio::rnea failed: " << e.what() << "\n";
        return Eigen::VectorXd::Zero(model.nv);
    }

    // Ensure correct size (safety)
    if (tau.size() != model.nv) {
        std::cerr << "InverseDynamics: unexpected tau size " << tau.size()
                  << ", expected " << model.nv << ". Resizing to zeros.\n";
        return Eigen::VectorXd::Zero(model.nv);
    }

    return tau;
}

Eigen::VectorXd YamFun::gravityCompensationCurrent() const {
    // q_current is mutable member in your class; this method is const so we copy it
    Eigen::VectorXd q = q_current;
    // Note: rnea uses model and data; data is mutable — here we cast away constness safely
    // because forwardKinematics and rnea mutate data. If you'd prefer, make this non-const.
    YamFun* self = const_cast<YamFun*>(this);
    return self->gravityCompensation(q);
}

Eigen::VectorXd YamFun::getCurrentConfig() const {
    return q_current;
}

void YamFun::setCurrentConfig(const Eigen::VectorXd& q) {
    if (q.size() == model.nq) {
        q_current = q;
    } else {
        std::cerr << "Error: Configuration size mismatch" << std::endl;
    }
}

void YamFun::resetConfig() {
    q_current = q_init;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> YamFun::getJointLimits() const {
    return {model.lowerPositionLimit, model.upperPositionLimit};
}

int YamFun::getNumJoints() const {
    return model.nq;
}
