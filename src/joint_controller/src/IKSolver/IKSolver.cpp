#include "IKSolver.h"

Eigen::VectorXd IKSolver::compute(
    const Eigen::Vector3d& r_d,
    const Eigen::Quaterniond& q_d,
    const Eigen::VectorXd& q,
    const Eigen::Vector3d* r,
    const Eigen::Quaterniond* q_curr
) 
{
    Eigen::Vector3d r_act;
    Eigen::Quaterniond q_act;
    if (r && q_curr) 
    {
        r_act = *r;
        q_act = *q_curr;
    } 
    else 
    {
        pinocchio::forwardKinematics(_model, _data, q);
        pinocchio::updateFramePlacements(_model, _data);
        const auto& oMf = _data.oMf[_ee_frame_id];
        r_act = oMf.translation();
        q_act = Eigen::Quaterniond(oMf.rotation());
    }

    /* Compute desired 6D twist to make the end-effector reach the target pose */
    Eigen::Matrix<double,6,1> v6_desired;
    _compute_des_twist(r_d, q_d, r_act, q_act, v6_desired);

    /* Compute the Jacobian */
    Eigen::MatrixXd J_full(6, _model.nv);
    pinocchio::computeFrameJacobian(_model, _data, q, _ee_frame_id, pinocchio::WORLD, J_full);

    /* Solve for q_dot
        W * J * q_dot = W * v6_desired
    */
    Eigen::MatrixXd WJ = _W * J_full;
    Eigen::Matrix<double,6,1> Wv = _W * v6_desired;
    Eigen::VectorXd q_dot = WJ.colPivHouseholderQr().solve(Wv);

    /* Euler integration */
    Eigen::VectorXd q_next;
    q_next = q + q_dot * _dt;
    _clamp_joint_limits(q_next);
    return q_next;
}


void IKSolver::_compute_des_twist(
    const Eigen::Vector3d& r_d,
    const Eigen::Quaterniond& q_d,
    const Eigen::Vector3d& r_act,
    const Eigen::Quaterniond& q_act,
    Eigen::Matrix<double,6,1>& v6_desired
) const 
{
    /* Position control
        Simple PD law
    */
    Eigen::Vector3d v_d = _Kp * (r_d - r_act);

    /* Orientation control
        Using quaternion error
        q_err = q_act^-1 * q_d
        w_d = 2 * Ko * q_err.vec()
    */
    Eigen::Quaterniond q_err = q_act.conjugate() * q_d; // Quaternion error
    if (q_err.w() < 0) q_err.coeffs() *= -1; // Ensure the quaternion is normalized and has a positive scalar part

    Eigen::Vector3d w_d = 2.0 * _Ko * q_err.vec(); // feedback law for angular velocity

    /* Combine linear and angular velocities into a 6D vector */
    v6_desired.head<3>() = v_d;
    v6_desired.tail<3>() = w_d;
}


void IKSolver::_init_joint_limits()
{
    _joint_lower = Eigen::VectorXd::Constant(_model.nq, -1e6);
    _joint_upper = Eigen::VectorXd::Constant(_model.nq,  1e6);
    Eigen::VectorXd lower = _model.lowerPositionLimit;
    Eigen::VectorXd upper = _model.upperPositionLimit;
    for (size_t i = 0; i < _model.joints.size(); ++i) 
    {
        const auto& joint = _model.joints[i];
        if (joint.nq() == 1) 
        {
            int joint_index = joint.idx_q();
            _joint_lower[joint_index] = lower[joint_index];
            _joint_upper[joint_index] = upper[joint_index];
        }
    }    
}

void IKSolver::_clamp_joint_limits(Eigen::VectorXd& q) const
{
    for (int i = 0; i < q.size(); ++i) 
    {
        q[i] = std::min(_joint_upper[i], std::max(_joint_lower[i], q[i]));
    }    
}