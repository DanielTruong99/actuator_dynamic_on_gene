#ifndef IK_SOLVER_H
#define IK_SOLVER_H

#pragma once
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>

class IKSolver
{
    public:
        IKSolver(const std::string& urdf_path, const std::string& ee_frame)
        : _ee_frame_name(ee_frame)
        {
            /* Load URDF model */
            pinocchio::urdf::buildModel(urdf_path, _model);
            _data = pinocchio::Data(_model);
            _ee_frame_id = _model.getFrameId(ee_frame);

            /* Initialize control gains */
            _Kp = 2.0;
            _Ko = 2.0;
            _dt = 0.01;
            _W.setIdentity();
            _W.topLeftCorner<3,3>() *= 0.1;
            _W.bottomRightCorner<3,3>() *= 1.0;

            /* Initialize joint limits */
            _init_joint_limits();
        }

        /**
         * @brief Computes the inverse kinematics for a desired end-effector pose.
         *
         * This function computes the joint configuration that achieves the desired end-effector position and orientation.
         * It uses the current joint configuration and optionally the current end-effector pose to compute the required joint velocities.
         *
         * @param r_d Desired end-effector position as a 3D vector.
         * @param q_d Desired end-effector orientation as a quaternion.
         * @param q Current joint configuration as a vector.
         * @param r Optional current end-effector position (if not provided, it will be computed).
         * @param q_curr Optional current end-effector orientation (if not provided, it will be computed).
         * @return Computed joint velocities as a vector.
         */
        Eigen::VectorXd compute(
            const Eigen::Vector3d& r_d,
            const Eigen::Quaterniond& q_d,
            const Eigen::VectorXd& q,
            const Eigen::Vector3d* r = nullptr,
            const Eigen::Quaterniond* q_curr = nullptr
        );

        void set_gains(double kp, double ko) { _Kp = kp; _Ko = ko; }
        void set_step(double dt) { _dt = dt; }
        void set_weight(double w_pos, double w_rot) 
        {
            _W.setIdentity();
            _W.topLeftCorner<3,3>() *= w_pos;
            _W.bottomRightCorner<3,3>() *= w_rot;
        }
        void set_joint_limits(const Eigen::VectorXd& lower, const Eigen::VectorXd& upper) 
        {
            _joint_lower = lower;
            _joint_upper = upper;
        }

    private:
        std::string _ee_frame_name;
        pinocchio::Model _model;
        pinocchio::Data _data;
        pinocchio::FrameIndex _ee_frame_id;

        double _Kp, _Ko, _dt;
        Eigen::MatrixXd _W;

        Eigen::VectorXd _joint_lower, _joint_upper;

        void _init_joint_limits();
        void _clamp_joint_limits(Eigen::VectorXd& q) const;

        
        /**
         * @brief Computes the desired 6D twist (linear and angular velocity) to move from the actual pose to the desired pose.
         *
         * This function calculates the desired spatial velocity (twist) required for a robot end-effector to move from its current
         * position and orientation (`r_act`, `q_act`) to a target position and orientation (`r_d`, `q_d`). The result is stored in
         * `v6_desired`, where the first three elements represent the linear velocity and the last three represent the angular velocity.
         *
         * @param[in] r_d        Desired position as a 3D vector.
         * @param[in] q_d        Desired orientation as a quaternion.
         * @param[in] r_act      Actual position as a 3D vector.
         * @param[in] q_act      Actual orientation as a quaternion.
         * @param[out] v6_desired Output 6D vector containing the desired twist (linear and angular velocity).
         */
        void _compute_des_twist(
            const Eigen::Vector3d& r_d,
            const Eigen::Quaterniond& q_d,
            const Eigen::Vector3d& r_act,
            const Eigen::Quaterniond& q_act,
            Eigen::Matrix<double,6,1>& v6_desired
        ) const;

};

#endif // IK_SOLVER_H