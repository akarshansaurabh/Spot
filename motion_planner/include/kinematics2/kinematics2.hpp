#ifndef KINEMATICS2_HPP
#define KINEMATICS2_HPP

#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <memory>

using namespace std;

const double PI = 3.14;

namespace Kinematics2
{
    struct CartesianState
    {
        Eigen::Matrix4d cart_pose;
        Eigen::Matrix<double, 6, 1> cart_velocity, cart_acceleration, cart_jerk;
    };

    struct JointState
    {
        Eigen::VectorXd joint_angles, joint_velocities, joint_accelerations, joint_jerks;
    };

    enum class IKSelector
    {
        ElbowUp,
        ElbowDown,
        ByDefault
    };

    // DHTransformationMatrix needs dh_table which is a 2d matrix of order DOFx3
    template <int DOF>
    class DHTransformationMatrix
    {
    private:
    public:
        Eigen::Matrix4d dht_matrix;
        DHTransformationMatrix(const Eigen::Matrix<double, DOF, 3> &dh_table, double t, int jnt_index);
        void Print();
    };

    template <typename pose_type>
    class RobotKinematics
    {
    public:
        static pair<vector<Eigen::Vector3d>, Eigen::VectorXd> spot_ik;
        virtual bool SolveIK(const pose_type &pose, const Eigen::VectorXd &prev_ik, const IKSelector &ik_selector, int chain_index, Eigen::VectorXd &ik) = 0;
        virtual bool SolveFK(const Eigen::VectorXd &thetas, int chain_index, pose_type &pose) = 0;
        virtual ~RobotKinematics() = default;
    };

    template <typename pose_type>
    class ArmKinematics : public RobotKinematics<pose_type>
    {
    public:
        Eigen::VectorXd arm_ik;
        std::unique_ptr<JointState> arm_jointstate_tracker;
        std::unique_ptr<CartesianState> arm_cartstate_tracker;

        bool SolveIK(const pose_type &pose, const Eigen::VectorXd &prev_ik, const IKSelector &ik_selector, int chain_index, Eigen::VectorXd &ik) override;
        bool SolveFK(const Eigen::VectorXd &thetas, int chain_index, pose_type &pose) override;
    };

    template <typename pose_type>
    class LegKinematics : public RobotKinematics<pose_type>
    {
    public:
        vector<Eigen::Vector3d> four_legs_ik;
        vector<std::unique_ptr<JointState>> leg_jointstate_tracker;
        vector<std::unique_ptr<CartesianState>> leg_cartstate_tracker;
        std::unique_ptr<CartesianState> body_cartstate_tracker;

        bool SolveIK(const pose_type &pose, const Eigen::VectorXd &prev_ik, const IKSelector &ik_selector, int chain_index, Eigen::VectorXd &ik) override;
        bool SolveIK(const vector<pose_type> &poses, const vector<Eigen::VectorXd> &prev_iks, const vector<IKSelector> &ik_selectors);
        bool SolveFK(const Eigen::VectorXd &thetas, int chain_index, pose_type &pose) override;
    };
}

#endif