#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>

using namespace std;

const double PI = 3.14;

namespace Kinematics
{
    enum class LegLinks
    {
        link2,
        link3,
        all
    };

    struct leg_state
    {
        Eigen::Vector4d foot_xyz;
        Eigen::Vector3d theta;
    };

    // body_xyz -> x,y,z of geometric center wrt World
    // body_rpy -> r,p,y of body wrt world
    // leg_states -> frame 1

    // Frame 0 and Frame W are fixed in world. Frame 1 is fixed on the robot body. At t=0, F1=F0.
    struct robot_state
    {
        Eigen::Vector4d body_xyz;
        Eigen::Matrix3d body_orientation;
        Eigen::Vector3d body_rpy;
        leg_state leg_states[4]; // wrt F1
    };

    struct Leg
    {
        // O is inner, O_ is outer
        Eigen::Vector4d O, xyz, O_; // xyz -> wrt 0
        Eigen::Vector3d theta;
        Eigen::Matrix4d T, T_inverse;
        Eigen::Matrix3d dh_table, dh_table_com1, dh_table_com2;
        double d0, l1, l2, r1, r2;
    };

    class DHTransformationMatrix
    {
    private:
    public:
        Eigen::Matrix4d dht_matrix;
        DHTransformationMatrix(const Eigen::Matrix3d &dh_table, double t, int jnt_index);
        void Print();
    };

    class Kinematics
    {
    private:
        Eigen::Vector3d SolveIK(const Eigen::Vector4d &xyz_wrt_leg_frame, int leg_index);
        Eigen::Vector4d SolveFK(const Eigen::Vector3d &thetas, int leg_index, const LegLinks &link_info);

    public:
        Eigen::Matrix4d T0wrtW, T1wrt0, T2wrt1, TBwrt1;
        Eigen::Matrix4d T0wrtW_inverse, TBwrt1_inverse;
        vector<vector<Eigen::Vector4d>> stance_legs_wrt_world;
        Leg legs[4];
        Kinematics();
        vector<Eigen::Vector3d> SolveIK(const Eigen::Vector4d &l1_xyz_wrt_o, const Eigen::Vector4d &l2_xyz_wrt_o,
                                        const Eigen::Vector4d &l3_xyz_wrt_o, const Eigen::Vector4d &l4_xyz_wrt_o);
        vector<Eigen::Vector4d> SolveFK(const Eigen::Vector3d &l1_t, const Eigen::Vector3d l2_t,
                                        const Eigen::Vector3d &l3_t, const Eigen::Vector3d &l4_t, const LegLinks &link_info);
    };

    extern std::unique_ptr<Kinematics> kinematics_solver;
    extern std::unique_ptr<robot_state> state_tracker;
}

#endif