#ifndef COM_SOLVER_HPP
#define COM_SOLVER_HPP

#include "kinematics/kinematics.hpp"
#include "kinematics/arm_kinematics.hpp"

#include "maths/commonmathssolver.hpp"

#include <eigen3/Eigen/Dense>
#include <memory>

namespace COM
{
    class BaseCOM
    {
    public:
        virtual Eigen::Vector4d TransformCOM_inWorld(const Eigen::Vector4d &com_xyz, const Eigen::Matrix4d &TM1wrtW) = 0;
        virtual ~BaseCOM() = default;
    };

    class ArmSystemCOM : public BaseCOM
    {
    private:
        double armlinks_mass[5];
        int dof1, dof2, dof4, dof5, dof6;
        KDL::JntArray angles1, angles2, angles4, angles5, angles6;

    public:
        double arm_mass;
        ArmSystemCOM();
        Eigen::Vector4d ComputeArmCOM(const KDL::JntArray &arm_ik, const pair<Eigen::Vector4d, Eigen::Vector3d> &tm1wrtw);
        Eigen::Vector4d TransformCOM_inWorld(const Eigen::Vector4d &com_xyz, const Eigen::Matrix4d &TM1wrtW) override;
    };

    class LeggedSystemCOM : public BaseCOM
    {
    private:
        double leglinks_mass[4][2];
        double body_mass;

    public:
        double quadruped_mass;
        LeggedSystemCOM();
        Eigen::Vector4d ComputeQuadCOM(const vector<Eigen::Vector3d> &leg_ik, const pair<Eigen::Vector4d, Eigen::Vector3d> &tm1wrtw);
        Eigen::Vector4d TransformCOM_inWorld(const Eigen::Vector4d &com_xyz, const Eigen::Matrix4d &TM1wrtW) override;
    };

    class WholeSystemCOM
    {
    private:
        std::unique_ptr<ArmSystemCOM> arm_com_solver;
        std::unique_ptr<COM::LeggedSystemCOM> legs_com_solver;

    public:
        WholeSystemCOM();
        Eigen::Vector4d ComputeWholeSystemCOM(const vector<Eigen::Vector3d> &leg_ik, const pair<Eigen::Vector4d, Eigen::Vector3d> &tm1wrtw, const KDL::JntArray &arm_ik);
    };
}

#endif