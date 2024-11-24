#include "com/com_solver.hpp"

namespace COM
{
    ArmSystemCOM::ArmSystemCOM()
    {
        arm_mass = 0.0;
        armlinks_mass[0] = 1.5;
        armlinks_mass[1] = 1.5;
        armlinks_mass[2] = 1.5;
        armlinks_mass[3] = 0.75;
        armlinks_mass[4] = 0.5;

        for (int i = 0; i < 5; i++)
            arm_mass += armlinks_mass[i];

        ArmKinematics::InitializeArmSolverInstances();

        dof1 = ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link1]->GetDOF();
        dof2 = ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link2]->GetDOF();
        dof4 = ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link34]->GetDOF();
        dof5 = ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link5]->GetDOF();
        dof6 = ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->GetDOF();

        angles1.resize(dof1);
        angles2.resize(dof2);
        angles4.resize(dof4);
        angles5.resize(dof5);
        angles6.resize(dof6);
    }

    Eigen::Vector4d ArmSystemCOM::TransformCOM_inWorld(const Eigen::Vector4d &com_xyz, const Eigen::Matrix4d &TM1wrtW) // wrt b
    {
        return TM1wrtW * Kinematics::kinematics_solver->TBwrt1 * com_xyz;
    }

    Eigen::Vector4d ArmSystemCOM::ComputeArmCOM(const KDL::JntArray &arm_ik, const pair<Eigen::Vector4d, Eigen::Vector3d> &tm1wrtw)
    {
        for (int i = 0; i < dof1; i++)
            angles1(i) = arm_ik(i);
        for (int i = 0; i < dof2; i++)
            angles2(i) = arm_ik(i);
        for (int i = 0; i < dof4; i++)
            angles4(i) = arm_ik(i);
        for (int i = 0; i < dof5; i++)
            angles5(i) = arm_ik(i);
        for (int i = 0; i < dof6; i++)
            angles6(i) = arm_ik(i);

        ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link1]->SolveFK(angles1);
        ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link2]->SolveFK(angles2);
        ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link34]->SolveFK(angles4);
        ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link5]->SolveFK(angles5);
        ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->SolveFK(angles6);

        Eigen::Vector4d link_com[5];

        link_com[0] = (Conversions::KDL_2Transform(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link1]->sixDpose)).block<4, 1>(0, 3);
        link_com[1] = (Conversions::KDL_2Transform(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link2]->sixDpose)).block<4, 1>(0, 3);
        link_com[2] = (Conversions::KDL_2Transform(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link34]->sixDpose)).block<4, 1>(0, 3);
        link_com[3] = (Conversions::KDL_2Transform(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link5]->sixDpose)).block<4, 1>(0, 3);
        link_com[4] = (Conversions::KDL_2Transform(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->sixDpose)).block<4, 1>(0, 3);
      
        Eigen::Vector4d arm_com_wrt_b, sigma_miri(0, 0, 0, 1);

        for (int i = 0; i < 5; i++)
            sigma_miri += armlinks_mass[i] * link_com[i];
        sigma_miri(3) = 1.0;
        arm_com_wrt_b = sigma_miri / arm_mass;
        arm_com_wrt_b(3) = 1.0;
        
        Eigen::Matrix4d TM1wrtW;
        TM1wrtW.block<4, 1>(0, 3) = tm1wrtw.first;
        TM1wrtW.block<3, 3>(0, 0) = CommonMathsSolver::OrientationNTransformaton::ComputeR(tm1wrtw.second);
        TM1wrtW.block<1, 3>(3, 0).setZero();
        return TransformCOM_inWorld(arm_com_wrt_b, TM1wrtW);
    }

    LeggedSystemCOM::LeggedSystemCOM()
    {
        body_mass = 20.0;
        quadruped_mass = body_mass;
        for (int i = 0; i < 4; i++)
        {
            leglinks_mass[i][0] = 1.5;
            leglinks_mass[i][1] = 1.0;
            quadruped_mass += leglinks_mass[i][0] + leglinks_mass[i][1];
        }
    }

    Eigen::Vector4d LeggedSystemCOM::TransformCOM_inWorld(const Eigen::Vector4d &com_xyz, const Eigen::Matrix4d &TM1wrtW) // wrt 1
    {
        return TM1wrtW * com_xyz;
    }

    Eigen::Vector4d LeggedSystemCOM::ComputeQuadCOM(const vector<Eigen::Vector3d> &leg_ik, const pair<Eigen::Vector4d, Eigen::Vector3d> &tm1wrtw)
    {
        Eigen::Vector4d sigma_miri(0, 0, 0, 1);
        Eigen::Vector4d link_com[4][2];

        vector<Eigen::Vector4d> link2_com = Kinematics::kinematics_solver->SolveFK(leg_ik[0], leg_ik[1], leg_ik[2], leg_ik[3], Kinematics::LegLinks::link2);
        vector<Eigen::Vector4d> link3_com = Kinematics::kinematics_solver->SolveFK(leg_ik[0], leg_ik[1], leg_ik[2], leg_ik[3], Kinematics::LegLinks::link3);

        for (int i = 0; i < 4; i++)
        {
            link_com[i][0] = link2_com[i];
            link_com[i][1] = link3_com[i];
        }

        for (int leg = 0; leg < 4; leg++)
            for (int link = 0; link < 2; link++)
                sigma_miri += leglinks_mass[leg][link] * link_com[leg][link];
        sigma_miri(3) = 1.0;
        Eigen::Vector4d quadruped_com_wrt_b = sigma_miri / quadruped_mass;
        quadruped_com_wrt_b(3) = 1.0;

        Eigen::Matrix4d TM1wrtW;
        TM1wrtW.block<4, 1>(0, 3) = tm1wrtw.first;
        TM1wrtW.block<3, 3>(0, 0) = CommonMathsSolver::OrientationNTransformaton::ComputeR(tm1wrtw.second);
        TM1wrtW.block<1, 3>(3, 0).setZero();
        return TransformCOM_inWorld(quadruped_com_wrt_b, TM1wrtW);
    }

    WholeSystemCOM::WholeSystemCOM()
    {
        arm_com_solver = std::make_unique<COM::ArmSystemCOM>();
        legs_com_solver = std::make_unique<COM::LeggedSystemCOM>();
    }

    Eigen::Vector4d WholeSystemCOM::ComputeWholeSystemCOM(const vector<Eigen::Vector3d> &leg_ik, const pair<Eigen::Vector4d, Eigen::Vector3d> &tm1wrtw, const KDL::JntArray &arm_ik)
    {
        Eigen::Vector4d arm_com_wrt_w = arm_com_solver->ComputeArmCOM(arm_ik, tm1wrtw);
        Eigen::Vector4d legs_com_wrt_w = legs_com_solver->ComputeQuadCOM(leg_ik, tm1wrtw);
        Eigen::Vector4d com_wrt_w = ((arm_com_solver->arm_mass * arm_com_wrt_w) + (legs_com_solver->quadruped_mass * legs_com_wrt_w)) / (arm_com_solver->arm_mass + legs_com_solver->quadruped_mass);
        com_wrt_w(2) = -0.31;
        return com_wrt_w;
    }
}