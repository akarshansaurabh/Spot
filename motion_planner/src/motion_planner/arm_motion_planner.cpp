#include "motion_planner/arm_motion_planner.hpp"

namespace ArmMotionPlanner
{
    PathPlanner::PathPlanner(double precision_) : precision(precision_)
    {
        T_e_wrt_b = Conversions::KDL_2Transform(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->sixDpose);
    }

    void PathPlanner::LinePlanner(const Eigen::Matrix4d &goal)
    {
        arm_ik.clear();
        Curves::Line line(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->pose_6_wrt_0.block<4, 1>(0, 3), goal.block<4, 1>(0, 3));
        vector<pair<Eigen::Vector4d, Eigen::Quaterniond>> line_path = line.Generate6DPoints(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->pose_6_wrt_0, goal, precision);
        for (const auto &pose : line_path)
        {
            Eigen::Matrix4d pose_transform;
            pose_transform.block<4, 1>(0, 3) = pose.first;
            pose_transform.block<3, 3>(0, 0) = pose.second.toRotationMatrix();
            pose_transform.block<1, 3>(3, 0).setZero();
            ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->pose_6_wrt_0 = pose_transform;
            ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->SolveIK(Conversions::Transform_2KDL(pose_transform));
            arm_ik.push_back(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->q);
        }
    }

    vector<Eigen::Vector4d> PathPlanner::GenerateBSPlineCP(const Eigen::Matrix4d &goal, Eigen::Matrix4d &current_pose, double z_shift, bool b)
    {
        ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]
            ->SolveFK(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->q); // q_home
        if (b)
            current_pose = Conversions::KDL_2Transform(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->sixDpose);
        // Eigen::Matrix4d current_pose = Conversions::KDL_2Transform(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->sixDpose);
     
        double x_shift = goal(0, 3) - current_pose(0, 3);
        double y_shift = (goal(1, 3) - current_pose(1, 3)) / 6.0;
        // double stride_h = fabs(goal(2, 3) - current_pose(2, 3));
        double stride_h = (goal(2, 3) - current_pose(2, 3));

        double del_x = x_shift / 3.0;
        double h1 = z_shift / 3.5;

        Eigen::Vector4d p1 = current_pose.block<4, 1>(0, 3);
        Eigen::Vector4d p2(p1(0) - h1, p1(1) + (1 * y_shift), p1(2) + (2.5 * fabs(h1)), 1.0);
        Eigen::Vector4d p3(p1(0) - h1, p1(1) + (2 * y_shift), p1(2) + fabs(z_shift), 1.0);
        Eigen::Vector4d p4(p1(0), p1(1) + (3 * y_shift), p1(2) + fabs(z_shift), 1.0);

        // Eigen::Vector4d p5(p1(0) + del_x, p1(1) + (4 * y_shift), goal(2, 3) + (2.0 * stride_h) / 3.0, 1.0);
        // Eigen::Vector4d p6(p1(0) + (2.0 * del_x), p1(1) + (5 * y_shift), goal(2, 3) + (1.0 * stride_h) / 3.0, 1.0);
        // Eigen::Vector4d p7(p1(0) + (x_shift), p1(1) + (6 * y_shift), goal(2, 3), 1.0);
        Eigen::Vector4d p5(p1(0) + del_x, p1(1) + (4 * y_shift), goal(2, 3) - (2.0 * stride_h) / 3.0, 1.0);
        Eigen::Vector4d p6(p1(0) + (2.0 * del_x), p1(1) + (5 * y_shift), goal(2, 3) - (1.0 * stride_h) / 3.0, 1.0);
        Eigen::Vector4d p7(p1(0) + (x_shift), p1(1) + (6 * y_shift), goal(2, 3), 1.0);

        vector<Eigen::Vector4d> ans = {p1, p2, p3, p4, p5, p6, p7};
        return ans;
    }

    vector<Eigen::Vector4d> PathPlanner::DancingGenerateBSPlineCP(vector<Eigen::Vector4d> &cp_i, const Eigen::Matrix4d &goal, double z_shift, double turning_radius, int index)
    {
        Eigen::Vector4d p2;
        cp_i.push_back(goal.block<4, 1>(0, 3));
        p2 = cp_i[index];
        p2(1) = turning_radius;
        Curves::Arc arc(cp_i[0], p2, cp_i[cp_i.size() - 1], "C");
        arc.point_generation = "default";
        int num = 200;
        vector<Eigen::Vector4d> arc_points = arc.Generate3DPoints(num);
        for (int i = 1; i < cp_i.size() - 1; i++)
            for (int j = 0; j < arc_points.size(); j++)
            {
                if (fabs(arc_points[j](0) - cp_i[i](0)) < 0.01)
                {
                    cp_i[i](1) = arc_points[j](1);
                    break;
                }
            }
        cout << "error" << endl;
        for (auto i : cp_i)
            cout << i.adjoint() << endl;
        return cp_i;
    }

    void PathPlanner::BSplinePlanner(Curves::BSpline &bspline_curve, const Eigen::Matrix4d &goal, int num, Eigen::Matrix4d &pose1)
    {
        arm_ik.clear();
        // Eigen::Matrix4d pose1 = Conversions::KDL_2Transform(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->sixDpose);

        vector<pair<Eigen::Vector4d, Eigen::Quaterniond>> spline_path = bspline_curve.Generate6DPoints(pose1, goal, num);
        int i = 1;
        for (const auto &pose : spline_path)
        {
            Eigen::Matrix4d pose_transform;
            pose_transform.block<4, 1>(0, 3) = pose.first;
            pose_transform.block<3, 3>(0, 0) = pose.second.toRotationMatrix();
            pose_transform.block<1, 3>(3, 0).setZero();
            ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->pose_6_wrt_0 = pose_transform;
            T_e_wrt_b = pose_transform;
            ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->SolveIK(Conversions::Transform_2KDL(pose_transform));
            arm_ik.push_back(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->q);
        }
    }

    void PathPlanner::ArcPlanner(const Eigen::Matrix4d &p1, const Eigen::Matrix4d &p2, const Eigen::Matrix4d &p3, int num, const string &str)
    {
        arm_ik.clear();
        Curves::Arc arc(p1.block<4, 1>(0, 3), p2.block<4, 1>(0, 3), p3.block<4, 1>(0, 3), "C");
        arc.point_generation = "default";
        vector<pair<Eigen::Vector4d, Eigen::Quaterniond>> arc_path = arc.Generate6DPoints(p1, p2, p3, num);
        arc.point_generation = "";
        for (const auto &pose : arc_path)
        {
            Eigen::Matrix4d pose_transform;
            pose_transform.block<4, 1>(0, 3) = pose.first;
            pose_transform.block<3, 3>(0, 0) = pose.second.toRotationMatrix();
            pose_transform.block<1, 3>(3, 0).setZero();
            ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->pose_6_wrt_0 = pose_transform;
            T_e_wrt_b = pose_transform;
            ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->SolveIK(Conversions::Transform_2KDL(pose_transform));
            arm_ik.push_back(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->q);
        }
    }

    void PathPlanner::ChickenHeadPlanner(const Eigen::Matrix4d &goal, const vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_pose_array)
    {
        // arm_ik.clear();
        if (goal(3, 3) < 0.5)
            for (int i = 0; i < spot_pose_array.size(); i++)
                arm_ik.push_back(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->q_home);
        else
        {
            Eigen::Matrix4d T1wrtW, TEwrtB = goal, TEwrtW;
            T1wrtW.block<4, 1>(0, 3) = spot_pose_array[0].first;
            T1wrtW.block<3, 3>(0, 0) = CommonMathsSolver::OrientationNTransformaton::ComputeR(spot_pose_array[0].second);
            T1wrtW.block<1, 3>(3, 0).setZero();

            TEwrtW = T1wrtW * Kinematics::kinematics_solver->TBwrt1 * TEwrtB; // TEwrtW = constant

            for (const auto &spot_pose : spot_pose_array)
            {
                T1wrtW.block<4, 1>(0, 3) = spot_pose.first;
                T1wrtW.block<3, 3>(0, 0) = CommonMathsSolver::OrientationNTransformaton::ComputeR(spot_pose.second);
                T1wrtW.block<1, 3>(3, 0).setZero();

                TEwrtB = (T1wrtW * Kinematics::kinematics_solver->TBwrt1).inverse() * TEwrtW;
                T_e_wrt_b = TEwrtB;
                ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->pose_6_wrt_0 = TEwrtB;
                ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->SolveIK(Conversions::Transform_2KDL(TEwrtB));
                // ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]
                //     ->SolveFK(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->q); // q_home

                arm_ik.push_back(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->q);
            }
        }
    }

    void PathPlanner::Rough(int num)
    {
        arm_ik.clear();
        for (int i = 0; i < num; i++)
            arm_ik.push_back(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->q_home);
    }
}