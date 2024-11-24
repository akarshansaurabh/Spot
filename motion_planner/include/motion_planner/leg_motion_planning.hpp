#ifndef LEG_MOTION_PLANNING_HPP
#define LEG_MOTION_PLANNING_HPP

#include <eigen3/Eigen/Dense>

#include "maths/commonmathssolver.hpp"
#include "maths/curves.hpp"

#include "motion_planner/posture_v2.hpp"

#include "kinematics/kinematics.hpp"

#include <iostream>

using namespace std;
using namespace Curves;

namespace LegMotionPlanningNamspace
{
    struct LegMotionPlanningParam
    {
        vector<Eigen::Vector4d> foot_control_points; // 0 to A
        Curves::CurveType curve_type;
        int num_of_points;
    };

    class LegMotionPlanning : public PostureControl::PostureControl
    {
    private:
        void GenerateIthLegMotion(const LegMotionPlanningParam &planning_param, const Eigen::Matrix4d &T1wrt0, int leg_index);
        void GenerateIthLegMotion(const LegMotionPlanningParam &planning_param, const vector<Eigen::Matrix4d> &T1wrt0, int leg_index);

    public:
        vector<LegMotionPlanningParam> leg_planning_param;
        LegMotionPlanning(double angle_precision_, double displacement_precision_);
        vector<vector<Eigen::Vector3d>> GenerateLegsMotion(const Eigen::Matrix4d &goal, vector<Eigen::Vector4d> &com_array, const Eigen::Matrix4d &T1wrt0, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array);
        vector<vector<Eigen::Vector3d>> GenerateLegsMotion(const Eigen::Matrix4d &goal, vector<Eigen::Vector4d> &com_array, const vector<Eigen::Matrix4d> &T1wrt0, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array);
        bool body_pose_needs_to_computed;
    };

    LegMotionPlanning::LegMotionPlanning(double angle_precision_, double displacement_precision_)
        : PostureControl::PostureControl(angle_precision_, displacement_precision_)
    {
        leg_planning_param.resize(4);
        body_pose_needs_to_computed = true;
    }

    void LegMotionPlanning::GenerateIthLegMotion(const LegMotionPlanningParam &planning_param, const Eigen::Matrix4d &T1wrt0, int leg_index)
    {
        vector<Eigen::Vector4d> control_points_wrt_1;
        int num = planning_param.num_of_points;
        // wrt 1
        for (int i = 0; i < planning_param.foot_control_points.size(); i++)
            control_points_wrt_1.push_back(T1wrt0.inverse() * planning_param.foot_control_points[i]);

        if (planning_param.foot_control_points.size() == 1)
            for (int i = 0; i < num; i++)
                if (leg_index == 0)
                    l1_path_wrt_o.push_back(control_points_wrt_1[0]);
                else if (leg_index == 1)
                    l2_path_wrt_o.push_back(control_points_wrt_1[0]);
                else if (leg_index == 2)
                    l3_path_wrt_o.push_back(control_points_wrt_1[0]);
                else
                    l4_path_wrt_o.push_back(control_points_wrt_1[0]);
        else
        {
            if (planning_param.curve_type == Curves::CurveType::Bezier)
            {
                Curves::Bezzier bezier_curve(control_points_wrt_1);
                if (leg_index == 0)
                    l1_path_wrt_o = bezier_curve.Generate3DPoints(num);
                else if (leg_index == 1)
                    l2_path_wrt_o = bezier_curve.Generate3DPoints(num);
                else if (leg_index == 2)
                    l3_path_wrt_o = bezier_curve.Generate3DPoints(num);
                else
                    l4_path_wrt_o = bezier_curve.Generate3DPoints(num);
            }
            else if (planning_param.curve_type == Curves::CurveType::B_Spline)
            {
                Curves::BSpline bspline_curve(control_points_wrt_1, (control_points_wrt_1.size() == 4) ? 3 : 4);
                if (leg_index == 0)
                    l1_path_wrt_o = bspline_curve.Generate3DPoints(num);
                else if (leg_index == 1)
                    l2_path_wrt_o = bspline_curve.Generate3DPoints(num);
                else if (leg_index == 2)
                    l3_path_wrt_o = bspline_curve.Generate3DPoints(num);
                else
                    l4_path_wrt_o = bspline_curve.Generate3DPoints(num);
            }
        }
        control_points_wrt_1.clear();
    }

    vector<vector<Eigen::Vector3d>> LegMotionPlanning::GenerateLegsMotion(const Eigen::Matrix4d &goal, vector<Eigen::Vector4d> &com_array, const Eigen::Matrix4d &T1wrt0, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array)
    {
        l1_path_wrt_o.clear();
        l2_path_wrt_o.clear();
        l3_path_wrt_o.clear();
        l4_path_wrt_o.clear();

        GenerateIthLegMotion(leg_planning_param[0], T1wrt0, 0);
        GenerateIthLegMotion(leg_planning_param[1], T1wrt0, 1);
        GenerateIthLegMotion(leg_planning_param[2], T1wrt0, 2);
        GenerateIthLegMotion(leg_planning_param[3], T1wrt0, 3);

        int num_of_points = l1_path_wrt_o.size();
        vector<vector<Eigen::Vector3d>> ans;
        for (int i = 0; i < num_of_points; i++)
        {
            if (body_pose_needs_to_computed)
                spot_com_pose_array.push_back(make_pair(Kinematics::state_tracker->body_xyz, Kinematics::state_tracker->body_rpy));
            ans.push_back(Kinematics::kinematics_solver->SolveIK(l1_path_wrt_o[i], l2_path_wrt_o[i], l3_path_wrt_o[i], l4_path_wrt_o[i]));
        }

        if (armik_needs_to_be_computed_posture)
            arm_planner_solver->ChickenHeadPlanner(goal, spot_com_pose_array);
        if (com_needs_to_be_computed_posture)
            for (int index = 0; index < spot_com_pose_array.size(); index++)
                com_array.push_back(com_solver->ComputeWholeSystemCOM(ans[index], spot_com_pose_array[index], arm_planner_solver->arm_ik[index]));

        return ans;
    }

    void LegMotionPlanning::GenerateIthLegMotion(const LegMotionPlanningParam &planning_param, const vector<Eigen::Matrix4d> &T1wrt0, int leg_index)
    {
        vector<Eigen::Vector4d> path_wrt_0;
        int num = planning_param.num_of_points;

        if (planning_param.foot_control_points.size() == 1)
            for (int i = 0; i < num; i++)
                if (leg_index == 0)
                    l1_path_wrt_o.push_back(T1wrt0[i].inverse() * planning_param.foot_control_points[0]);
                else if (leg_index == 1)
                    l2_path_wrt_o.push_back(T1wrt0[i].inverse() * planning_param.foot_control_points[0]);
                else if (leg_index == 2)
                    l3_path_wrt_o.push_back(T1wrt0[i].inverse() * planning_param.foot_control_points[0]);
                else
                    l4_path_wrt_o.push_back(T1wrt0[i].inverse() * planning_param.foot_control_points[0]);
        else
        {
            if (planning_param.curve_type == Curves::CurveType::Bezier)
            {
                Curves::Bezzier bezier_curve(planning_param.foot_control_points);
                path_wrt_0 = bezier_curve.Generate3DPoints(num);

                if (leg_index == 0)
                    for (int i = 0; i < num; i++)
                        l1_path_wrt_o.push_back(T1wrt0[i].inverse() * path_wrt_0[i]);
                else if (leg_index == 1)
                    for (int i = 0; i < num; i++)
                        l2_path_wrt_o.push_back(T1wrt0[i].inverse() * path_wrt_0[i]);
                else if (leg_index == 2)
                    for (int i = 0; i < num; i++)
                        l3_path_wrt_o.push_back(T1wrt0[i].inverse() * path_wrt_0[i]);
                else
                    for (int i = 0; i < num; i++)
                        l4_path_wrt_o.push_back(T1wrt0[i].inverse() * path_wrt_0[i]);
            }
            else if (planning_param.curve_type == Curves::CurveType::B_Spline)
            {
                Curves::BSpline bspline_curve(planning_param.foot_control_points, (planning_param.foot_control_points.size() == 4) ? 3 : 4);
                path_wrt_0 = bspline_curve.Generate3DPoints(num);
                if (leg_index == 1)
                {
                    // cout << "2222222222222222222222222222222222222222222222222222222222222" << endl;
                    // for (auto p : path_wrt_0)
                    //     cout << p.adjoint() << endl;
                    
                }
              
                if (leg_index == 0)
                    for (int i = 0; i < num; i++)
                        l1_path_wrt_o.push_back(T1wrt0[i].inverse() * path_wrt_0[i]);
                else if (leg_index == 1)
                    for (int i = 0; i < num; i++)
                        l2_path_wrt_o.push_back(T1wrt0[i].inverse() * path_wrt_0[i]);
                else if (leg_index == 2)
                    for (int i = 0; i < num; i++)
                        l3_path_wrt_o.push_back(T1wrt0[i].inverse() * path_wrt_0[i]);
                else
                    for (int i = 0; i < num; i++)
                        l4_path_wrt_o.push_back(T1wrt0[i].inverse() * path_wrt_0[i]);
            }
        }
        // control_points_wrt_1.clear();
    }

    vector<vector<Eigen::Vector3d>> LegMotionPlanning::GenerateLegsMotion(const Eigen::Matrix4d &goal, vector<Eigen::Vector4d> &com_array, const vector<Eigen::Matrix4d> &T1wrt0, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array)
    {
        l1_path_wrt_o.clear();
        l2_path_wrt_o.clear();
        l3_path_wrt_o.clear();
        l4_path_wrt_o.clear();

        GenerateIthLegMotion(leg_planning_param[0], T1wrt0, 0);
        GenerateIthLegMotion(leg_planning_param[1], T1wrt0, 1);
        GenerateIthLegMotion(leg_planning_param[2], T1wrt0, 2);
        GenerateIthLegMotion(leg_planning_param[3], T1wrt0, 3);

        int num_of_points = l1_path_wrt_o.size();
        vector<vector<Eigen::Vector3d>> ans;
        for (int i = 0; i < num_of_points; i++)
        {
            if (body_pose_needs_to_computed)
                spot_com_pose_array.push_back(make_pair(Kinematics::state_tracker->body_xyz, Kinematics::state_tracker->body_rpy));
            ans.push_back(Kinematics::kinematics_solver->SolveIK(l1_path_wrt_o[i], l2_path_wrt_o[i], l3_path_wrt_o[i], l4_path_wrt_o[i]));
        }

        if (armik_needs_to_be_computed_posture)
            arm_planner_solver->ChickenHeadPlanner(goal, spot_com_pose_array);
        if (com_needs_to_be_computed_posture)
            for (int index = 0; index < spot_com_pose_array.size(); index++)
                com_array.push_back(com_solver->ComputeWholeSystemCOM(ans[index], spot_com_pose_array[index], arm_planner_solver->arm_ik[index]));

        return ans;
    }
}

#endif