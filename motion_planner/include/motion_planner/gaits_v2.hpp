#ifndef GAITS_V2_HPP
#define GAITS_V2_HPP

#include "maths/commonmathssolver.hpp"
#include "maths/curves.hpp"

#include "kinematics/kinematics.hpp"

// #include "motion_planner/posture.hpp"
#include "motion_planner/leg_motion_planning.hpp"

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

using namespace std;
using namespace PostureControl;

const double R2D = 57.295;

namespace Gaits
{
    enum class MotionStragegy
    {
        Spin,
        Straight
    };

    class Gaits
    {
    private:
        double t[4], yaw_theta;
        bool base_circle_status;
        std::unique_ptr<Curves::Arc> base_circle;

        vector<Eigen::Vector4d> GenerateFootCP_ForSpinnningGait(double ti, double tf, double stride_h);
        vector<Eigen::Vector4d> GenerateFootCP_ForTrotGait(double stride_s, double stride_h, int leg_index);
        vector<Eigen::Vector4d> GenerateFootCP_ForCrawlGait(double stride_s, double stride_h, int leg_index, string str);
        void ReplaceColumn(vector<vector<Eigen::Vector3d>> &v1, vector<vector<Eigen::Vector3d>> &v2, int col);
        void GenerateStanceLegPositions(const vector<int> &sequence, vector<vector<Eigen::Vector3d>> &trasition_ik, unique_ptr<Curves::Arc> &base_circle_);
        void HelpingFunForCrawlGait(const Eigen::Matrix4d &goal, vector<Eigen::Vector4d> &com_array, vector<vector<Eigen::Vector3d>> &ans,
                                    int swing_leg_index, int num, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array2);
        void SetStanceLegs(const vector<int> &sequence);
        void HelpingFunForTransitionPhase(const Eigen::Matrix4d &ee_goal, vector<Eigen::Vector4d> &com_array, vector<vector<Eigen::Vector3d>> &ans, const vector<int> &sequence, vector<vector<Eigen::Vector3d>> &trasition_ik, unique_ptr<Curves::Arc> &base_circle_,
                                          const MotionStragegy &strategy, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array, double step_theta, double stride_h, int num, int spin_n, int crawl_n, const string &str);

    public:
        unique_ptr<LegMotionPlanningNamspace::LegMotionPlanning> leg_planner_solver;
        Gaits(double angle_precision_, double displacement_precision_);
        vector<vector<Eigen::Vector3d>> CrawlSpinningGait(const Eigen::Matrix4d &ee_goal, vector<Eigen::Vector4d> &com_array, vector<KDL::JntArray> &arm_ik, double spinning_angle, double stride_h, double stride_t, int step_num, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array);
        vector<vector<Eigen::Vector3d>> ReverseCrawlSpinningGait(const Eigen::Matrix4d &ee_goal, vector<Eigen::Vector4d> &com_array, vector<KDL::JntArray> &arm_ik, double spinning_angle, double stride_h, double stride_t, int step_num, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array);
        vector<vector<Eigen::Vector3d>> CrawlGait(const Eigen::Matrix4d &ee_goal, vector<Eigen::Vector4d> &com_array, vector<KDL::JntArray> &arm_ik, double distance, double stride_h, int step_num, const string &str, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array);
        vector<vector<Eigen::Vector3d>> ReverseCrawlGait(const Eigen::Matrix4d &ee_goal, vector<Eigen::Vector4d> &com_array, vector<KDL::JntArray> &arm_ik, double distance, double stride_h, int step_num, const Eigen::Vector3d &direction, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array);
    };

    void Gaits::ReplaceColumn(vector<vector<Eigen::Vector3d>> &v1, vector<vector<Eigen::Vector3d>> &v2, int col)
    {
        int index = v1.size() - v2.size();
        for (int i = 0; i < v2.size(); ++i)
            v1[index + i][col] = v2[i][col];
    }

    Gaits::Gaits(double angle_precision_, double displacement_precision_)
    {
        base_circle_status = true;
        leg_planner_solver = make_unique<LegMotionPlanningNamspace::LegMotionPlanning>(angle_precision_, displacement_precision_);
    }

    void Gaits::SetStanceLegs(const vector<int> &sequence)
    {
        leg_planner_solver->leg_planning_param[sequence[0]].foot_control_points = {Kinematics::kinematics_solver->legs[sequence[0]].xyz};
        leg_planner_solver->leg_planning_param[sequence[1]].foot_control_points = {Kinematics::kinematics_solver->legs[sequence[1]].xyz};
        leg_planner_solver->leg_planning_param[sequence[2]].foot_control_points = {Kinematics::kinematics_solver->legs[sequence[2]].xyz};
    }

    void Gaits::HelpingFunForCrawlGait(const Eigen::Matrix4d &goal, vector<Eigen::Vector4d> &com_array, vector<vector<Eigen::Vector3d>> &ans,
                                       int swing_leg_index, int num, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array2)
    {
        leg_planner_solver->leg_planning_param[0].num_of_points = leg_planner_solver->leg_planning_param[1].num_of_points =
            leg_planner_solver->leg_planning_param[2].num_of_points = leg_planner_solver->leg_planning_param[3].num_of_points = num;
        leg_planner_solver->leg_planning_param[0].curve_type = leg_planner_solver->leg_planning_param[1].curve_type =
            leg_planner_solver->leg_planning_param[2].curve_type = leg_planner_solver->leg_planning_param[3].curve_type = Curves::CurveType::Bezier;
        leg_planner_solver->arm_ik_needs_to_computed = leg_planner_solver->com_needs_to_computed = false;
        vector<vector<Eigen::Vector3d>> crawl_ik = leg_planner_solver->GenerateLegsMotion(goal, com_array, Kinematics::kinematics_solver->T1wrt0, spot_com_pose_array2);
        ReplaceColumn(ans, crawl_ik, swing_leg_index);
    }

    void Gaits::GenerateStanceLegPositions(const vector<int> &sequence, vector<vector<Eigen::Vector3d>> &trasition_ik, unique_ptr<Curves::Arc> &base_circle_)
    {
        vector<Eigen::Vector4d> stance_legs_single_state(3);
        for (int i = 0; i < trasition_ik.size(); i++)
        {
            stance_legs_single_state[0] = Kinematics::kinematics_solver->T0wrtW * Kinematics::kinematics_solver->legs[sequence[0]].xyz;
            stance_legs_single_state[1] = Kinematics::kinematics_solver->T0wrtW * Kinematics::kinematics_solver->legs[sequence[1]].xyz;
            stance_legs_single_state[2] = Kinematics::kinematics_solver->T0wrtW * Kinematics::kinematics_solver->legs[sequence[2]].xyz;
            Kinematics::kinematics_solver->stance_legs_wrt_world.push_back(stance_legs_single_state);
        }
        Kinematics::kinematics_solver->legs[sequence[3]].xyz = leg_planner_solver->leg_planning_param[sequence[3]].foot_control_points[3];
        if (base_circle_status)
            for (int i = 0; i < 4; i++)
                t[i] = base_circle_->ComputeTheta(Kinematics::kinematics_solver->legs[i].xyz);
        trasition_ik.clear();
    }

    void Gaits::HelpingFunForTransitionPhase(const Eigen::Matrix4d &ee_goal, vector<Eigen::Vector4d> &com_array, vector<vector<Eigen::Vector3d>> &ans, const vector<int> &sequence, vector<vector<Eigen::Vector3d>> &trasition_ik, unique_ptr<Curves::Arc> &base_circle_,
                                             const MotionStragegy &strategy, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array, double step_theta, double stride_h, int num, int spin_n, int crawl_n, const string &str)
    {
        if (strategy == MotionStragegy::Spin)
            leg_planner_solver->leg_planning_param[sequence[3]].foot_control_points = GenerateFootCP_ForSpinnningGait(t[sequence[3]], t[sequence[3]] + (spin_n * (D2R * 2.0 * step_theta)), stride_h);
        else
            leg_planner_solver->leg_planning_param[sequence[3]].foot_control_points = GenerateFootCP_ForCrawlGait(crawl_n * step_theta, stride_h, sequence[3], str);

        SetStanceLegs(sequence);
        leg_planner_solver->leg_planning_param[0].num_of_points = leg_planner_solver->leg_planning_param[1].num_of_points = leg_planner_solver->leg_planning_param[2].num_of_points = leg_planner_solver->leg_planning_param[3].num_of_points = num;
        leg_planner_solver->leg_planning_param[0].curve_type = leg_planner_solver->leg_planning_param[1].curve_type = leg_planner_solver->leg_planning_param[2].curve_type = leg_planner_solver->leg_planning_param[3].curve_type = Curves::CurveType::Bezier;
        trasition_ik = leg_planner_solver->GenerateLegsMotion(ee_goal, com_array, Kinematics::kinematics_solver->T1wrt0, spot_com_pose_array);
        if (ans.empty())
            ans.insert(ans.begin(), trasition_ik.begin(), trasition_ik.end());
        else
            ans.insert(ans.end(), trasition_ik.begin(), trasition_ik.end());
        GenerateStanceLegPositions(sequence, trasition_ik, base_circle);
    }

    // 4,1,2,3
    vector<vector<Eigen::Vector3d>> Gaits::ReverseCrawlSpinningGait(const Eigen::Matrix4d &ee_goal, vector<Eigen::Vector4d> &com_array, vector<KDL::JntArray> &arm_ik, double spinning_angle, double stride_h, double stride_t, int step_num, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array)
    {
        leg_planner_solver->str = "true";
        base_circle_status = true;
        base_circle = std::make_unique<Curves::Arc>(Kinematics::kinematics_solver->legs[0].xyz,
                                                    Kinematics::kinematics_solver->legs[1].xyz,
                                                    Kinematics::kinematics_solver->legs[2].xyz, "C");
        base_circle->SetValue(Kinematics::state_tracker->body_rpy(2), Kinematics::kinematics_solver->legs[0].xyz(2));
        for (int i = 0; i < 4; i++)
            t[i] = base_circle->ComputeTheta(Kinematics::kinematics_solver->legs[i].xyz);
        double step_theta = spinning_angle / (4 * step_num);

        vector<int> seq = {0, 1, 2}, sequence(4);
        vector<vector<int>> seq_vector = {seq};
        vector<Eigen::Vector4d> l_foot_cp[4], stance_legs_single_state(3);
        vector<vector<Eigen::Vector3d>> ans, trasition_ik;

        double initial_yaw = R2D * Kinematics::state_tracker->body_rpy(2);
        Kinematics::kinematics_solver->stance_legs_wrt_world.clear();
        vector<pair<Eigen::Vector4d, Eigen::Vector3d>> spot_com_pose_array2, spot_com_pose_array_clear;

        sequence = {1, 2, 0, 3};
        HelpingFunForTransitionPhase(ee_goal, com_array, ans, sequence, trasition_ik, base_circle, MotionStragegy::Spin, spot_com_pose_array_clear, step_theta, stride_h, 100, -1, 0, "");
        spot_com_pose_array.insert(spot_com_pose_array.begin(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
        spot_com_pose_array_clear.clear();
        sequence = {1, 3, 0, 2};
        HelpingFunForTransitionPhase(ee_goal, com_array, ans, sequence, trasition_ik, base_circle, MotionStragegy::Spin, spot_com_pose_array_clear, step_theta, stride_h, 100, 1, 0, "");
        spot_com_pose_array.insert(spot_com_pose_array.begin(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
        spot_com_pose_array_clear.clear();
        arm_ik.insert(arm_ik.begin(), leg_planner_solver->arm_planner_solver->arm_ik.begin(), leg_planner_solver->arm_planner_solver->arm_ik.end());

        // pattern
        for (int i = 0; i < 4 * step_num; i++)
        {
            // compute only leg and arm state
            leg_planner_solver->com_needs_to_be_computed_posture = false;
            Eigen::Vector3d rpy(0, 0, initial_yaw + ((i + 1) * step_theta));
            vector<Eigen::Vector3d> rpy_vector = {rpy};
            vector<vector<Eigen::Vector3d>> yaw_ik = leg_planner_solver->PerformRPY(leg_planner_solver->arm_planner_solver->T_e_wrt_b, rpy_vector, seq_vector, com_array, spot_com_pose_array_clear, FootHold::DoNotReset);
            spot_com_pose_array.insert(spot_com_pose_array.end(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
            arm_ik.insert(arm_ik.end(), leg_planner_solver->arm_planner_solver->arm_ik.begin(), leg_planner_solver->arm_planner_solver->arm_ik.end());
            ans.insert(ans.end(), yaw_ik.begin(), yaw_ik.end());
            if (spinning_angle > 0)
            {
                if (i % 4 == 0)
                    sequence = {1, 2, 0, 3};
                else if (i % 4 == 1)
                    sequence = {1, 2, 3, 0};
                else if (i % 4 == 2)
                    sequence = {3, 2, 0, 1};
                else if (i % 4 == 3)
                    sequence = {1, 3, 0, 2};
                leg_planner_solver->leg_planning_param[sequence[3]].foot_control_points = GenerateFootCP_ForSpinnningGait(t[sequence[3]], t[sequence[3]] + (4 * D2R * step_theta), stride_h);
                SetStanceLegs(sequence);
                // modify only swing leg state
                HelpingFunForCrawlGait(leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array, ans, sequence[3], yaw_ik.size(), spot_com_pose_array_clear);
                GenerateStanceLegPositions(sequence, yaw_ik, base_circle);
                // compute com
                vector<vector<Eigen::Vector3d>> ans_com(ans.end() - spot_com_pose_array_clear.size(), ans.end());
                for (int index = 0; index < spot_com_pose_array_clear.size(); index++)
                    com_array.push_back(leg_planner_solver->com_solver->ComputeWholeSystemCOM(ans_com[index], spot_com_pose_array_clear[index], leg_planner_solver->arm_planner_solver->arm_ik[index]));
                spot_com_pose_array_clear.clear();
                leg_planner_solver->arm_ik_needs_to_computed = leg_planner_solver->com_needs_to_computed = true;
            }
            rpy_vector.clear();
        }
        leg_planner_solver->arm_planner_solver->arm_ik.clear();
        sequence = {0, 1, 2, 3};
        HelpingFunForTransitionPhase(leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array, ans, sequence, trasition_ik, base_circle, MotionStragegy::Spin, spot_com_pose_array_clear, step_theta, stride_h, 100, 1, 0, "");
        spot_com_pose_array.insert(spot_com_pose_array.end(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
        spot_com_pose_array_clear.clear();
        sequence = {0, 1, 3, 2};
        HelpingFunForTransitionPhase(leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array, ans, sequence, trasition_ik, base_circle, MotionStragegy::Spin, spot_com_pose_array_clear, step_theta, stride_h, 100, -1, 0, "");
        spot_com_pose_array.insert(spot_com_pose_array.end(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
        spot_com_pose_array_clear.clear();

        arm_ik.insert(arm_ik.end(), leg_planner_solver->arm_planner_solver->arm_ik.begin(), leg_planner_solver->arm_planner_solver->arm_ik.end());
        leg_planner_solver->arm_planner_solver->arm_ik.clear();
        return ans;
    }
    // 1,4,3,2
    vector<vector<Eigen::Vector3d>> Gaits::CrawlSpinningGait(const Eigen::Matrix4d &ee_goal, vector<Eigen::Vector4d> &com_array, vector<KDL::JntArray> &arm_ik, double spinning_angle, double stride_h, double stride_t, int step_num, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array)
    {
        leg_planner_solver->str = "true";
        base_circle_status = true;
        base_circle = std::make_unique<Curves::Arc>(Kinematics::kinematics_solver->legs[0].xyz,
                                                    Kinematics::kinematics_solver->legs[1].xyz,
                                                    Kinematics::kinematics_solver->legs[2].xyz, "C");
        base_circle->SetValue(Kinematics::state_tracker->body_rpy(2), Kinematics::kinematics_solver->legs[0].xyz(2));
        for (int i = 0; i < 4; i++)
            t[i] = base_circle->ComputeTheta(Kinematics::kinematics_solver->legs[i].xyz);
        double step_theta = spinning_angle / (4 * step_num);

        vector<int> seq = {0, 1, 2}, sequence(4);
        vector<vector<int>> seq_vector = {seq};
        vector<Eigen::Vector4d> l_foot_cp[4], stance_legs_single_state(3);
        vector<vector<Eigen::Vector3d>> ans, trasition_ik;

        double initial_yaw = R2D * Kinematics::state_tracker->body_rpy(2);
        Kinematics::kinematics_solver->stance_legs_wrt_world.clear();
        vector<pair<Eigen::Vector4d, Eigen::Vector3d>> spot_com_pose_array2, spot_com_pose_array_clear;

        sequence = {1, 2, 3, 0};
        HelpingFunForTransitionPhase(ee_goal, com_array, ans, sequence, trasition_ik, base_circle, MotionStragegy::Spin, spot_com_pose_array_clear, step_theta, stride_h, 100, -1, 0, "");
        spot_com_pose_array.insert(spot_com_pose_array.begin(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
        spot_com_pose_array_clear.clear();
        sequence = {0, 2, 3, 1};
        HelpingFunForTransitionPhase(ee_goal, com_array, ans, sequence, trasition_ik, base_circle, MotionStragegy::Spin, spot_com_pose_array_clear, step_theta, stride_h, 100, 1, 0, "");
        spot_com_pose_array.insert(spot_com_pose_array.begin(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
        spot_com_pose_array_clear.clear();
        arm_ik.insert(arm_ik.begin(), leg_planner_solver->arm_planner_solver->arm_ik.begin(), leg_planner_solver->arm_planner_solver->arm_ik.end());

        // pattern
        for (int i = 0; i < 4 * step_num; i++) // 4 * step_num
        {
            // compute only leg and arm state
            leg_planner_solver->com_needs_to_be_computed_posture = false;
            Eigen::Vector3d rpy(0, 0, initial_yaw + ((i + 1) * step_theta));
            vector<Eigen::Vector3d> rpy_vector = {rpy};
            vector<vector<Eigen::Vector3d>> yaw_ik = leg_planner_solver->PerformRPY(leg_planner_solver->arm_planner_solver->T_e_wrt_b, rpy_vector, seq_vector, com_array, spot_com_pose_array_clear, FootHold::DoNotReset);
            spot_com_pose_array.insert(spot_com_pose_array.end(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
            arm_ik.insert(arm_ik.end(), leg_planner_solver->arm_planner_solver->arm_ik.begin(), leg_planner_solver->arm_planner_solver->arm_ik.end());
            ans.insert(ans.end(), yaw_ik.begin(), yaw_ik.end());
            if (spinning_angle < 0)
            {
                if (i % 4 == 0)
                    sequence = {1, 2, 3, 0};
                else if (i % 4 == 1)
                    sequence = {1, 2, 0, 3};
                else if (i % 4 == 2)
                    sequence = {1, 0, 3, 2};
                else if (i % 4 == 3)
                    sequence = {0, 2, 3, 1};
                leg_planner_solver->leg_planning_param[sequence[3]].foot_control_points = GenerateFootCP_ForSpinnningGait(t[sequence[3]], t[sequence[3]] + (4 * D2R * step_theta), stride_h);
                SetStanceLegs(sequence);
                // modify only swing leg state
                HelpingFunForCrawlGait(leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array, ans, sequence[3], yaw_ik.size(), spot_com_pose_array_clear);
                GenerateStanceLegPositions(sequence, yaw_ik, base_circle);
                // compute com
                vector<vector<Eigen::Vector3d>> ans_com(ans.end() - spot_com_pose_array_clear.size(), ans.end());
                for (int index = 0; index < spot_com_pose_array_clear.size(); index++)
                    com_array.push_back(leg_planner_solver->com_solver->ComputeWholeSystemCOM(ans_com[index], spot_com_pose_array_clear[index], leg_planner_solver->arm_planner_solver->arm_ik[index]));
                spot_com_pose_array_clear.clear();
                leg_planner_solver->arm_ik_needs_to_computed = leg_planner_solver->com_needs_to_computed = true;
            }
            rpy_vector.clear();
        }
        leg_planner_solver->arm_planner_solver->arm_ik.clear();
        sequence = {1, 2, 3, 0};
        HelpingFunForTransitionPhase(leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array, ans, sequence, trasition_ik, base_circle, MotionStragegy::Spin, spot_com_pose_array_clear, step_theta, stride_h, 100, 1, 0, "");
        spot_com_pose_array.insert(spot_com_pose_array.end(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
        spot_com_pose_array_clear.clear();
        sequence = {0, 2, 3, 1};
        HelpingFunForTransitionPhase(leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array, ans, sequence, trasition_ik, base_circle, MotionStragegy::Spin, spot_com_pose_array_clear, step_theta, stride_h, 100, -1, 0, "");
        spot_com_pose_array.insert(spot_com_pose_array.end(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
        spot_com_pose_array_clear.clear();
        arm_ik.insert(arm_ik.end(), leg_planner_solver->arm_planner_solver->arm_ik.begin(), leg_planner_solver->arm_planner_solver->arm_ik.end());
        leg_planner_solver->arm_planner_solver->arm_ik.clear();
        return ans;
    }

    vector<Eigen::Vector4d> Gaits::GenerateFootCP_ForSpinnningGait(double ti, double tf, double stride_h)
    {
        Eigen::Vector4d p1 = base_circle->GenerateSinglePoint(ti);
        Eigen::Vector4d p4 = base_circle->GenerateSinglePoint(tf);
        Eigen::Vector4d p2(p1(0), p1(1), (p1(2) + stride_h), 1.0);
        Eigen::Vector4d p3(p4(0), p4(1), (p4(2) + stride_h), 1.0);
        vector<Eigen::Vector4d> ans = {p1, p2, p3, p4};
        return ans;
    }

    vector<Eigen::Vector4d> Gaits::GenerateFootCP_ForTrotGait(double stride_s, double stride_h, int leg_index)
    {
        double theta = Kinematics::state_tracker->body_rpy(2);
        Eigen::Vector3d r_cap(cos(theta), sin(theta), 0.0);

        Eigen::Vector4d p1 = Kinematics::kinematics_solver->legs[leg_index].xyz;
        Eigen::Vector4d p2(p1(0), p1(1), p1(2) + stride_h, 1.0);
        Eigen::Vector4d p3(p1(0) + (stride_s * r_cap(0)), p1(1) + (stride_s * r_cap(1)), p2(2), 1.0);
        Eigen::Vector4d p4(p3(0), p3(1), p1(2), 1.0);
        vector<Eigen::Vector4d> ans = {p1, p2, p3, p4};
        return ans;
    }

    vector<Eigen::Vector4d> Gaits::GenerateFootCP_ForCrawlGait(double stride_s, double stride_h, int leg_index, string str)
    {
        double theta = Kinematics::state_tracker->body_rpy(2);
        Eigen::Vector3d r_cap;
        if (str == "SP")
            r_cap << cos(theta + (PI / 2)), sin(theta + (PI / 2)), 0.0;
        else if (str == "SN")
            r_cap << cos(theta - (PI / 2)), sin(theta - (PI / 2)), 0.0;
        else
            r_cap << cos(theta), sin(theta), 0.0;
        Eigen::Vector4d p1 = Kinematics::kinematics_solver->legs[leg_index].xyz;
        Eigen::Vector4d p2(p1(0), p1(1), p1(2) + stride_h, 1.0);
        Eigen::Vector4d p3(p1(0) + (stride_s * r_cap(0)), p1(1) + (stride_s * r_cap(1)), p2(2), 1.0);
        Eigen::Vector4d p4(p3(0), p3(1), p1(2), 1.0);

        vector<Eigen::Vector4d> ans = {p1, p2, p3, p4};
        return ans;
    }

    // 0,2,3,1
    vector<vector<Eigen::Vector3d>> Gaits::CrawlGait(const Eigen::Matrix4d &ee_goal, vector<Eigen::Vector4d> &com_array, vector<KDL::JntArray> &arm_ik, double distance, double stride_h, int step_num, const string &str, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array)
    {
        base_circle_status = false;
        vector<vector<Eigen::Vector3d>> ans, ans2, trasition_ik;
        vector<Eigen::Vector4d> l_foot_cp[4];
        vector<int> sequence(4);
        double stride_s = distance / (4 * step_num);
        Kinematics::kinematics_solver->stance_legs_wrt_world.clear();
        vector<pair<Eigen::Vector4d, Eigen::Vector3d>> spot_com_pose_array2, spot_com_pose_array_clear;

        sequence = {1, 2, 3, 0};
        HelpingFunForTransitionPhase(ee_goal, com_array, ans, sequence, trasition_ik, base_circle, MotionStragegy::Straight, spot_com_pose_array_clear, stride_s, stride_h, 50, 0, -2, str);
        spot_com_pose_array.insert(spot_com_pose_array.begin(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
        spot_com_pose_array_clear.clear();
        sequence = {0, 2, 3, 1};
        HelpingFunForTransitionPhase(ee_goal, com_array, ans, sequence, trasition_ik, base_circle, MotionStragegy::Straight, spot_com_pose_array_clear, stride_s, stride_h, 50, 0, 2, str);
        spot_com_pose_array.insert(spot_com_pose_array.end(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
        spot_com_pose_array_clear.clear();
        arm_ik.insert(arm_ik.begin(), leg_planner_solver->arm_planner_solver->arm_ik.begin(), leg_planner_solver->arm_planner_solver->arm_ik.end());

        double theta = Kinematics::state_tracker->body_rpy(2);
        Eigen::Vector3d r_cap(cos(theta), sin(theta), 0.0);
        double x, y;
        if (str == "SP")
        {
            x = (stride_s * cos(theta + (PI / 2)));
            y = (stride_s * sin(theta + (PI / 2)));
        }
        else if (str == "SN")
        {
            x = (stride_s * cos(theta - (PI / 2)));
            y = (stride_s * sin(theta - (PI / 2)));
        }
        else
        {
            x = (stride_s * cos(theta));
            y = (stride_s * sin(theta));
        }
        Eigen::Vector4d rpv(x, y, 0.0, 1.0);
        vector<Eigen::Vector4d> rpv_vector = {rpv};

        for (int i = 0; i < (4 * step_num); i++) //(4 * step_num)
        {
            // compute only leg and arm state
            leg_planner_solver->com_needs_to_be_computed_posture = false;
            vector<vector<Eigen::Vector3d>> crawl_ik = leg_planner_solver->PerformTranslation(leg_planner_solver->arm_planner_solver->T_e_wrt_b, rpv_vector, com_array, spot_com_pose_array_clear, FootHold::DoNotReset);
            spot_com_pose_array.insert(spot_com_pose_array.end(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
            arm_ik.insert(arm_ik.end(), leg_planner_solver->arm_planner_solver->arm_ik.begin(), leg_planner_solver->arm_planner_solver->arm_ik.end());

            ans.insert(ans.end(), crawl_ik.begin(), crawl_ik.end());
            if (i % 4 == 0)
                sequence = {1, 2, 3, 0};
            else if (i % 4 == 1)
                sequence = {0, 1, 3, 2};
            else if (i % 4 == 2)
                sequence = {0, 1, 2, 3};
            else if (i % 4 == 3)
                sequence = {0, 2, 3, 1};
            leg_planner_solver->leg_planning_param[sequence[3]].foot_control_points = Gaits::GenerateFootCP_ForCrawlGait(4 * stride_s, stride_h, sequence[3], str);
            SetStanceLegs(sequence);
            // modify only swing leg state
            HelpingFunForCrawlGait(leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array, ans, sequence[3], crawl_ik.size(), spot_com_pose_array_clear);
            GenerateStanceLegPositions(sequence, crawl_ik, base_circle);
            // compute com
            vector<vector<Eigen::Vector3d>> ans_com(ans.end() - spot_com_pose_array_clear.size(), ans.end());
            for (int index = 0; index < spot_com_pose_array_clear.size(); index++)
                com_array.push_back(leg_planner_solver->com_solver->ComputeWholeSystemCOM(ans_com[index], spot_com_pose_array_clear[index], leg_planner_solver->arm_planner_solver->arm_ik[index]));
            spot_com_pose_array_clear.clear();
            leg_planner_solver->arm_ik_needs_to_computed = leg_planner_solver->com_needs_to_computed = true;
        }
        leg_planner_solver->arm_planner_solver->arm_ik.clear();
        sequence = {1, 2, 3, 0};
        HelpingFunForTransitionPhase(leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array, ans, sequence, trasition_ik, base_circle, MotionStragegy::Straight, spot_com_pose_array_clear, stride_s, stride_h, 50, 0, 2, str);
        spot_com_pose_array.insert(spot_com_pose_array.end(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
        spot_com_pose_array_clear.clear();
        sequence = {0, 2, 3, 1};
        HelpingFunForTransitionPhase(leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array, ans, sequence, trasition_ik, base_circle, MotionStragegy::Straight, spot_com_pose_array_clear, stride_s, stride_h, 50, 0, -2, str);
        spot_com_pose_array.insert(spot_com_pose_array.end(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
        spot_com_pose_array_clear.clear();

        arm_ik.insert(arm_ik.end(), leg_planner_solver->arm_planner_solver->arm_ik.begin(), leg_planner_solver->arm_planner_solver->arm_ik.end());
        leg_planner_solver->arm_planner_solver->arm_ik.clear();
        return ans;
    }

    // 0,2,3,1
    vector<vector<Eigen::Vector3d>> Gaits::ReverseCrawlGait(const Eigen::Matrix4d &ee_goal, vector<Eigen::Vector4d> &com_array, vector<KDL::JntArray> &arm_ik, double distance, double stride_h, int step_num, const Eigen::Vector3d &direction, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array)
    {
        cout << "reverse" << endl;
        base_circle_status = false;
        vector<vector<Eigen::Vector3d>> ans, trasition_ik;
        vector<Eigen::Vector4d> l_foot_cp[4];
        vector<int> sequence(4);
        double stride_s = distance / (4 * step_num);
        Kinematics::kinematics_solver->stance_legs_wrt_world.clear();
        vector<pair<Eigen::Vector4d, Eigen::Vector3d>> spot_com_pose_array2, spot_com_pose_array_clear;

        sequence = {1, 2, 3, 0};
        HelpingFunForTransitionPhase(ee_goal, com_array, ans, sequence, trasition_ik, base_circle, MotionStragegy::Straight, spot_com_pose_array_clear, stride_s, stride_h, 50, 0, -2, "");
        spot_com_pose_array.insert(spot_com_pose_array.begin(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
        spot_com_pose_array_clear.clear();
        sequence = {0, 2, 3, 1};
        HelpingFunForTransitionPhase(ee_goal, com_array, ans, sequence, trasition_ik, base_circle, MotionStragegy::Straight, spot_com_pose_array_clear, stride_s, stride_h, 50, 0, 2, "");
        spot_com_pose_array.insert(spot_com_pose_array.begin(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
        spot_com_pose_array_clear.clear();
        arm_ik.insert(arm_ik.begin(), leg_planner_solver->arm_planner_solver->arm_ik.begin(), leg_planner_solver->arm_planner_solver->arm_ik.end());
        cout << "10" << endl;
        double theta = Kinematics::state_tracker->body_rpy(2);
        Eigen::Vector3d r_cap(cos(theta), sin(theta), 0.0);
        double x = (-stride_s * cos(theta));
        double y = (-stride_s * sin(theta));
        Eigen::Vector4d rpv(x, y, 0.0, 1.0);
        vector<Eigen::Vector4d> rpv_vector = {rpv};

        for (int i = 0; i < (4 * step_num); i++)
        {
            cout << "nnn" << endl;
            // compute only leg and arm state
            leg_planner_solver->com_needs_to_be_computed_posture = false;
            vector<vector<Eigen::Vector3d>> crawl_ik = leg_planner_solver->PerformTranslation(leg_planner_solver->arm_planner_solver->T_e_wrt_b, rpv_vector, com_array, spot_com_pose_array_clear, FootHold::DoNotReset);
            spot_com_pose_array.insert(spot_com_pose_array.end(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
            arm_ik.insert(arm_ik.end(), leg_planner_solver->arm_planner_solver->arm_ik.begin(), leg_planner_solver->arm_planner_solver->arm_ik.end());

            ans.insert(ans.end(), crawl_ik.begin(), crawl_ik.end());
            if (i % 4 == 0)
                sequence = {0, 2, 3, 1};
            else if (i % 4 == 1)
                sequence = {2, 1, 0, 3};
            else if (i % 4 == 2)
                sequence = {3, 1, 0, 2};
            else if (i % 4 == 3)
                sequence = {1, 2, 3, 0};
            leg_planner_solver->leg_planning_param[sequence[3]].foot_control_points = Gaits::GenerateFootCP_ForTrotGait(-4 * stride_s, stride_h, sequence[3]);
            SetStanceLegs(sequence);
            // modify only swing leg state
            HelpingFunForCrawlGait(leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array, ans, sequence[3], crawl_ik.size(), spot_com_pose_array_clear);
            GenerateStanceLegPositions(sequence, crawl_ik, base_circle);
            // compute com
            vector<vector<Eigen::Vector3d>> ans_com(ans.end() - spot_com_pose_array_clear.size(), ans.end());
            for (int index = 0; index < spot_com_pose_array_clear.size(); index++)
                com_array.push_back(leg_planner_solver->com_solver->ComputeWholeSystemCOM(ans_com[index], spot_com_pose_array_clear[index], leg_planner_solver->arm_planner_solver->arm_ik[index]));
            spot_com_pose_array_clear.clear();
            leg_planner_solver->arm_ik_needs_to_computed = leg_planner_solver->com_needs_to_computed = true;
        }
        cout << "10" << endl;
        leg_planner_solver->arm_planner_solver->arm_ik.clear();
        sequence = {1, 2, 3, 0};
        HelpingFunForTransitionPhase(leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array, ans, sequence, trasition_ik, base_circle, MotionStragegy::Straight, spot_com_pose_array_clear, stride_s, stride_h, 50, 0, 2, "");
        spot_com_pose_array.insert(spot_com_pose_array.end(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
        spot_com_pose_array_clear.clear();
        sequence = {0, 2, 3, 1};
        HelpingFunForTransitionPhase(leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array, ans, sequence, trasition_ik, base_circle, MotionStragegy::Straight, spot_com_pose_array_clear, stride_s, stride_h, 50, 0, -2, "");
        spot_com_pose_array.insert(spot_com_pose_array.end(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
        spot_com_pose_array_clear.clear();
        arm_ik.insert(arm_ik.end(), leg_planner_solver->arm_planner_solver->arm_ik.begin(), leg_planner_solver->arm_planner_solver->arm_ik.end());
        cout << "10" << endl;
        return ans;
    }
}

#endif
