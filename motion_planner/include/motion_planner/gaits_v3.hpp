#ifndef GAITS_V3_HPP
#define GAITS_V3_HPP

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

    class CrawlMaths
    {
    private:
        vector<int> leg_sequence;

    public:
        CrawlMaths();
        vector<int> BestSupportPolygon(const Eigen::Vector4d &com_projection);
    };

    CrawlMaths::CrawlMaths()
    {
        leg_sequence.resize(4);
    }
    vector<int> CrawlMaths::BestSupportPolygon(const Eigen::Vector4d &com_projection)
    {
        vector<int> leg_index_for_best_st(3);
        vector<pair<double, vector<int>>> sm_st_pair_array;
        int j, k;
        for (int i = 0; i < 4; i++)
        {
            if (CommonMathsSolver::Geometry::PointIsInsideTriangle(Kinematics::kinematics_solver->legs[i].xyz,
                                                                   Kinematics::kinematics_solver->legs[(i + 1 > 3) ? (i + 1 - 4) : (i + 1)].xyz,
                                                                   Kinematics::kinematics_solver->legs[(i + 2 > 3) ? (i + 2 - 4) : (i + 2)].xyz,
                                                                   com_projection))
            {
                j = (i + 1 > 3) ? (i + 1 - 4) : (i + 1);
                k = (i + 2 > 3) ? (i + 2 - 4) : (i + 2);
                CommonMathsSolver::Geometry::Line l1(Kinematics::kinematics_solver->legs[i].xyz, Kinematics::kinematics_solver->legs[j].xyz);
                CommonMathsSolver::Geometry::Line l2(Kinematics::kinematics_solver->legs[j].xyz, Kinematics::kinematics_solver->legs[k].xyz);
                CommonMathsSolver::Geometry::Line l3(Kinematics::kinematics_solver->legs[k].xyz, Kinematics::kinematics_solver->legs[i].xyz);

                double p1 = CommonMathsSolver::Geometry::DistanceFromLine(l1, com_projection);
                double p2 = CommonMathsSolver::Geometry::DistanceFromLine(l2, com_projection);
                double p3 = CommonMathsSolver::Geometry::DistanceFromLine(l3, com_projection);
                cout << "p1 " << p1 << " p2 " << p2 << " p3 " << p3 << endl;
                cout << i << " " << j << " " << k << endl;
                sm_st_pair_array.push_back(make_pair(CommonMathsSolver::MaxMin::FindMin(vector<double>{p1, p2, p3}), vector<int>{i, j, k}));
            }
        }
        cout << "possible support triangle num = " << sm_st_pair_array.size() << endl;
        if (sm_st_pair_array.size() == 1)
            leg_index_for_best_st = sm_st_pair_array[0].second;
        else if (sm_st_pair_array.size() == 2)
            leg_index_for_best_st = (sm_st_pair_array[0].first >= sm_st_pair_array[1].first)
                                        ? sm_st_pair_array[0].second
                                        : sm_st_pair_array[1].second;
        for (const auto &j : leg_index_for_best_st)
            cout << j << " ";
        cout << endl;
        return leg_index_for_best_st;
    }

    class Gaits
    {
    private:
        double t[4], yaw_theta;
        bool base_circle_status;
        std::unique_ptr<Curves::Arc> base_circle;
        CrawlMaths crawl_math_solver;
        vector<Eigen::Vector4d> GenerateFootCP_ForSpinnningGait(double ti, double tf, double stride_h);
        vector<Eigen::Vector4d> GenerateFootCP_ForTrotGait(double stride_s, double stride_h, int leg_index);
        void GenerateStanceLegPositions(const vector<int> &sequence, vector<vector<Eigen::Vector3d>> &trasition_ik, unique_ptr<Curves::Arc> &base_circle_);
        void HelpingFunForCrawlGait(const Eigen::Matrix4d &goal, vector<Eigen::Vector4d> &com_array, vector<vector<Eigen::Vector3d>> &ans,
                                    int swing_leg_index, int num, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array2);
        void HelpingFunForTransitionPhase(const Eigen::Matrix4d &ee_goal, vector<Eigen::Vector4d> &com_array, vector<vector<Eigen::Vector3d>> &ans, const vector<int> &sequence, vector<vector<Eigen::Vector3d>> &trasition_ik, unique_ptr<Curves::Arc> &base_circle_,
                                          const MotionStragegy &strategy, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array, double step_theta, double stride_h, int num, int spin_n, int crawl_n, const string &str);
        void HelpingFunForDancingCrawl(const Eigen::Matrix4d &goal, vector<Eigen::Vector4d> &com_array, vector<vector<Eigen::Vector3d>> &ans, int swing_leg_index,
                                       int num, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array2, const vector<Eigen::Matrix4d> &t1wrt0);
        void ComputeCurrentStateCOM();

    public:
        Eigen::Vector4d current_state_com;
        unique_ptr<LegMotionPlanningNamspace::LegMotionPlanning> leg_planner_solver;
        Gaits(double angle_precision_, double displacement_precision_);
        void SetStanceLegs(const vector<int> &sequence);
        void ReplaceColumn(vector<vector<Eigen::Vector3d>> &v1, vector<vector<Eigen::Vector3d>> &v2, int col);
        vector<Eigen::Vector4d> GenerateFootCP_ForCrawlGait(double stride_s, double stride_h, int leg_index, string str);
        vector<vector<Eigen::Vector3d>> CrawlSpinningGait(const Eigen::Matrix4d &ee_goal, vector<Eigen::Vector4d> &com_array, vector<KDL::JntArray> &arm_ik, double spinning_angle, double stride_h, double stride_t, int step_num, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array);
        vector<vector<Eigen::Vector3d>> ReverseCrawlSpinningGait(const Eigen::Matrix4d &ee_goal, vector<Eigen::Vector4d> &com_array, vector<KDL::JntArray> &arm_ik, double spinning_angle, double stride_h, double stride_t, int step_num, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array);
        vector<vector<Eigen::Vector3d>> CrawlGait(const Eigen::Matrix4d &ee_goal, vector<Eigen::Vector4d> &com_array, vector<KDL::JntArray> &arm_ik, double distance, double stride_h, int step_num, const string &str, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array);
        vector<vector<Eigen::Vector3d>> ReverseCrawlGait(const Eigen::Matrix4d &ee_goal, vector<Eigen::Vector4d> &com_array, vector<KDL::JntArray> &arm_ik, double distance, double stride_h, int step_num, const Eigen::Vector3d &direction, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array);

        vector<vector<Eigen::Vector3d>> DancingCrawlGait(const Eigen::Matrix4d &ee_goal, vector<Eigen::Vector4d> &com_array, vector<KDL::JntArray> &arm_ik, double distance, double stride_h, int step_num, double angle, const string &str, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array);
    };

    class ComplexPostures
    {
    private:
        vector<Eigen::Vector4d> GenerateFootCP_ForBowDown(double stride_s, double stride_h, double y_shift, int leg_index);
        vector<Eigen::Vector4d> GenerateFootCP_ForDancing(double x_max, double y_max, double z_max, double z_f, int leg_index);
        vector<Eigen::Vector4d> GenerateFootCP_ForSpotDancing(double backward_l, double radius, int leg_index);
        void HelpingFunForBowDown(const Eigen::Matrix4d &goal, vector<Eigen::Vector4d> &com_array, vector<vector<Eigen::Vector3d>> &ans,
                                  int num, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array2,
                                  const vector<Eigen::Vector4d> &swing_leg_cp, const vector<int> &sequence, const vector<Eigen::Matrix4d> &t1wrt0);
        vector<vector<Eigen::Vector3d>> RP_Helping(double r, double p, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array, bool b);
        bool modify;

    public:
        // unique_ptr<LegMotionPlanningNamspace::LegMotionPlanning> leg_planner_solver;
        unique_ptr<Gaits> gait_solver;
        ComplexPostures(double angle_precision_, double displacement_precision_);
        //+ve pitch -> down
        vector<vector<Eigen::Vector3d>> BowDown(double x_shift, double y_shift, double pitch_angle, double translate, string &str, int static_num,
                                                double stride_s, double stride_h, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_pose_array,
                                                vector<KDL::JntArray> &arm_ik_array, const Eigen::Matrix4d &goal);
        vector<vector<Eigen::Vector3d>> RP_Dance(int abc, double x_shift, double y_shift, double z_shift, double roll_angle,
                                                 double pitch_angle, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_pose_array,
                                                 vector<KDL::JntArray> &arm_ik_array, const Eigen::Matrix4d &goal);
        vector<vector<Eigen::Vector3d>> P_Dance(double fy, double fz, double by, double bz, double pitch_angle,
                                                vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_pose_array,
                                                vector<KDL::JntArray> &arm_ik_array, const Eigen::Matrix4d &goal,
                                                const Eigen::Matrix4d &goal2);
        vector<vector<Eigen::Vector3d>> PYT_Dance(double backward_l, double radius, double yaw_angle, double height,
                                                  vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_pose_array,
                                                  vector<KDL::JntArray> &arm_ik_array, Eigen::Matrix4d &goal1,
                                                  Eigen::Matrix4d &goal2, Eigen::Vector3d &rpy1, Eigen::Vector3d &rpy2);
    };

    ComplexPostures::ComplexPostures(double angle_precision_, double displacement_precision_)
    {
        gait_solver = make_unique<Gaits>(angle_precision_, displacement_precision_);
        modify = true;
    }

    vector<vector<Eigen::Vector3d>> ComplexPostures::RP_Helping(double r, double p, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array, bool b)
    {
        vector<Eigen::Vector4d> com_array;

        Eigen::Vector3d rpy1(r, p, 0);
        vector<int> seq1 = {0, 1, 2};
        vector<Eigen::Vector3d> rpy_vector = {rpy1};
        vector<vector<int>> seq_vector = {seq1};
        gait_solver->leg_planner_solver->com_needs_to_be_computed_posture = false;
        gait_solver->leg_planner_solver->armik_needs_to_be_computed_posture = b;

        return gait_solver->leg_planner_solver->PerformRPY(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, rpy_vector, seq_vector, com_array, spot_com_pose_array, FootHold::DoNotReset);
    }

    vector<vector<Eigen::Vector3d>> ComplexPostures::RP_Dance(int abc, double x_shift, double y_shift, double z_shift, double roll_angle,
                                                              double pitch_angle, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_pose_array,
                                                              vector<KDL::JntArray> &arm_ik_array, const Eigen::Matrix4d &goal)
    {
        // vector<pair<Eigen::Vector4d, Eigen::Vector3d>> spot_pose_array1;
        // vector<KDL::JntArray> arm_ik_array1;
        // vector<Eigen::Matrix4d> t1wrt0, t1wrt0_;
        // vector<int> sequence(4);
        // gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.clear();

        // Eigen::Matrix4d goal2;
        // goal2.block<3, 3>(0, 0).setIdentity();
        // goal2.block<1, 3>(3, 0).setZero();

        // goal2(0, 3) = goal(0, 3);
        // goal2(2, 3) = goal(2, 3) - 0.25;
        // goal2(3, 3) = goal(3, 3);
        // goal2(1, 3) = -goal(1, 3);

        // // initial state information
        // Kinematics::robot_state initial_state_tracker = *(Kinematics::state_tracker);
        // Kinematics::Kinematics initial_kinematic_solver = *(Kinematics::kinematics_solver);
        // vector<vector<Eigen::Vector3d>> rp_ik, legs_ik, legs_ik1, legs_ik_static;
        // vector<Eigen::Vector4d> l1_cp, l2_cp, l3_cp, l4_cp, com_array;
        // Eigen::Matrix4d home_pose;
        // if (abc == 1)
        // { // quadruped
        //     for (int j = 1; j < static_cast<int>(pitch_angle); j++)
        //     {
        //         rp_ik = RP_Helping(j / 2.0, j, spot_pose_array, false);
        //         if (legs_ik.empty())
        //             legs_ik.insert(legs_ik.begin(), rp_ik.begin(), rp_ik.end());
        //         else
        //             legs_ik.insert(legs_ik.end(), rp_ik.begin(), rp_ik.end());
        //     }
        //     int num = spot_pose_array.size();
        //     // swing legs
        //     l1_cp = GenerateFootCP_ForBowDown(x_shift, z_shift, y_shift, 0);
        //     l3_cp = GenerateFootCP_ForBowDown(x_shift, z_shift, y_shift, 2);
        //     l2_cp = GenerateFootCP_ForBowDown((2 * z_shift), z_shift, 0.0, 1);

        //     gait_solver->leg_planner_solver->body_pose_needs_to_computed = false;
        //     sequence = {1, 2, 3, 0};
        //     for (int i = 0; i < legs_ik.size(); i++)
        //         t1wrt0.push_back(Kinematics::kinematics_solver->T0wrtW_inverse * Conversions::Pair_2Transform(spot_pose_array[i]));
        //     HelpingFunForBowDown(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array,
        //                          legs_ik, legs_ik.size(), spot_pose_array, l1_cp, sequence, t1wrt0);
        //     sequence = {0, 1, 3, 2};
        //     HelpingFunForBowDown(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array,
        //                          legs_ik, legs_ik.size(), spot_pose_array, l3_cp, sequence, t1wrt0);
        //     t1wrt0.clear();

        //     // arm
        //     vector<Eigen::Vector4d> cp = gait_solver->leg_planner_solver->arm_planner_solver->GenerateBSPlineCP(goal, 0.1);
        //     Curves::BSpline spine_arm(cp, 4);
        //     home_pose = Conversions::KDL_2Transform(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->sixDpose);
        //     gait_solver->leg_planner_solver->arm_planner_solver->BSplinePlanner(spine_arm, goal, spot_pose_array.size());
        //     if (arm_ik_array.empty())
        //         arm_ik_array.insert(arm_ik_array.begin(), gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.begin(),
        //                             gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.end());
        //     else
        //         arm_ik_array.insert(arm_ik_array.end(), gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.begin(),
        //                             gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.end());

        //     // static and swing leg
        //     modify = false; // legs_ik_static
        //     sequence = {0, 2, 3, 1};
        //     for (int i = 0; i < 75; i++)
        //         t1wrt0.push_back(Kinematics::kinematics_solver->T0wrtW_inverse * Conversions::Pair_2Transform(spot_pose_array[spot_pose_array.size() - 1]));
        //     HelpingFunForBowDown(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array,
        //                          legs_ik_static, t1wrt0.size(), spot_pose_array, l2_cp, sequence, t1wrt0);
        //     for (int i = 0; i < 75; i++)
        //     {
        //         legs_ik.push_back(legs_ik[legs_ik.size() - 1]);
        //         spot_pose_array.push_back(spot_pose_array[spot_pose_array.size() - 1]);
        //         arm_ik_array.push_back(arm_ik_array[arm_ik_array.size() - 1]);
        //     }
        //     for (int i = 0; i < 75; i++)
        //     {
        //         legs_ik[legs_ik.size() + i - 75][1] = legs_ik_static[i][1];
        //         arm_ik_array.push_back(arm_ik_array[arm_ik_array.size() - 1]);
        //     }
        //     // reverse motion
        //     vector<vector<Eigen::Vector3d>> legs_ik2 = legs_ik;
        //     std::reverse(legs_ik2.begin(), legs_ik2.end());
        //     vector<pair<Eigen::Vector4d, Eigen::Vector3d>> spot_pose_array2 = spot_pose_array;
        //     std::reverse(spot_pose_array2.begin(), spot_pose_array2.end());
        //     legs_ik.insert(legs_ik.end(), legs_ik2.begin(), legs_ik2.end());
        //     spot_pose_array.insert(spot_pose_array.end(), spot_pose_array2.begin(), spot_pose_array2.end());
        //     // arm
        //     std::reverse(cp.begin(), cp.end());
        //     Curves::BSpline spine2_arm(cp, 4);
        //     gait_solver->leg_planner_solver->arm_planner_solver->BSplinePlanner(spine2_arm, home_pose, num);
        //     arm_ik_array.insert(arm_ik_array.end(), gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.begin(),
        //                         gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.end());

        //     *(Kinematics::state_tracker) = initial_state_tracker;
        //     *(Kinematics::kinematics_solver) = initial_kinematic_solver;
        //     gait_solver->leg_planner_solver->com_needs_to_be_computed_posture = true;
        //     gait_solver->leg_planner_solver->armik_needs_to_be_computed_posture = true;
        //     gait_solver->leg_planner_solver->body_pose_needs_to_computed = true;
        //     modify = true;
        //     l1_cp.clear();
        //     l2_cp.clear();
        //     l3_cp.clear();
        //     l4_cp.clear();
        // } ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // else
        // {
        //     initial_state_tracker = *(Kinematics::state_tracker);
        //     initial_kinematic_solver = *(Kinematics::kinematics_solver);
        //     // quadruped
        //     vector<vector<Eigen::Vector3d>> rp_ik1, legs_ik1;
        //     // for (int j = -1; j > -static_cast<int>(pitch_angle); j--)
        //     // for (int j = 1; j < static_cast<int>(pitch_angle); j++)
        //     for (int j = -1; j > -static_cast<int>(pitch_angle); j--)
        //     {
        //         rp_ik1 = RP_Helping(j / 2.0, j, spot_pose_array1, false);
        //         if (legs_ik1.empty())
        //             legs_ik1.insert(legs_ik1.begin(), rp_ik1.begin(), rp_ik1.end());
        //         else
        //             legs_ik1.insert(legs_ik1.end(), rp_ik1.begin(), rp_ik1.end());
        //     }
        //     int num1 = spot_pose_array1.size();

        //     // swing legs
        //     l2_cp = GenerateFootCP_ForBowDown(x_shift, z_shift, -y_shift, 1);
        //     l4_cp = GenerateFootCP_ForBowDown(x_shift, z_shift, -y_shift, 3);
        //     l3_cp = GenerateFootCP_ForBowDown((2 * z_shift), z_shift, 0.0, 2);
        //     gait_solver->leg_planner_solver->body_pose_needs_to_computed = false;
        //     sequence = {0, 2, 3, 1};
        //     for (int i = 0; i < legs_ik1.size(); i++)
        //         t1wrt0_.push_back(Kinematics::kinematics_solver->T0wrtW_inverse * Conversions::Pair_2Transform(spot_pose_array1[i]));
        //     HelpingFunForBowDown(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array,
        //                          legs_ik1, legs_ik1.size(), spot_pose_array1, l2_cp, sequence, t1wrt0_);
        //     sequence = {0, 1, 2, 3};
        //     HelpingFunForBowDown(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array,
        //                          legs_ik1, legs_ik1.size(), spot_pose_array1, l4_cp, sequence, t1wrt0_);
        //     t1wrt0_.clear();

        //     // arm
        //     vector<Eigen::Vector4d> cp2 = gait_solver->leg_planner_solver->arm_planner_solver->GenerateBSPlineCP(goal2, 0.1);
        //     Curves::BSpline spine_arm2(cp2, 4);
        //     Eigen::Matrix4d home_pose2 = Conversions::KDL_2Transform(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->sixDpose);
        //     gait_solver->leg_planner_solver->arm_planner_solver->BSplinePlanner(spine_arm2, goal2, spot_pose_array1.size());
        //     if (arm_ik_array1.empty())
        //         arm_ik_array1.insert(arm_ik_array1.begin(), gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.begin(),
        //                              gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.end());
        //     else
        //         arm_ik_array1.insert(arm_ik_array1.end(), gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.begin(),
        //                              gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.end());

        //     // static and swing leg
        //     modify = false; // legs_ik_static
        //     legs_ik_static.clear();
        //     sequence = {0, 1, 3, 2};
        //     for (int i = 0; i < 75; i++)
        //         t1wrt0_.push_back(Kinematics::kinematics_solver->T0wrtW_inverse * Conversions::Pair_2Transform(spot_pose_array1[spot_pose_array1.size() - 1]));
        //     HelpingFunForBowDown(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array,
        //                          legs_ik_static, t1wrt0_.size(), spot_pose_array1, l3_cp, sequence, t1wrt0_);

        //     for (int i = 0; i < 75; i++)
        //     {
        //         legs_ik1.push_back(legs_ik1[legs_ik1.size() - 1]);
        //         spot_pose_array1.push_back(spot_pose_array1[spot_pose_array1.size() - 1]);
        //         arm_ik_array1.push_back(arm_ik_array1[arm_ik_array1.size() - 1]);
        //     }

        //     for (int i = 0; i < 75; i++)
        //     {
        //         legs_ik1[legs_ik1.size() + i - 75][2] = legs_ik_static[i][2];
        //         arm_ik_array1.push_back(arm_ik_array1[arm_ik_array1.size() - 1]);
        //     }

        //     // reverse motion
        //     vector<vector<Eigen::Vector3d>> legs_ik3 = legs_ik1;
        //     std::reverse(legs_ik3.begin(), legs_ik3.end());
        //     vector<pair<Eigen::Vector4d, Eigen::Vector3d>> spot_pose_array3 = spot_pose_array1;
        //     std::reverse(spot_pose_array3.begin(), spot_pose_array3.end());
        //     legs_ik1.insert(legs_ik1.end(), legs_ik3.begin(), legs_ik3.end());
        //     spot_pose_array1.insert(spot_pose_array1.end(), spot_pose_array3.begin(), spot_pose_array3.end());
        //     // arm
        //     std::reverse(cp2.begin(), cp2.end());
        //     Curves::BSpline spine2_arm2(cp2, 4);
        //     gait_solver->leg_planner_solver->arm_planner_solver->BSplinePlanner(spine2_arm2, home_pose, num1);
        //     arm_ik_array1.insert(arm_ik_array1.end(), gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.begin(),
        //                          gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.end());

        //     *(Kinematics::state_tracker) = initial_state_tracker;
        //     *(Kinematics::kinematics_solver) = initial_kinematic_solver;
        //     gait_solver->leg_planner_solver->com_needs_to_be_computed_posture = true;
        //     gait_solver->leg_planner_solver->armik_needs_to_be_computed_posture = true;
        //     gait_solver->leg_planner_solver->body_pose_needs_to_computed = true;
        //     modify = true;

        //     legs_ik.insert(legs_ik.end(), legs_ik1.begin(), legs_ik1.end());
        //     arm_ik_array.insert(arm_ik_array.end(), arm_ik_array1.begin(), arm_ik_array1.end());
        //     spot_pose_array.insert(spot_pose_array.end(), spot_pose_array1.begin(), spot_pose_array1.end());
        // }

        // return legs_ik;
    }

    vector<Eigen::Vector4d> ComplexPostures::GenerateFootCP_ForBowDown(double stride_s, double stride_h, double y_shift, int leg_index)
    {
        double del_y = y_shift / 6.0;
        double h1 = stride_h / 3.5;
        double del_x = stride_s / 3.0;

        Eigen::Vector4d p1 = Kinematics::kinematics_solver->legs[leg_index].xyz;
        Eigen::Vector4d p2(p1(0) - h1, p1(1) + (1.0 * del_y), p1(2) + (2.5 * h1), 1.0);
        Eigen::Vector4d p3(p1(0) - h1, p1(1) + (2.0 * del_y), p1(2) + stride_h, 1.0);
        Eigen::Vector4d p4(p1(0), p1(1) + (3.0 * del_y), p1(2) + stride_h, 1.0);
        Eigen::Vector4d p5(p1(0) + (1.0 * stride_s) / 3.0, p1(1) + (4.0 * del_y), p1(2) + (2.0 * stride_h) / 3.0, 1.0);
        Eigen::Vector4d p6(p1(0) + (2.0 * stride_s) / 3.0, p1(1) + (5.0 * del_y), p1(2) + (1.0 * stride_h) / 3.0, 1.0);
        Eigen::Vector4d p7(p1(0) + (stride_s), p1(1) + (6.0 * del_y), p1(2), 1.0);

        vector<Eigen::Vector4d> ans = {p1, p2, p3, p4, p5, p6, p7};
        return ans;
    }

    vector<Eigen::Vector4d> ComplexPostures::GenerateFootCP_ForDancing(double x_max, double y_max, double z_max, double z_f, int leg_index)
    {
        double del_x = x_max / 3.0;
        double del_y = y_max / 6.0;
        double del_z = z_max / 3.0;
        double stride_h = (z_max - z_f);
        Eigen::Vector4d p1 = Kinematics::kinematics_solver->legs[leg_index].xyz;
        Eigen::Vector4d p2(p1(0) - del_x, p1(1) + (1.0 * del_y), p1(2) + (2.0 * del_z), 1.0);
        Eigen::Vector4d p3(p1(0) - del_x, p1(1) + (2.0 * del_y), p1(2) + z_max, 1.0);
        Eigen::Vector4d p4(p1(0), p1(1) + (3.0 * del_y), p1(2) + z_max, 1.0);

        Eigen::Vector4d p5(p1(0) + (1.0 * del_x) / 3.0, p1(1) + (4.0 * del_y), p1(2) + z_f + (2.0 * stride_h) / 3.0, 1.0);
        Eigen::Vector4d p6(p1(0) + (2.0 * del_x) / 3.0, p1(1) + (5.0 * del_y), p1(2) + z_f + (1.0 * stride_h) / 3.0, 1.0);
        Eigen::Vector4d p7(p1(0) + (x_max), p1(1) + (6.0 * del_y), p1(2) + z_f, 1.0);

        vector<Eigen::Vector4d> ans = {p1, p2, p3, p4, p5, p6, p7};
        return ans;
    }

    vector<Eigen::Vector4d> ComplexPostures::GenerateFootCP_ForSpotDancing(double backward_l, double radius, int leg_index)
    {
        Eigen::Vector4d p1l1 = Kinematics::kinematics_solver->legs[leg_index].xyz;
        Eigen::Vector4d p2l1 = Kinematics::kinematics_solver->legs[leg_index].xyz;
        p2l1(0) = p1l1(0) - backward_l;

        Eigen::Vector4d p1l2 = p2l1;
        p1l2(2) = p2l1(2) + (2 * radius);

        Eigen::Vector4d p2l2 = Kinematics::kinematics_solver->legs[leg_index].xyz;
        Eigen::Vector4d p3 = (p2l1 + p1l2) / 2;
        p3(3) = 1.0;
        p3(0) = p2l1(0) - radius;
        Curves::Line l1(p1l1, p2l1), l2(p1l2, p2l2);
        Curves::Arc semi_circle(p2l1, p3, p1l2, "C");
        semi_circle.point_generation = "default";
        int num = 3;

        vector<Eigen::Vector4d> l1_xyzs = l1.Generate3DPoints(num);
        vector<Eigen::Vector4d> l2_xyzs = l2.Generate3DPoints(num);
        vector<Eigen::Vector4d> arc_xyzs = semi_circle.Generate3DPoints(num);

        vector<Eigen::Vector4d> ans(l1_xyzs);
        ans.insert(ans.end(), arc_xyzs.begin(), arc_xyzs.end());
        ans.insert(ans.end(), l2_xyzs.begin(), l2_xyzs.end());
        semi_circle.point_generation = "";
        return ans;
    }

    void ComplexPostures::HelpingFunForBowDown(const Eigen::Matrix4d &goal, vector<Eigen::Vector4d> &com_array, vector<vector<Eigen::Vector3d>> &ans,
                                               int num, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array2,
                                               const vector<Eigen::Vector4d> &swing_leg_cp, const vector<int> &sequence, const vector<Eigen::Matrix4d> &t1wrt0)
    {
        // set all 4 legs motion planning parameters : control points for swing and stance legs, num of points, type of path
        gait_solver->leg_planner_solver->leg_planning_param[sequence[3]].foot_control_points = swing_leg_cp;
        gait_solver->SetStanceLegs(sequence);

        gait_solver->leg_planner_solver->leg_planning_param[0].num_of_points =
            gait_solver->leg_planner_solver->leg_planning_param[1].num_of_points =
                gait_solver->leg_planner_solver->leg_planning_param[2].num_of_points =
                    gait_solver->leg_planner_solver->leg_planning_param[3].num_of_points = num;
        gait_solver->leg_planner_solver->leg_planning_param[0].curve_type =
            gait_solver->leg_planner_solver->leg_planning_param[1].curve_type =
                gait_solver->leg_planner_solver->leg_planning_param[2].curve_type =
                    gait_solver->leg_planner_solver->leg_planning_param[3].curve_type = Curves::CurveType::B_Spline;
        gait_solver->leg_planner_solver->com_needs_to_be_computed_posture = false;
        vector<vector<Eigen::Vector3d>> legs_ik = gait_solver->leg_planner_solver->GenerateLegsMotion(goal, com_array, t1wrt0, spot_com_pose_array2);
        if (ans.empty())
            ans.insert(ans.begin(), legs_ik.begin(), legs_ik.end());
        else
        {
            if (modify)
                gait_solver->ReplaceColumn(ans, legs_ik, sequence[3]);
            else
                ans.insert(ans.end(), legs_ik.begin(), legs_ik.end());
        }
        vector<Eigen::Vector4d> stance_legs_single_state(3);
        for (int i = 0; i < legs_ik.size(); i++)
        {
            stance_legs_single_state[0] = Kinematics::kinematics_solver->T0wrtW * Kinematics::kinematics_solver->legs[sequence[0]].xyz;
            stance_legs_single_state[1] = Kinematics::kinematics_solver->T0wrtW * Kinematics::kinematics_solver->legs[sequence[1]].xyz;
            stance_legs_single_state[2] = Kinematics::kinematics_solver->T0wrtW * Kinematics::kinematics_solver->legs[sequence[2]].xyz;
            Kinematics::kinematics_solver->stance_legs_wrt_world.push_back(stance_legs_single_state);
        }
        int indexing = gait_solver->leg_planner_solver->leg_planning_param[sequence[3]].foot_control_points.size() - 1;
        Kinematics::kinematics_solver->legs[sequence[3]].xyz = gait_solver->leg_planner_solver->leg_planning_param[sequence[3]].foot_control_points[indexing];
    }

    vector<vector<Eigen::Vector3d>> ComplexPostures::PYT_Dance(double backward_l, double radius, double yaw_angle, double height,
                                                               vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_pose_array,
                                                               vector<KDL::JntArray> &arm_ik_array, Eigen::Matrix4d &goal1,
                                                               Eigen::Matrix4d &goal2, Eigen::Vector3d &rpy1, Eigen::Vector3d &rpy2)
    {
        vector<vector<Eigen::Vector3d>> legs_ik, yaw_ik;
        vector<Eigen::Vector4d> l1_cp, l2_cp, l3_cp, l4_cp, com_array;
        vector<pair<Eigen::Vector4d, Eigen::Vector3d>> spot_pose_array2;
        vector<Eigen::Matrix4d> t1wrt0;
        Eigen::Matrix4d dummy_pose;

        gait_solver->leg_planner_solver->com_needs_to_be_computed_posture = false;
        gait_solver->leg_planner_solver->armik_needs_to_be_computed_posture = false;

        vector<int> seq = {0, 1, 2};
        vector<int> sequence(4);
        Eigen::Vector4d rp(0, 0, height, 1.0);
        modify = true;
        vector<Eigen::Vector4d> arm_cp;
        unique_ptr<Curves::BSpline> arm_bspline;

        Eigen::Matrix3d R1 = CommonMathsSolver::OrientationNTransformaton::Compute_Rz(rpy1(2));
        Eigen::Matrix3d R2 = CommonMathsSolver::OrientationNTransformaton::Compute_Rz(rpy2(2));
        goal1.block<3, 3>(0, 0) = R1;
        goal2.block<3, 3>(0, 0) = R2;
        cout << " R1 " << R1 << endl;
        cout << " R2 " << R2 << endl;

        for (int i = 0; i < 9; i++)
        {
            Eigen::Vector3d rpy;
            if (i % 2 == 0)
                rpy << 0, R2D * Kinematics::state_tracker->body_rpy(1), yaw_angle;
            else
                rpy << 0, R2D * Kinematics::state_tracker->body_rpy(1), -yaw_angle;

            // yaw + ascent
            yaw_ik = gait_solver->leg_planner_solver->GenerateRPYTranslation(rpy, seq, rp, spot_pose_array2, FootHold::Vertical);

            (spot_pose_array.empty()) ? spot_pose_array.insert(spot_pose_array.begin(), spot_pose_array2.begin(), spot_pose_array2.end())
                                      : spot_pose_array.insert(spot_pose_array.end(), spot_pose_array2.begin(), spot_pose_array2.end());
            (legs_ik.empty()) ? legs_ik.insert(legs_ik.begin(), yaw_ik.begin(), yaw_ik.end())
                              : legs_ik.insert(legs_ik.end(), yaw_ik.begin(), yaw_ik.end());

            for (int i = 0; i < yaw_ik.size(); i++)
                t1wrt0.push_back(Kinematics::kinematics_solver->T0wrtW_inverse * Conversions::Pair_2Transform(spot_pose_array2[i]));
            gait_solver->leg_planner_solver->body_pose_needs_to_computed = false;
            // alternate legs
            if (i % 2 == 0)
            {
                l1_cp = GenerateFootCP_ForSpotDancing(backward_l, radius, 0);
                l3_cp = GenerateFootCP_ForSpotDancing(backward_l, radius, 2);
                sequence = {1, 2, 3, 0};
                HelpingFunForBowDown(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array,
                                     legs_ik, yaw_ik.size(), spot_pose_array, l1_cp, sequence, t1wrt0);
                sequence = {0, 1, 3, 2};
                HelpingFunForBowDown(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array,
                                     legs_ik, yaw_ik.size(), spot_pose_array, l3_cp, sequence, t1wrt0);
            }
            else
            {
                l2_cp = GenerateFootCP_ForSpotDancing(backward_l, radius, 1);
                l4_cp = GenerateFootCP_ForSpotDancing(backward_l, radius, 3);
                sequence = {0, 2, 3, 1};
                HelpingFunForBowDown(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array,
                                     legs_ik, yaw_ik.size(), spot_pose_array, l2_cp, sequence, t1wrt0);
                sequence = {0, 1, 2, 3};
                HelpingFunForBowDown(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array,
                                     legs_ik, yaw_ik.size(), spot_pose_array, l4_cp, sequence, t1wrt0);
            }

            // move arm
            Eigen::Matrix4d current_pose;
            (i > 0) ? current_pose = gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b
                    : current_pose = Conversions::KDL_2Transform(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->sixDpose);

            arm_cp = gait_solver->leg_planner_solver->arm_planner_solver->GenerateBSPlineCP((i % 2 == 0) ? goal1 : goal2, current_pose, 0.1, false);
            arm_bspline = make_unique<Curves::BSpline>(arm_cp, 4);
            // Eigen::Matrix4d current_pose = Conversions::KDL_2Transform(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->sixDpose);
            gait_solver->leg_planner_solver->arm_planner_solver->BSplinePlanner(*arm_bspline, goal1, spot_pose_array2.size(), current_pose);
            //(condition) ? true : false
            (arm_ik_array.empty()) ? arm_ik_array.insert(arm_ik_array.begin(), gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.begin(),
                                                         gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.end())
                                   : arm_ik_array.insert(arm_ik_array.end(), gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.begin(),
                                                         gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.end());
            arm_cp.clear();
            t1wrt0.clear();
            yaw_ik.clear();
            spot_pose_array2.clear();
        }

        return legs_ik;
    }

    vector<vector<Eigen::Vector3d>> ComplexPostures::P_Dance(double fy, double fz, double by, double bz, double pitch_angle,
                                                             vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_pose_array,
                                                             vector<KDL::JntArray> &arm_ik_array, const Eigen::Matrix4d &goal,
                                                             const Eigen::Matrix4d &goal2)
    {
        vector<vector<Eigen::Vector3d>> legs_ik, pitch_ik;
        vector<Eigen::Vector4d> l1_cp, l2_cp, l3_cp, l4_cp, com_array;
        vector<pair<Eigen::Vector4d, Eigen::Vector3d>> spot_pose_array2;
        vector<Eigen::Matrix4d> t1wrt0;
        Eigen::Matrix4d dummy_pose;
        // initial state information
        // Kinematics::robot_state initial_state_tracker = *(Kinematics::state_tracker);
        // Kinematics::Kinematics initial_kinematic_solver = *(Kinematics::kinematics_solver);

        gait_solver->leg_planner_solver->com_needs_to_be_computed_posture = false;
        gait_solver->leg_planner_solver->armik_needs_to_be_computed_posture = false;

        // pitch + arm strech
        pitch_ik = RP_Helping(0.0, pitch_angle, spot_pose_array, false);
        legs_ik.insert(legs_ik.begin(), pitch_ik.begin(), pitch_ik.end());
        vector<Eigen::Vector4d> cp = gait_solver->leg_planner_solver->arm_planner_solver->GenerateBSPlineCP(goal, dummy_pose, 0.1, true);
        // cp = gait_solver->leg_planner_solver->arm_planner_solver->DancingGenerateBSPlineCP(cp, goal, 0.4, -0.4, 5);
        Curves::BSpline spine_arm(cp, 4);
        Eigen::Matrix4d home_pose = Conversions::KDL_2Transform(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->sixDpose);
        Eigen::Matrix4d pose1 = Conversions::KDL_2Transform(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->sixDpose);
        gait_solver->leg_planner_solver->arm_planner_solver->BSplinePlanner(spine_arm, goal, spot_pose_array.size(), pose1);
        arm_ik_array.insert(arm_ik_array.begin(), gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.begin(),
                            gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.end());

        // pitch + ch + backlegs
        gait_solver->leg_planner_solver->str = "true";
        modify = true;
        pitch_ik.clear();
        // pitch + ch
        pitch_ik = RP_Helping(0.0, pitch_angle + 10.0, spot_pose_array2, true);
        Eigen::Matrix4d current_pose = gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b;

        for (int i = 0; i < pitch_ik.size(); i++)
            t1wrt0.push_back(Kinematics::kinematics_solver->T0wrtW_inverse * Conversions::Pair_2Transform(spot_pose_array2[i]));

        legs_ik.insert(legs_ik.end(), pitch_ik.begin(), pitch_ik.end());
        spot_pose_array.insert(spot_pose_array.end(), spot_pose_array2.begin(), spot_pose_array2.end());
        arm_ik_array.insert(arm_ik_array.end(), gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.begin(),
                            gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.end());
        l2_cp = GenerateFootCP_ForBowDown(0.0, bz, -by / 1.25, 1);
        l3_cp = GenerateFootCP_ForBowDown(0.0, bz, by / 1.25, 2);
        gait_solver->leg_planner_solver->body_pose_needs_to_computed = false;
        gait_solver->leg_planner_solver->armik_needs_to_be_computed_posture = false;

        vector<int> sequence = {0, 2, 3, 1};
        HelpingFunForBowDown(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array,
                             legs_ik, pitch_ik.size(), spot_pose_array, l2_cp, sequence, t1wrt0);
        sequence = {0, 1, 3, 2};
        HelpingFunForBowDown(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array,
                             legs_ik, pitch_ik.size(), spot_pose_array, l3_cp, sequence, t1wrt0);

        // dancing arm + up pitch
        pitch_ik.clear();
        spot_pose_array2.clear();
        pitch_ik = RP_Helping(0.0, 0.0, spot_pose_array2, false);
        cp.clear();
        cp = gait_solver->leg_planner_solver->arm_planner_solver->GenerateBSPlineCP(goal2, current_pose, -0.3, false);
        cp = gait_solver->leg_planner_solver->arm_planner_solver->DancingGenerateBSPlineCP(cp, goal2, 0.4, 0.4, 5);
        Curves::BSpline spine2_arm(cp, 4);
        gait_solver->leg_planner_solver->arm_planner_solver->BSplinePlanner(spine2_arm, goal2, pitch_ik.size(),
                                                                            current_pose);
        legs_ik.insert(legs_ik.end(), pitch_ik.begin(), pitch_ik.end());
        arm_ik_array.insert(arm_ik_array.end(), gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.begin(),
                            gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.end());
        spot_pose_array.insert(spot_pose_array.end(), spot_pose_array2.begin(), spot_pose_array2.end());

        // up pitch + home pose + l1 l2
        pitch_ik.clear();
        spot_pose_array2.clear();
        pitch_ik = RP_Helping(0.0, -pitch_angle, spot_pose_array2, false);
        t1wrt0.clear();
        for (int i = 0; i < pitch_ik.size(); i++)
            t1wrt0.push_back(Kinematics::kinematics_solver->T0wrtW_inverse * Conversions::Pair_2Transform(spot_pose_array2[i]));
        legs_ik.insert(legs_ik.end(), pitch_ik.begin(), pitch_ik.end());

        l1_cp = GenerateFootCP_ForBowDown(0.0, bz, by, 0);
        l4_cp = GenerateFootCP_ForBowDown(0.0, bz, -by, 3);
        gait_solver->leg_planner_solver->body_pose_needs_to_computed = false;
        gait_solver->leg_planner_solver->armik_needs_to_be_computed_posture = false;
        sequence = {1, 2, 3, 0};
        HelpingFunForBowDown(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array,
                             legs_ik, pitch_ik.size(), spot_pose_array, l1_cp, sequence, t1wrt0);
        sequence = {0, 1, 2, 3};
        HelpingFunForBowDown(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array,
                             legs_ik, pitch_ik.size(), spot_pose_array, l4_cp, sequence, t1wrt0);
        cp.clear();
        current_pose = gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b;
        Eigen::Matrix4d goal_pd;
        goal_pd << 0, 1, 0, 0.4,
            0, 0, -1, 0,
            -1, 0, 0, -0.4,
            0, 0, 0, 1;
        cp = gait_solver->leg_planner_solver->arm_planner_solver->GenerateBSPlineCP(goal_pd, current_pose, -0.3, false);
        cp = gait_solver->leg_planner_solver->arm_planner_solver->DancingGenerateBSPlineCP(cp, goal_pd, 0.4, 0.4, 5);
        Curves::BSpline spine3_arm(cp, 4);
        gait_solver->leg_planner_solver->arm_planner_solver->BSplinePlanner(spine3_arm, home_pose, pitch_ik.size(),
                                                                            current_pose);
        arm_ik_array.insert(arm_ik_array.end(), gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.begin(),
                            gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.end());
        spot_pose_array.insert(spot_pose_array.end(), spot_pose_array2.begin(), spot_pose_array2.end());

        // last pitch + ch
        pitch_ik.clear();
        spot_pose_array2.clear();
        pitch_ik = RP_Helping(0.0, pitch_angle / 3.0, spot_pose_array2, true);
        spot_pose_array.insert(spot_pose_array.end(), spot_pose_array2.begin(), spot_pose_array2.end());
        legs_ik.insert(legs_ik.end(), pitch_ik.begin(), pitch_ik.end());
        arm_ik_array.insert(arm_ik_array.end(), gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.begin(),
                            gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.end());

        // pitch + arm + l2 jump
        // When I will reverse the motion, final state of the reverse motion will not be computed by the algo. So, I need to store that state in another variable
        Kinematics::robot_state initial_state_tracker = *(Kinematics::state_tracker);
        Kinematics::Kinematics initial_kinematic_solver = *(Kinematics::kinematics_solver);
        pitch_ik.clear();
        spot_pose_array2.clear();
        pitch_ik = RP_Helping(0.0, pitch_angle + 10.0, spot_pose_array2, false);
        t1wrt0.clear();
        for (int i = 0; i < pitch_ik.size(); i++)
            t1wrt0.push_back(Kinematics::kinematics_solver->T0wrtW_inverse * Conversions::Pair_2Transform(spot_pose_array2[i]));
        legs_ik.insert(legs_ik.end(), pitch_ik.begin(), pitch_ik.end());
        spot_pose_array.insert(spot_pose_array.end(), spot_pose_array2.begin(), spot_pose_array2.end());

        gait_solver->leg_planner_solver->body_pose_needs_to_computed = false;
        gait_solver->leg_planner_solver->armik_needs_to_be_computed_posture = false;
        sequence = {0, 2, 3, 1};
        l2_cp.clear();
        l2_cp = GenerateFootCP_ForDancing(-2 * bz, 2 * bz, 2 * bz, bz, 1);
        HelpingFunForBowDown(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array,
                             legs_ik, pitch_ik.size(), spot_pose_array, l2_cp, sequence, t1wrt0);
        cp.clear();
        current_pose = gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b;
        Eigen::Matrix4d intermediate_pose = current_pose;
        goal_pd = current_pose;
        goal_pd.block<3, 3>(0, 0).setIdentity();
        goal_pd(1, 3) = -0.2;
        goal_pd(0, 3) = current_pose(0, 3) - 0.1;
        cp = gait_solver->leg_planner_solver->arm_planner_solver->GenerateBSPlineCP(goal_pd, current_pose, 0.3, false);
        cp = gait_solver->leg_planner_solver->arm_planner_solver->DancingGenerateBSPlineCP(cp, goal_pd, 0.4, 0.2, 6);
        Curves::BSpline spine4_arm(cp, 4);
        gait_solver->leg_planner_solver->arm_planner_solver->BSplinePlanner(spine4_arm, goal_pd, pitch_ik.size(),
                                                                            current_pose);
        arm_ik_array.insert(arm_ik_array.end(), gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.begin(),
                            gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.end());

        // reverse
        std::reverse(spot_pose_array2.begin(), spot_pose_array2.end());
        vector<vector<Eigen::Vector3d>> leg_ik_reverse(legs_ik.end() - pitch_ik.size(), legs_ik.end());
        std::reverse(leg_ik_reverse.begin(), leg_ik_reverse.end());
        legs_ik.insert(legs_ik.end(), leg_ik_reverse.begin(), leg_ik_reverse.end());
        spot_pose_array.insert(spot_pose_array.end(), spot_pose_array2.begin(), spot_pose_array2.end());
        current_pose = gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b;
        goal_pd = intermediate_pose;
        cp = gait_solver->leg_planner_solver->arm_planner_solver->GenerateBSPlineCP(goal_pd, current_pose, 0.3, false);
        cp = gait_solver->leg_planner_solver->arm_planner_solver->DancingGenerateBSPlineCP(cp, goal_pd, 0.4, 0.2, 6);
        Curves::BSpline spine5_arm(cp, 4);
        gait_solver->leg_planner_solver->arm_planner_solver->BSplinePlanner(spine5_arm, goal_pd, pitch_ik.size(),
                                                                            current_pose);
        arm_ik_array.insert(arm_ik_array.end(), gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.begin(),
                            gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.end());
        *(Kinematics::state_tracker) = initial_state_tracker;
        *(Kinematics::kinematics_solver) = initial_kinematic_solver;

        // pitch + arm + l3 jump
        initial_state_tracker = *(Kinematics::state_tracker);
        initial_kinematic_solver = *(Kinematics::kinematics_solver);
        pitch_ik.clear();
        spot_pose_array2.clear();
        pitch_ik = RP_Helping(0.0, pitch_angle + 10.0, spot_pose_array2, false);
        t1wrt0.clear();
        for (int i = 0; i < pitch_ik.size(); i++)
            t1wrt0.push_back(Kinematics::kinematics_solver->T0wrtW_inverse * Conversions::Pair_2Transform(spot_pose_array2[i]));
        legs_ik.insert(legs_ik.end(), pitch_ik.begin(), pitch_ik.end());
        spot_pose_array.insert(spot_pose_array.end(), spot_pose_array2.begin(), spot_pose_array2.end());

        gait_solver->leg_planner_solver->body_pose_needs_to_computed = false;
        gait_solver->leg_planner_solver->armik_needs_to_be_computed_posture = false;
        sequence = {0, 1, 3, 2};
        l3_cp.clear();
        l3_cp = GenerateFootCP_ForDancing(-2 * bz, -2 * bz, 2 * bz, bz, 2);
        HelpingFunForBowDown(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array,
                             legs_ik, pitch_ik.size(), spot_pose_array, l3_cp, sequence, t1wrt0);
        cp.clear();
        current_pose = gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b;
        intermediate_pose = current_pose;

        goal_pd = current_pose;
        goal_pd.block<3, 3>(0, 0).setIdentity();
        goal_pd(1, 3) = 0.2;
        goal_pd(0, 3) = current_pose(0, 3) - 0.1;
        goal_pd(0, 0) = -1;
        goal_pd(1, 1) = -1;
        cp = gait_solver->leg_planner_solver->arm_planner_solver->GenerateBSPlineCP(goal_pd, current_pose, 0.3, false);
        cp = gait_solver->leg_planner_solver->arm_planner_solver->DancingGenerateBSPlineCP(cp, goal_pd, 0.4, 0.2, 6);
        Curves::BSpline spine6_arm(cp, 4);
        gait_solver->leg_planner_solver->arm_planner_solver->BSplinePlanner(spine6_arm, goal_pd, pitch_ik.size(),
                                                                            current_pose);
        arm_ik_array.insert(arm_ik_array.end(), gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.begin(),
                            gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.end());

        // reverse
        std::reverse(spot_pose_array2.begin(), spot_pose_array2.end());
        vector<vector<Eigen::Vector3d>> leg_ik_reverse2(legs_ik.end() - pitch_ik.size(), legs_ik.end());
        std::reverse(leg_ik_reverse2.begin(), leg_ik_reverse2.end());
        legs_ik.insert(legs_ik.end(), leg_ik_reverse2.begin(), leg_ik_reverse2.end());
        spot_pose_array.insert(spot_pose_array.end(), spot_pose_array2.begin(), spot_pose_array2.end());
        current_pose = gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b;
        goal_pd = intermediate_pose;
        cp = gait_solver->leg_planner_solver->arm_planner_solver->GenerateBSPlineCP(goal_pd, current_pose, 0.3, false);
        cp = gait_solver->leg_planner_solver->arm_planner_solver->DancingGenerateBSPlineCP(cp, goal_pd, 0.4, 0.2, 6);
        Curves::BSpline spine7_arm(cp, 4);
        gait_solver->leg_planner_solver->arm_planner_solver->BSplinePlanner(spine7_arm, goal_pd, pitch_ik.size(),
                                                                            current_pose);
        arm_ik_array.insert(arm_ik_array.end(), gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.begin(),
                            gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.end());
        *(Kinematics::state_tracker) = initial_state_tracker;
        *(Kinematics::kinematics_solver) = initial_kinematic_solver;

        return legs_ik;
    }

    vector<vector<Eigen::Vector3d>> ComplexPostures::BowDown(double x_shift, double y_shift, double pitch_angle, double translate, string &str, int static_num,
                                                             double stride_s, double stride_h, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_pose_array,
                                                             vector<KDL::JntArray> &arm_ik_array, const Eigen::Matrix4d &goal)
    {
        vector<vector<Eigen::Vector3d>> ans, pitch_ik, translate_ik;
        vector<Eigen::Vector4d> rl_cp, ll_cp;
        vector<pair<Eigen::Vector4d, Eigen::Vector3d>> spot_pose_array2;
        vector<Eigen::Matrix4d> t1wrt0;
        Eigen::Matrix4d dummy_pose;
        // initial state information
        Kinematics::robot_state initial_state_tracker = *(Kinematics::state_tracker);
        Kinematics::Kinematics initial_kinematic_solver = *(Kinematics::kinematics_solver);

        gait_solver->leg_planner_solver->com_needs_to_be_computed_posture = false;
        gait_solver->leg_planner_solver->armik_needs_to_be_computed_posture = false;
        Eigen::Vector3d rpy(0, pitch_angle, 0);
        vector<Eigen::Vector3d> rpy_vector = {rpy};
        vector<int> seq = {0, 1, 2};
        vector<vector<int>> seq_vector = {seq};
        vector<Eigen::Vector4d> com_array;
        double x = (translate * cos(D2R * pitch_angle));
        double z = -(translate * sin(D2R * pitch_angle));
        Eigen::Vector4d rpv(x, 0.0, z, 1.0);
        vector<Eigen::Vector4d> rpv_vector = {rpv};

        if (str == "LL")
        {
            // 1. rl 2. pitch 3. trans 4. swing_leg => 4,2,3 parallel
            rl_cp = gait_solver->GenerateFootCP_ForCrawlGait(stride_s, stride_h, 3, "SP");
            ll_cp = GenerateFootCP_ForBowDown(x_shift, stride_h / 2.0, -y_shift, 0);

            // no com
            // swing->r
            vector<int> sequence = {0, 1, 2, 3};
            gait_solver->leg_planner_solver->body_pose_needs_to_computed = true;
            for (int i = 0; i < 50; i++)
                t1wrt0.push_back(Kinematics::kinematics_solver->T0wrtW_inverse * Conversions::Pair_2Transform(make_pair(Kinematics::state_tracker->body_xyz, Kinematics::state_tracker->body_rpy)));
            HelpingFunForBowDown(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b,
                                 com_array, ans, 50, spot_pose_array, rl_cp, sequence, t1wrt0);
            t1wrt0.clear();

            int num1 = ans.size();
            pitch_ik = gait_solver->leg_planner_solver->PerformRPY(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, rpy_vector, seq_vector, com_array, spot_pose_array, FootHold::DoNotReset);
            ans.insert(ans.end(), pitch_ik.begin(), pitch_ik.end());
            translate_ik = gait_solver->leg_planner_solver->PerformTranslation(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, rpv_vector, com_array, spot_pose_array, FootHold::DoNotReset);
            ans.insert(ans.end(), translate_ik.begin(), translate_ik.end());

            gait_solver->leg_planner_solver->body_pose_needs_to_computed = false;
            sequence = {1, 2, 3, 0};
            vector<pair<Eigen::Vector4d, Eigen::Vector3d>> spot_pose_array3(spot_pose_array.end() - (spot_pose_array.size() - num1), spot_pose_array.end());
            for (int i = 0; i < ans.size() - num1; i++)
                t1wrt0.push_back(Kinematics::kinematics_solver->T0wrtW_inverse * Conversions::Pair_2Transform(spot_pose_array3[i]));
            HelpingFunForBowDown(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array,
                                 ans, ans.size() - num1, spot_pose_array2, ll_cp, sequence, t1wrt0);
            int num = spot_pose_array.size();
            t1wrt0.clear();

            // arm ik computation
            Eigen::Matrix4d pose1 = Conversions::KDL_2Transform(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->sixDpose);
            vector<Eigen::Vector4d> cp = gait_solver->leg_planner_solver->arm_planner_solver->GenerateBSPlineCP(goal, dummy_pose, 0.1, true);
            Curves::BSpline spine_arm(cp, 4);
            Eigen::Matrix4d home_pose = Conversions::KDL_2Transform(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->sixDpose);
            gait_solver->leg_planner_solver->arm_planner_solver->BSplinePlanner(spine_arm, goal, spot_pose_array.size(), pose1);
            arm_ik_array.insert(arm_ik_array.begin(), gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.begin(),
                                gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.end());

            for (int i = 0; i < static_num; i++)
            {
                ans.push_back(ans[ans.size() - 1]);
                spot_pose_array.push_back(spot_pose_array[spot_pose_array.size() - 1]);
                arm_ik_array.push_back(arm_ik_array[arm_ik_array.size() - 1]);
            }

            vector<vector<Eigen::Vector3d>> ans2 = ans;
            std::reverse(ans2.begin(), ans2.end());
            vector<pair<Eigen::Vector4d, Eigen::Vector3d>> spot_pose_array2 = spot_pose_array;
            std::reverse(spot_pose_array2.begin(), spot_pose_array2.end());
            ans.insert(ans.end(), ans2.begin(), ans2.end());
            spot_pose_array.insert(spot_pose_array.end(), spot_pose_array2.begin(), spot_pose_array2.end());
            // arm
            std::reverse(cp.begin(), cp.end());
            Curves::BSpline spine2_arm(cp, 4);
            pose1 = Conversions::KDL_2Transform(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->sixDpose);
            gait_solver->leg_planner_solver->arm_planner_solver->BSplinePlanner(spine2_arm, home_pose, num + static_num, pose1);
            arm_ik_array.insert(arm_ik_array.end(), gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.begin(),
                                gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.end());
        }
        else if (str == "RL")
        {
            // 1. rl 2. pitch 3. trans 4. swing_leg => 4,2,3 parallel
            ll_cp = gait_solver->GenerateFootCP_ForCrawlGait(stride_s, stride_h, 0, "SN");
            rl_cp = GenerateFootCP_ForBowDown(x_shift, stride_h / 2.0, y_shift, 3);
            // no com
            // swing->r
            vector<int> sequence = {1, 2, 3, 0};
            gait_solver->leg_planner_solver->body_pose_needs_to_computed = true;
            for (int i = 0; i < 50; i++)
                t1wrt0.push_back(Kinematics::kinematics_solver->T0wrtW_inverse * Conversions::Pair_2Transform(make_pair(Kinematics::state_tracker->body_xyz, Kinematics::state_tracker->body_rpy)));
            HelpingFunForBowDown(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b,
                                 com_array, ans, 50, spot_pose_array, ll_cp, sequence, t1wrt0);
            t1wrt0.clear();

            int num1 = ans.size();

            pitch_ik = gait_solver->leg_planner_solver->PerformRPY(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, rpy_vector, seq_vector, com_array, spot_pose_array, FootHold::DoNotReset);
            ans.insert(ans.end(), pitch_ik.begin(), pitch_ik.end());
            translate_ik = gait_solver->leg_planner_solver->PerformTranslation(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, rpv_vector, com_array, spot_pose_array, FootHold::DoNotReset);
            ans.insert(ans.end(), translate_ik.begin(), translate_ik.end());

            gait_solver->leg_planner_solver->body_pose_needs_to_computed = false;
            sequence = {0, 1, 2, 3};
            vector<pair<Eigen::Vector4d, Eigen::Vector3d>> spot_pose_array3(spot_pose_array.end() - (spot_pose_array.size() - num1), spot_pose_array.end());

            for (int i = 0; i < ans.size() - num1; i++)
                t1wrt0.push_back(Kinematics::kinematics_solver->T0wrtW_inverse * Conversions::Pair_2Transform(spot_pose_array3[i]));
            HelpingFunForBowDown(gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array,
                                 ans, ans.size() - num1, spot_pose_array2, rl_cp, sequence, t1wrt0);
            int num = spot_pose_array.size();
            t1wrt0.clear();

            Eigen::Matrix4d pose1 = Conversions::KDL_2Transform(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->sixDpose);
            vector<Eigen::Vector4d> cp = gait_solver->leg_planner_solver->arm_planner_solver->GenerateBSPlineCP(goal, dummy_pose, 0.1, true);
            Curves::BSpline spine_arm(cp, 4);
            Eigen::Matrix4d home_pose = Conversions::KDL_2Transform(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->sixDpose);
            gait_solver->leg_planner_solver->arm_planner_solver->BSplinePlanner(spine_arm, goal, spot_pose_array.size(), pose1);
            arm_ik_array.insert(arm_ik_array.begin(), gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.begin(),
                                gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.end());

            for (int i = 0; i < static_num; i++)
            {
                ans.push_back(ans[ans.size() - 1]);
                spot_pose_array.push_back(spot_pose_array[spot_pose_array.size() - 1]);
                arm_ik_array.push_back(arm_ik_array[arm_ik_array.size() - 1]);
            }
            vector<vector<Eigen::Vector3d>> ans2 = ans;
            std::reverse(ans2.begin(), ans2.end());
            vector<pair<Eigen::Vector4d, Eigen::Vector3d>> spot_pose_array2 = spot_pose_array;
            std::reverse(spot_pose_array2.begin(), spot_pose_array2.end());
            ans.insert(ans.end(), ans2.begin(), ans2.end());
            spot_pose_array.insert(spot_pose_array.end(), spot_pose_array2.begin(), spot_pose_array2.end());
            // arm
            std::reverse(cp.begin(), cp.end());
            Curves::BSpline spine2_arm(cp, 4);
            pose1 = Conversions::KDL_2Transform(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->sixDpose);
            gait_solver->leg_planner_solver->arm_planner_solver->BSplinePlanner(spine2_arm, home_pose, num + static_num, pose1);
            arm_ik_array.insert(arm_ik_array.end(), gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.begin(),
                                gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.end());
        }

        *(Kinematics::state_tracker) = initial_state_tracker;
        *(Kinematics::kinematics_solver) = initial_kinematic_solver;
        gait_solver->leg_planner_solver->com_needs_to_be_computed_posture = true;
        gait_solver->leg_planner_solver->armik_needs_to_be_computed_posture = true;
        gait_solver->leg_planner_solver->body_pose_needs_to_computed = true;
        return ans;
    }

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

    void Gaits::ComputeCurrentStateCOM()
    {
        vector<Eigen::Vector3d> leg_ik = {Kinematics::state_tracker->leg_states[0].theta,
                                          Kinematics::state_tracker->leg_states[1].theta,
                                          Kinematics::state_tracker->leg_states[2].theta,
                                          Kinematics::state_tracker->leg_states[3].theta};
        current_state_com = leg_planner_solver->com_solver->ComputeWholeSystemCOM(leg_ik,
                                                                                  make_pair(Kinematics::state_tracker->body_xyz, Kinematics::state_tracker->body_rpy),
                                                                                  ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->q);
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
        leg_planner_solver->armik_needs_to_be_computed_posture = leg_planner_solver->com_needs_to_be_computed_posture = false;
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

    void Gaits::HelpingFunForDancingCrawl(const Eigen::Matrix4d &goal, vector<Eigen::Vector4d> &com_array, vector<vector<Eigen::Vector3d>> &ans, int swing_leg_index,
                                          int num, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array2, const vector<Eigen::Matrix4d> &t1wrt0)
    {
        leg_planner_solver->leg_planning_param[0].num_of_points = leg_planner_solver->leg_planning_param[1].num_of_points =
            leg_planner_solver->leg_planning_param[2].num_of_points = leg_planner_solver->leg_planning_param[3].num_of_points = num;
        leg_planner_solver->leg_planning_param[0].curve_type = leg_planner_solver->leg_planning_param[1].curve_type =
            leg_planner_solver->leg_planning_param[2].curve_type = leg_planner_solver->leg_planning_param[3].curve_type = Curves::CurveType::Bezier;
        leg_planner_solver->armik_needs_to_be_computed_posture = leg_planner_solver->com_needs_to_be_computed_posture = false;
        vector<vector<Eigen::Vector3d>> crawl_ik = leg_planner_solver->GenerateLegsMotion(goal, com_array, t1wrt0, spot_com_pose_array2);
        ReplaceColumn(ans, crawl_ik, swing_leg_index);
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
                // leg_planner_solver->arm_ik_needs_to_computed = leg_planner_solver->com_needs_to_computed = true;
                leg_planner_solver->com_needs_to_be_computed_posture = true;
                leg_planner_solver->armik_needs_to_be_computed_posture = true;
                leg_planner_solver->body_pose_needs_to_computed = true;
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
                // leg_planner_solver->arm_ik_needs_to_computed = leg_planner_solver->com_needs_to_computed = true;
                leg_planner_solver->com_needs_to_be_computed_posture = true;
                leg_planner_solver->armik_needs_to_be_computed_posture = true;
                leg_planner_solver->body_pose_needs_to_computed = true;
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
        base_circle->point_generation = "";
        Eigen::Vector4d p1 = base_circle->GenerateSinglePoint(ti);
        Eigen::Vector4d p4 = base_circle->GenerateSinglePoint(tf);
        Eigen::Vector4d p2(p1(0), p1(1), (p1(2) + stride_h), 1.0);
        Eigen::Vector4d p3(p4(0), p4(1), (p4(2) + stride_h), 1.0);
        vector<Eigen::Vector4d> ans = {p1, p2, p3, p4};
        base_circle->point_generation = "default";
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

    vector<vector<Eigen::Vector3d>> Gaits::DancingCrawlGait(const Eigen::Matrix4d &ee_goal, vector<Eigen::Vector4d> &com_array, vector<KDL::JntArray> &arm_ik, double distance, double stride_h, int step_num, double angle, const string &str, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array)
    {
        base_circle_status = false;
        leg_planner_solver->armik_needs_to_be_computed_posture = leg_planner_solver->com_needs_to_be_computed_posture = false;
        vector<vector<Eigen::Vector3d>> ans, ans2, trasition_ik;
        vector<Eigen::Vector4d> l_foot_cp[4];
        vector<int> sequence(4);
        vector<Eigen::Matrix4d> t1wrt0;
        double stride_s = distance / (4 * step_num);
        Kinematics::kinematics_solver->stance_legs_wrt_world.clear();
        vector<pair<Eigen::Vector4d, Eigen::Vector3d>> spot_com_pose_array2, spot_com_pose_array_clear;
        cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << endl;
        sequence = {1, 2, 3, 0};
        HelpingFunForTransitionPhase(ee_goal, com_array, ans, sequence, trasition_ik, base_circle, MotionStragegy::Straight, spot_com_pose_array_clear, stride_s, stride_h, 200, 0, -2, str);
        spot_com_pose_array.insert(spot_com_pose_array.begin(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
        spot_com_pose_array_clear.clear();
        sequence = {0, 2, 3, 1};
        HelpingFunForTransitionPhase(ee_goal, com_array, ans, sequence, trasition_ik, base_circle, MotionStragegy::Straight, spot_com_pose_array_clear, stride_s, stride_h, 200, 0, 2, str);
        spot_com_pose_array.insert(spot_com_pose_array.end(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
        spot_com_pose_array_clear.clear();
        // arm_ik.insert(arm_ik.begin(), leg_planner_solver->arm_planner_solver->arm_ik.begin(), leg_planner_solver->arm_planner_solver->arm_ik.end());

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
        Eigen::Vector3d rpy;

        for (int i = 0; i < (4 * step_num); i++) //(4 * step_num)
        {
            // compute only leg and arm state
            leg_planner_solver->com_needs_to_be_computed_posture = false;
            cout << "pitch " << Kinematics::state_tracker->body_rpy(1) << endl;
            if (i % 2 == 0)
                rpy << 0, R2D * Kinematics::state_tracker->body_rpy(1), angle;
            else
                rpy << 0, R2D * Kinematics::state_tracker->body_rpy(1), -angle;
            cout << "input " << rpy.adjoint() << endl;

            vector<vector<Eigen::Vector3d>> crawl_ik = leg_planner_solver->GenerateRPYTranslation(rpy, vector<int>{0, 1, 2}, rpv, spot_com_pose_array_clear, FootHold::Horizontal);
            for (int i = 0; i < spot_com_pose_array_clear.size(); i++)
                t1wrt0.push_back(Kinematics::kinematics_solver->T0wrtW_inverse * Conversions::Pair_2Transform(spot_com_pose_array_clear[i]));
            spot_com_pose_array.insert(spot_com_pose_array.end(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
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
            HelpingFunForDancingCrawl(leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array, ans, sequence[3], crawl_ik.size(), spot_com_pose_array_clear, t1wrt0);
            GenerateStanceLegPositions(sequence, crawl_ik, base_circle);
            spot_com_pose_array_clear.clear();
            t1wrt0.clear();
            leg_planner_solver->com_needs_to_be_computed_posture = true;
            leg_planner_solver->armik_needs_to_be_computed_posture = true;
            leg_planner_solver->body_pose_needs_to_computed = true;
        }

        sequence = {1, 2, 3, 0};
        HelpingFunForTransitionPhase(leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array, ans, sequence, trasition_ik, base_circle, MotionStragegy::Straight, spot_com_pose_array_clear, stride_s, stride_h, 200, 0, 2, str);
        spot_com_pose_array.insert(spot_com_pose_array.end(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
        spot_com_pose_array_clear.clear();
        sequence = {0, 2, 3, 1};
        HelpingFunForTransitionPhase(leg_planner_solver->arm_planner_solver->T_e_wrt_b, com_array, ans, sequence, trasition_ik, base_circle, MotionStragegy::Straight, spot_com_pose_array_clear, stride_s, stride_h, 200, 0, -2, str);
        spot_com_pose_array.insert(spot_com_pose_array.end(), spot_com_pose_array_clear.begin(), spot_com_pose_array_clear.end());
        spot_com_pose_array_clear.clear();

        leg_planner_solver->arm_planner_solver->arm_ik.clear();
        leg_planner_solver->arm_planner_solver->ChickenHeadPlanner(ee_goal, spot_com_pose_array);
        arm_ik.insert(arm_ik.end(), leg_planner_solver->arm_planner_solver->arm_ik.begin(), leg_planner_solver->arm_planner_solver->arm_ik.end());

        leg_planner_solver->armik_needs_to_be_computed_posture = leg_planner_solver->com_needs_to_be_computed_posture = true;
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

        ComputeCurrentStateCOM();
        cout << "------------------------------------------------------------------------------------" << endl;
        crawl_math_solver.BestSupportPolygon(current_state_com);
        cout << "------------------------------------------------------------------------------------" << endl;

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
            // leg_planner_solver->arm_ik_needs_to_computed = leg_planner_solver->com_needs_to_computed = true;
            leg_planner_solver->com_needs_to_be_computed_posture = true;
            leg_planner_solver->armik_needs_to_be_computed_posture = true;
            leg_planner_solver->body_pose_needs_to_computed = true;
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

        double theta = Kinematics::state_tracker->body_rpy(2);
        Eigen::Vector3d r_cap(cos(theta), sin(theta), 0.0);
        double x = (-stride_s * cos(theta));
        double y = (-stride_s * sin(theta));
        Eigen::Vector4d rpv(x, y, 0.0, 1.0);
        vector<Eigen::Vector4d> rpv_vector = {rpv};

        for (int i = 0; i < (4 * step_num); i++)
        {
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
            // leg_planner_solver->arm_ik_needs_to_computed = leg_planner_solver->com_needs_to_computed = true;
            leg_planner_solver->com_needs_to_be_computed_posture = true;
            leg_planner_solver->armik_needs_to_be_computed_posture = true;
            leg_planner_solver->body_pose_needs_to_computed = true;
        }
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
        return ans;
    }
}

#endif
