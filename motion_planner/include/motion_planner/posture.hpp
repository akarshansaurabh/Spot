// #ifndef POSTURE_HPP
// #define POSTURE_HPP

// #include "maths/commonmathssolver.hpp"
// #include "kinematics/kinematics.hpp"
// #include "com/com_solver.hpp"

// #include <eigen3/Eigen/Dense>
// #include <iostream>
// #include <vector>

// using namespace std;

// const double D2R = 0.01745329;

// namespace PostureControl
// {
//     enum class FootHold
//     {
//         Reset,
//         DoNotReset,
//         DoNotResetButComputeStates
//     };

//     class PostureControl
//     {
//     private:
//         vector<vector<Eigen::Vector3d>> Generate_SpotPath_ForSingleRPY(Eigen::Vector3d &rpy, const vector<int> &seq, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_pose_array, bool b1, const FootHold &foot_hold);
//         vector<vector<Eigen::Vector3d>> Generate_SpotPath_ForSingleTranslation(Eigen::Vector4d &rp, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_pose_array, const FootHold &foot_hold);
//         vector<vector<Eigen::Vector3d>> Generate_SpotPath_For2AxisBodyRotation(double t, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_pose_array, bool b1);

//     protected:
//         double angle_precision, displacement_precision;
//         // std::unique_ptr<COM::WholeSystemCOM> com_solver;
//         vector<vector<Eigen::Vector3d>> GenerateJointSpaceLegsPath();
//         vector<Eigen::Vector4d> l1_path_wrt_o, l2_path_wrt_o, l3_path_wrt_o, l4_path_wrt_o;

//     public:
//         PostureControl(double angle_precision_, double displacement_precision_);
//         vector<vector<Eigen::Vector3d>> PerformRPY(const vector<Eigen::Vector3d> &rpy_vector, const vector<vector<int>> &seq_vector, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array, bool b1, const FootHold &foot_hold);
//         vector<vector<Eigen::Vector3d>> Perform2AxisBodyRotation(double t, int num, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array, bool b1);
//         vector<vector<Eigen::Vector3d>> PerformTranslation(const vector<Eigen::Vector4d> &rp_vector, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array, const FootHold &foot_hold);
//         vector<vector<Eigen::Vector3d>> PerformCircles(double radius, int rev_num, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array);
//     };

//     PostureControl::PostureControl(double angle_precision_, double displacement_precision_) : displacement_precision(displacement_precision_)
//     {
//         angle_precision = D2R * angle_precision_;
//         // com_solver = std::make_unique<COM::WholeSystemCOM>();
//         // legs_com_solver = std::make_unique<COM::LeggedSystemCOM>();
//     }

//     vector<vector<Eigen::Vector3d>> PostureControl::Generate_SpotPath_ForSingleTranslation(Eigen::Vector4d &rp, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_pose_array, const FootHold &foot_hold)
//     {
//         vector<vector<Eigen::Vector3d>> ans;
//         l1_path_wrt_o.clear();
//         l2_path_wrt_o.clear();
//         l3_path_wrt_o.clear();
//         l4_path_wrt_o.clear();

//         // 1 wrt w
//         Eigen::Vector3d initial_rpy = Kinematics::state_tracker->body_rpy;
//         Eigen::Vector4d initial_pv = Kinematics::state_tracker->body_xyz;
//         Eigen::Vector3d rp_;
//         rp_ << rp(0), rp(1), rp(2);

//         int num;
//         Eigen::Vector3d direction_ = rp_ / rp_.norm();
//         Eigen::Vector4d direction, target_pv;
//         direction << direction_(0), direction_(1), direction_(2), 0.0;
//         double displacement_precision_;

//         Eigen::Matrix3d initial_rot = CommonMathsSolver::OrientationNTransformaton::ComputeR(initial_rpy);

//         num = rp_.norm() / displacement_precision;
//         displacement_precision_ = rp_.norm() / num;
//         vector<Eigen::Vector4d> stance_legs_single_state;
//         stance_legs_single_state.resize(4);
//         // if (foot_hold == FootHold::Reset)
//         //     Kinematics::kinematics_solver->stance_legs_wrt_world.clear();
//         for (int j = 0; j < num; j++)
//         {
//             target_pv = initial_pv + ((j + 1) * displacement_precision_ * direction);
//             Eigen::Matrix4d T1wrtW;
//             T1wrtW.block<3, 3>(0, 0) = initial_rot;
//             T1wrtW.block<4, 1>(0, 3) = target_pv;
//             T1wrtW.block<1, 3>(3, 0).setZero();
//             Kinematics::kinematics_solver->T1wrt0 = Kinematics::kinematics_solver->T0wrtW.inverse() * T1wrtW;
//             l1_path_wrt_o.push_back(Kinematics::kinematics_solver->T1wrt0.inverse() * Kinematics::kinematics_solver->legs[0].xyz); // legs[0].xyz  -> wrt 0
//             l2_path_wrt_o.push_back(Kinematics::kinematics_solver->T1wrt0.inverse() * Kinematics::kinematics_solver->legs[1].xyz);
//             l3_path_wrt_o.push_back(Kinematics::kinematics_solver->T1wrt0.inverse() * Kinematics::kinematics_solver->legs[2].xyz);
//             l4_path_wrt_o.push_back(Kinematics::kinematics_solver->T1wrt0.inverse() * Kinematics::kinematics_solver->legs[3].xyz);
//             if (foot_hold == FootHold::Reset || foot_hold == FootHold::DoNotResetButComputeStates)
//             {
//                 for (int k = 0; k < 4; k++)
//                     stance_legs_single_state[k] = Kinematics::kinematics_solver->T0wrtW * Kinematics::kinematics_solver->legs[k].xyz;
//                 Kinematics::kinematics_solver->stance_legs_wrt_world.push_back(stance_legs_single_state);
//             }
//             spot_pose_array.push_back(make_pair(target_pv, initial_rpy));
//         }
//         Kinematics::state_tracker->body_xyz = target_pv;

//         vector<vector<Eigen::Vector3d>> ik = GenerateJointSpaceLegsPath();
//         if (ans.size() == 0)
//             ans.insert(ans.begin(), ik.begin(), ik.end());
//         else
//             ans.insert(ans.end(), ik.begin(), ik.end());
//         return ans;
//     }

//     // rpy is absolute, not relative
//     vector<vector<Eigen::Vector3d>> PostureControl::Generate_SpotPath_ForSingleRPY(Eigen::Vector3d &rpy, const vector<int> &seq, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_pose_array, bool b1, const FootHold &foot_hold)
//     {
//         double d_rpy[3], angle_precision_[3];
//         int num[3];

//         vector<vector<Eigen::Vector3d>> ans;
//         Eigen::Vector3d initial_rpy = Kinematics::state_tracker->body_rpy;
//         Eigen::Vector3d target_rpy = initial_rpy;

//         if (b1)
//         {
//             rpy(0) = (D2R * rpy(0)) + initial_rpy(0);
//             rpy(1) = (D2R * rpy(1)) + initial_rpy(1);
//             rpy(2) = (D2R * rpy(2)) + initial_rpy(2);
//         }
//         else // my case
//         {
//             rpy(0) = D2R * rpy(0);
//             rpy(1) = D2R * rpy(1);
//             rpy(2) = D2R * rpy(2);
//         }

//         for (int i = 0; i < 3; i++)
//         {
//             d_rpy[i] = rpy(i) - initial_rpy(i);
//             num[i] = fabs(d_rpy[i]) / angle_precision;
//             if (num[i] > 0)
//                 angle_precision_[i] = (d_rpy[i]) / num[i];
//         }

//         vector<Eigen::Vector4d> stance_legs_single_state;
//         stance_legs_single_state.resize(4);
//         // Kinematics::kinematics_solver->stance_legs_wrt_world.clear();
//         for (int i = 0; i < 3; i++)
//         {
//             l1_path_wrt_o.clear();
//             l2_path_wrt_o.clear();
//             l3_path_wrt_o.clear();
//             l4_path_wrt_o.clear();
//             for (int j = 0; j < num[seq[i]]; j++)
//             {
//                 target_rpy(seq[i]) = initial_rpy(seq[i]) + ((j + 1) * angle_precision_[seq[i]]);
//                 Eigen::Matrix3d target_rot = CommonMathsSolver::OrientationNTransformaton::ComputeR(target_rpy);
//                 // 1 wrt 0
//                 Kinematics::kinematics_solver->T1wrt0 << target_rot(0, 0), target_rot(0, 1), target_rot(0, 2), Kinematics::kinematics_solver->T1wrt0(0, 3),
//                     target_rot(1, 0), target_rot(1, 1), target_rot(1, 2), Kinematics::kinematics_solver->T1wrt0(1, 3),
//                     target_rot(2, 0), target_rot(2, 1), target_rot(2, 2), Kinematics::kinematics_solver->T1wrt0(2, 3),
//                     0, 0, 0, 1;

//                 l1_path_wrt_o.push_back(Kinematics::kinematics_solver->T1wrt0.inverse() * Kinematics::kinematics_solver->legs[0].xyz); // legs[0].xyz  -> wrt 0
//                 l2_path_wrt_o.push_back(Kinematics::kinematics_solver->T1wrt0.inverse() * Kinematics::kinematics_solver->legs[1].xyz);
//                 l3_path_wrt_o.push_back(Kinematics::kinematics_solver->T1wrt0.inverse() * Kinematics::kinematics_solver->legs[2].xyz);
//                 l4_path_wrt_o.push_back(Kinematics::kinematics_solver->T1wrt0.inverse() * Kinematics::kinematics_solver->legs[3].xyz);
//                 if (foot_hold == FootHold::Reset || foot_hold == FootHold::DoNotResetButComputeStates)
//                 {
//                     for (int k = 0; k < 4; k++)
//                         stance_legs_single_state[k] = Kinematics::kinematics_solver->T0wrtW * Kinematics::kinematics_solver->legs[k].xyz;
//                     Kinematics::kinematics_solver->stance_legs_wrt_world.push_back(stance_legs_single_state);
//                 }
//                 spot_pose_array.push_back(make_pair(Kinematics::state_tracker->body_xyz, target_rpy));
//             }
//             // transformation of 1 wrt 0
//             Kinematics::state_tracker->body_rpy = target_rpy;
//             // Kinematics::state_tracker->body_xyz = Kinematics::kinematics_solver->T1wrt0.block<4, 1>(0, 3);
//             vector<vector<Eigen::Vector3d>> ik = GenerateJointSpaceLegsPath();
//             if (ans.size() == 0)
//                 ans.insert(ans.begin(), ik.begin(), ik.end());
//             else
//                 ans.insert(ans.end(), ik.begin(), ik.end());
//         }
//         return ans;
//     }

//     vector<vector<Eigen::Vector3d>> PostureControl::Generate_SpotPath_For2AxisBodyRotation(double t, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_pose_array, bool b1)
//     {
//         Eigen::Vector3d rpy(0, t, 0);
//         vector<int> seq = {1, 0, 0};
//         vector<Eigen::Vector3d> rpy_vector = {rpy};
//         vector<vector<int>> seq_vector = {seq};
//         vector<vector<Eigen::Vector3d>> pitch_path = PostureControl::PerformRPY(rpy_vector, seq_vector, spot_pose_array, b1, FootHold::DoNotResetButComputeStates);
//         vector<vector<Eigen::Vector3d>> ans(pitch_path);
//         seq.clear();

//         for (double i = 1.0; i <= 360; i++)
//         {
//             rpy_vector.clear();
//             seq_vector.clear();
//             rpy << 0, t * cos(D2R * i), t * sin(D2R * i);
//             seq.push_back(0);
//             seq.push_back(1);
//             seq.push_back(2);
//             rpy_vector.push_back(rpy);
//             seq_vector.push_back(seq);
//             vector<vector<Eigen::Vector3d>> path = PostureControl::PerformRPY(rpy_vector, seq_vector, spot_pose_array, b1, FootHold::DoNotResetButComputeStates);
//             ans.insert(ans.end(), path.begin(), path.end());
//         }
//         return ans;
//     }

//     // array of l1,l2,l3,l4
//     vector<vector<Eigen::Vector3d>> PostureControl::GenerateJointSpaceLegsPath()
//     {
//         vector<vector<Eigen::Vector3d>> ans;

//         int num_of_points = l1_path_wrt_o.size();
//         for (int i = 0; i < num_of_points; i++)
//             ans.push_back(Kinematics::kinematics_solver->SolveIK(l1_path_wrt_o[i], l2_path_wrt_o[i], l3_path_wrt_o[i], l4_path_wrt_o[i]));

//         return ans;
//     }
//     vector<vector<Eigen::Vector3d>> PostureControl::PerformRPY(const vector<Eigen::Vector3d> &rpy_vector, const vector<vector<int>> &seq_vector, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array, bool b1, const FootHold &foot_hold)
//     {
//         vector<vector<Eigen::Vector3d>> spot_legs_path_array;
//         vector<vector<Eigen::Vector3d>> spot_leg_path_array_for_single_rpy[rpy_vector.size()];
//         if (foot_hold == FootHold::Reset)
//             Kinematics::kinematics_solver->stance_legs_wrt_world.clear();
//         for (int i = 0; i < rpy_vector.size(); i++)
//         {
//             Eigen::Vector3d rpy = rpy_vector[i];
//             vector<int> seq = seq_vector[i];
//             spot_leg_path_array_for_single_rpy[i] = Generate_SpotPath_ForSingleRPY(rpy, seq, spot_com_pose_array, b1, foot_hold);
//             if (i == 0)
//                 spot_legs_path_array.insert(spot_legs_path_array.begin(), spot_leg_path_array_for_single_rpy[i].begin(), spot_leg_path_array_for_single_rpy[i].end());
//             else
//                 spot_legs_path_array.insert(spot_legs_path_array.end(), spot_leg_path_array_for_single_rpy[i].begin(), spot_leg_path_array_for_single_rpy[i].end());
//         }
//         return spot_legs_path_array;
//     }
//     vector<vector<Eigen::Vector3d>> PostureControl::PerformTranslation(const vector<Eigen::Vector4d> &rp_vector, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array, const FootHold &foot_hold)
//     {
//         vector<vector<Eigen::Vector3d>> spot_legs_path_array;
//         vector<vector<Eigen::Vector3d>> spot_leg_path_array_for_single_translation[rp_vector.size()];
//         if (foot_hold == FootHold::Reset)
//             Kinematics::kinematics_solver->stance_legs_wrt_world.clear();
//         for (int i = 0; i < rp_vector.size(); i++)
//         {
//             Eigen::Vector4d rp = rp_vector[i];
//             spot_leg_path_array_for_single_translation[i] = Generate_SpotPath_ForSingleTranslation(rp, spot_com_pose_array, foot_hold);
//             if (i == 0)
//                 spot_legs_path_array.insert(spot_legs_path_array.begin(), spot_leg_path_array_for_single_translation[i].begin(), spot_leg_path_array_for_single_translation[i].end());
//             else
//                 spot_legs_path_array.insert(spot_legs_path_array.end(), spot_leg_path_array_for_single_translation[i].begin(), spot_leg_path_array_for_single_translation[i].end());
//         }
//         return spot_legs_path_array;
//     }

//     vector<vector<Eigen::Vector3d>> PostureControl::Perform2AxisBodyRotation(double t, int num, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array, bool b1)
//     {
//         Kinematics::kinematics_solver->stance_legs_wrt_world.clear();
//         vector<vector<Eigen::Vector3d>> ans;
//         for (int i = 0; i < num; i++)
//         {
//             vector<vector<Eigen::Vector3d>> partial_ans = PostureControl::Generate_SpotPath_For2AxisBodyRotation(t, spot_com_pose_array, b1);
//             if (i == 0)
//                 ans.insert(ans.begin(), partial_ans.begin(), partial_ans.end());
//             else
//                 ans.insert(ans.end(), partial_ans.begin(), partial_ans.end());
//         }
//         return ans;
//     }

//     vector<vector<Eigen::Vector3d>> PostureControl::PerformCircles(double radius, int rev_num, vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_com_pose_array)
//     {
//         vector<vector<Eigen::Vector3d>> ans;
//         // yz
//         Eigen::Vector3d initial_rpy = Kinematics::state_tracker->body_rpy;
//         Eigen::Vector4d initial_xyz = Kinematics::state_tracker->body_xyz, current_xyz;
//         Eigen::Vector3d center(initial_xyz(0), initial_xyz(1), initial_xyz(2) + radius);
//         Eigen::Matrix3d initial_rot = CommonMathsSolver::OrientationNTransformaton::ComputeR(initial_rpy);
//         vector<Eigen::Vector4d> stance_legs_single_state;
//         stance_legs_single_state.resize(4);
//         Kinematics::kinematics_solver->stance_legs_wrt_world.clear();
//         for (int i = 0; i < rev_num; i++)
//         {
//             l1_path_wrt_o.clear();
//             l2_path_wrt_o.clear();
//             l3_path_wrt_o.clear();
//             l4_path_wrt_o.clear();
//             for (double t = -PI / 2; t <= 1.5 * PI; t += angle_precision)
//             {
//                 current_xyz << center(0), (center(1) + (radius * cos(t))), (center(2) + (radius * sin(t))), 1.0;
//                 Eigen::Matrix4d T1wrtW;
//                 T1wrtW.block<3, 3>(0, 0) = initial_rot;
//                 T1wrtW.block<4, 1>(0, 3) = current_xyz;
//                 T1wrtW.block<1, 3>(3, 0).setZero();
//                 Kinematics::kinematics_solver->T1wrt0 = Kinematics::kinematics_solver->T0wrtW.inverse() * T1wrtW;
//                 l1_path_wrt_o.push_back(Kinematics::kinematics_solver->T1wrt0.inverse() * Kinematics::kinematics_solver->legs[0].xyz); // legs[0].xyz  -> wrt 1
//                 l2_path_wrt_o.push_back(Kinematics::kinematics_solver->T1wrt0.inverse() * Kinematics::kinematics_solver->legs[1].xyz);
//                 l3_path_wrt_o.push_back(Kinematics::kinematics_solver->T1wrt0.inverse() * Kinematics::kinematics_solver->legs[2].xyz);
//                 l4_path_wrt_o.push_back(Kinematics::kinematics_solver->T1wrt0.inverse() * Kinematics::kinematics_solver->legs[3].xyz);
//                 for (int k = 0; k < 4; k++)
//                     stance_legs_single_state[k] = Kinematics::kinematics_solver->T0wrtW * Kinematics::kinematics_solver->legs[k].xyz;
//                 Kinematics::kinematics_solver->stance_legs_wrt_world.push_back(stance_legs_single_state);
//                 spot_com_pose_array.push_back(make_pair(current_xyz, initial_rpy));
//             }
//             Kinematics::state_tracker->body_xyz = current_xyz;

//             vector<vector<Eigen::Vector3d>> ik = GenerateJointSpaceLegsPath();
//             if (ans.size() == 0)
//                 ans.insert(ans.begin(), ik.begin(), ik.end());
//             else
//                 ans.insert(ans.end(), ik.begin(), ik.end());
//         }
//         return ans;
//     }
// }

// #endif