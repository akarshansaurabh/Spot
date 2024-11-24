#include "otg.h"
#include <fstream>
#include <cmath>
#include <algorithm>

using namespace std;

namespace OwlOtg
{
    double control_cycle;
    vector<double> a_min, v_min;
    vector<bool> plot_data, state_is_generated;
    vector<double> trajectory_time, user_time_tracker, joint_variant_b_time, joint_variant_a_time;
    vector<int> global_dt_counter, no_sol, variant_b_index, num_of_segments;

    vector<vector<Eigen::Vector4d>> invalid_profile; // need to think, some joints can be invalid, some can be valid
    vector<vector<ValidProfileCode>> profile_sequence;
    vector<vector<Eigen::Vector4d>> profile_matrix, n_profiles;  // vector of profile_point
    vector<Eigen::Vector4d> profile_point, single_joint_profile; // profile_point is my desired output of otg
    vector<vector<double>> zero_time;
    vector<double> t0;

    // when i have to generate single state, i will resize profile_point
    // when i have to generate entite profile, i will resize
}

using namespace OwlOtg;

template <int DOF>
ProfileCode OwlOtg::ProfileBase<DOF>::TargetStateAchieved(const Eigen::Matrix<double, DOF, 3> &target_state,
                                                          double xt, double vt, double at, int joint_index)
{
    if (fabs(xt - target_state(joint_index, 0)) > 0.05 || fabs(vt - target_state(joint_index, 1)) > 0.05 || fabs(at - target_state(joint_index, 2)) > 0.05)
    {
        no_sol[joint_index]++;
        return ProfileCode::Profile16;
        cout << "absolute pos error = " << fabs(xt - target_state(joint_index, 0)) << " absolute vel error = " << fabs(vt - target_state(joint_index, 1)) << " absolute acc error = " << fabs(at - target_state(joint_index, 2)) << endl;
    }
}

template <int DOF>
void OwlOtg::ProfileBase<DOF>::ComputeNextState(double time_of_intererst, double zero_time, int joint_index)
{
    // cout << "time_of_intererst = " << time_of_intererst << endl;
    // state_is_generated = true;
    double xt, vt, at, t = time_of_intererst - zero_time;
    HelpingFun2Gen_State(xt, vt, at, t, joint_index);
    Eigen::Vector4d single_point;
    single_point << xt, vt, at, time_of_intererst;
    profile_point[joint_index] = single_point;
    std::ofstream log_file("/home/akarshan/OTG_2/owl_otg/one_dof_otg/text_files/plot_data.txt",
                           std::ios::out | std::ios::app);
    if (plot_data[joint_index])
    {
        // if (joint_index < 5)
        //     log_file << xt << " " << vt << " " << at << " " << time_of_intererst << " ";
        // else
        //     log_file << endl;
        log_file << xt << " " << vt << " " << at << " " << time_of_intererst << " " << endl;
    }
}

template <int DOF>
OwlOtg::OwlOtg1dof<DOF>::OwlOtg1dof(double control_frequency, vector<double> v_min_, vector<double> a_min_, vector<bool> plot_data_)
{
    control_cycle = control_frequency;

    t0.resize(DOF);
    no_sol.resize(DOF);
    zero_time.resize(DOF);
    trajectory_time.resize(DOF);
    user_time_tracker.resize(DOF);
    state_is_generated.resize(DOF);
    variant_b_index.resize(DOF);
    global_dt_counter.resize(DOF);
    profile_sequence.resize(DOF);
    profile_point.resize(DOF);
    n_profiles.resize(DOF);
    num_of_segments.resize(DOF);
    joint_variant_b_time.resize(DOF);
    joint_variant_a_time.resize(DOF);

    for (int i = 0; i < DOF; i++)
    {
        user_time_tracker[i] = 0.0;
        global_dt_counter[i] = 0;
        t0[i] = 0.0;
        variant_b_index[i] = -1;
        no_sol[i] = 0;
        state_is_generated[i] = false;
        trajectory_time[i] = control_frequency;
        zero_time[i].push_back(0.0);
        joint_variant_b_time[i] = 0.0;
        joint_variant_a_time[i] = 0.0;

        a_min.push_back(a_min_[i]);
        v_min.push_back(v_min_[i]);
        plot_data.push_back(plot_data_[i]);
    }

    profiles[make_pair(ProfileCode::Profile9, 0)] = make_unique<Profile9<DOF>>();
    profiles[make_pair(ProfileCode::Profile10, 0)] = make_unique<Profile10<DOF>>();
    profiles[make_pair(ProfileCode::Profile11, 0)] = make_unique<Profile11<DOF>>();
    profiles[make_pair(ProfileCode::Profile12, 0)] = make_unique<Profile12<DOF>>();
    profiles[make_pair(ProfileCode::Profile13, 0)] = make_unique<Profile13<DOF>>();
    profiles[make_pair(ProfileCode::Profile14, 0)] = make_unique<Profile14<DOF>>();
    profiles[make_pair(ProfileCode::Profile15, 0)] = make_unique<Profile15<DOF>>();
    profiles[make_pair(ProfileCode::Profile16, 0)] = make_unique<Profile16<DOF>>();
    for (int i = 0; i < 100; i++)
    {
        profiles[make_pair(ProfileCode::PositiveVariantB, i)] = make_unique<PositiveVarientB<DOF>>();
        profiles[make_pair(ProfileCode::NegativeVariantB, i)] = make_unique<NegativeVarientB<DOF>>();
    }
}

template <int DOF>
void OwlOtg::OwlOtg1dof<DOF>::Reset_JointTime(int joint_index)
{
    user_time_tracker[joint_index] = 0.0;
    global_dt_counter[joint_index] = 0;
    t0[joint_index] = 0.0;
    variant_b_index[joint_index] = -1;
    no_sol[joint_index] = 0;
    state_is_generated[joint_index] = false;
    zero_time[joint_index].clear();
    zero_time[joint_index].push_back(0.0);
    profile_sequence[joint_index].clear();

    for (int i = 0; i < 100; i++)
    {
        auto &neg_varB_pointer = profiles[make_pair(ProfileCode::NegativeVariantB, i)];
        neg_varB_pointer->tbd(joint_index, 0) = 0.0;
        neg_varB_pointer->tba(joint_index, 0) = 0.0;
        neg_varB_pointer->tbc(joint_index, 0) = 0.0;

        auto &pos_varB_pointer = profiles[make_pair(ProfileCode::PositiveVariantB, i)];
        pos_varB_pointer->tbd(joint_index, 0) = 0.0;
        pos_varB_pointer->tba(joint_index, 0) = 0.0;
        pos_varB_pointer->tbc(joint_index, 0) = 0.0;
    }
}

template <int DOF>
void OwlOtg::OwlOtg1dof<DOF>::FuncForLastProfile_SingleJoint(ProfileCode &expr,
                                                             Eigen::Matrix<double, DOF, 3> &current_state,
                                                             const Eigen::Matrix<double, DOF, 3> &target_state,
                                                             Eigen::Matrix<double, DOF, 3> &boundary_condition, int joint_index)
{
    ProfileCode profile_generator_output;
    switch (expr)
    {
    case ProfileCode::PositiveVariantB:
        DecisionTree_1DOF(current_state, target_state, boundary_condition, joint_index);
        break;
    case ProfileCode::NegativeVariantB:
        DecisionTree_1DOF(current_state, target_state, boundary_condition, joint_index);
        break;
    }
}

template <int DOF>
void OwlOtg::OwlOtg1dof<DOF>::FuncForBothRows_SingleJoint(ProfileCode &expr,
                                                          Eigen::Matrix<double, DOF, 3> &current_state,
                                                          const Eigen::Matrix<double, DOF, 3> &target_state,
                                                          Eigen::Matrix<double, DOF, 3> &boundary_condition, int joint_index)
{
    ProfileCode profile_generator_output;
    switch (expr)
    {
    case ProfileCode::Profile12:
        profile_generator_output = profiles[make_pair(ProfileCode::Profile12, 0)]->ComputeSegmentTrajectoryTimes(current_state, target_state, boundary_condition, joint_index);
        FuncForLastProfile_SingleJoint(profile_generator_output, current_state, target_state, boundary_condition, joint_index);
        break;
    case ProfileCode::Profile16:
        profile_generator_output = profiles[make_pair(ProfileCode::Profile16, 0)]->ComputeSegmentTrajectoryTimes(current_state, target_state, boundary_condition, joint_index);
        FuncForLastProfile_SingleJoint(profile_generator_output, current_state, target_state, boundary_condition, joint_index);
        break;
    case ProfileCode::PositiveVariantB:
        DecisionTree_1DOF(current_state, target_state, boundary_condition, joint_index);
        break;
    case ProfileCode::NegativeVariantB:
        DecisionTree_1DOF(current_state, target_state, boundary_condition, joint_index);
        break;
    }
}

template <int DOF>
void OwlOtg::OwlOtg1dof<DOF>::DecisionTree_1DOF(Eigen::Matrix<double, DOF, 3> &current_state,
                                                const Eigen::Matrix<double, DOF, 3> &target_state,
                                                Eigen::Matrix<double, DOF, 3> &boundary_condition, int joint_index)
{
    global_dt_counter[joint_index]++;

    ProfileCode profile_generator_output, profile_generator_output_;

    if (target_state == current_state)
    {
        // delete
        Eigen::Vector4d state(target_state(joint_index, 0), target_state(joint_index, 1), target_state(joint_index, 2), (t0[joint_index] + control_cycle));
        profile_point[joint_index] = state;
    }
    else
    {
        // cout << "hhhhhh" << endl;
        if (current_state(joint_index, 1) > boundary_condition(joint_index, 0))
        {
            variant_b_index[joint_index]++;
            // cout << "DT: +B" << endl;
            profile_generator_output = profiles[make_pair(ProfileCode::PositiveVariantB, variant_b_index[joint_index])]->ComputeSegmentTrajectoryTimes(current_state, target_state, boundary_condition, joint_index);
        }
        else if (current_state(joint_index, 1) < -boundary_condition(joint_index, 0))
        {
            // cout << "DT: -B" << endl;
            variant_b_index[joint_index]++;
            profile_generator_output = profiles[make_pair(ProfileCode::NegativeVariantB, variant_b_index[joint_index])]->ComputeSegmentTrajectoryTimes(current_state, target_state, boundary_condition, joint_index);
        }
        if ((current_state(0) < target_state(joint_index, 0)) || ((current_state(0) == target_state(joint_index, 0)) && current_state(2) >= 0.0))
        {
            profile_generator_output = profiles[make_pair(ProfileCode::Profile9, 0)]->ComputeSegmentTrajectoryTimes(current_state, target_state, boundary_condition, joint_index);
            switch (profile_generator_output)
            {
            case ProfileCode::ValidProfile:
                break;
            case ProfileCode::Profile10:
                profile_generator_output_ = profiles[make_pair(ProfileCode::Profile10, 0)]->ComputeSegmentTrajectoryTimes(current_state, target_state, boundary_condition, joint_index);
                FuncForBothRows_SingleJoint(profile_generator_output_, current_state, target_state, boundary_condition, joint_index);
                break;
            case ProfileCode::Profile11:
                profile_generator_output_ = profiles[make_pair(ProfileCode::Profile11, 0)]->ComputeSegmentTrajectoryTimes(current_state, target_state, boundary_condition, joint_index);
                FuncForBothRows_SingleJoint(profile_generator_output_, current_state, target_state, boundary_condition, joint_index);
                break;
            case ProfileCode::Profile12:
                profile_generator_output_ = profiles[make_pair(ProfileCode::Profile12, 0)]->ComputeSegmentTrajectoryTimes(current_state, target_state, boundary_condition, joint_index);
                FuncForLastProfile_SingleJoint(profile_generator_output_, current_state, target_state, boundary_condition, joint_index);
                break;
            case ProfileCode::PositiveVariantB:
                DecisionTree_1DOF(current_state, target_state, boundary_condition, joint_index);
                break;
            case ProfileCode::NegativeVariantB:
                DecisionTree_1DOF(current_state, target_state, boundary_condition, joint_index);
                break;
            }
        }
        else if (current_state(0) > target_state(joint_index, 0) || ((current_state(0) == target_state(joint_index, 0)) && current_state(2) <= 0.0)) //
        {
            profile_generator_output = profiles[make_pair(ProfileCode::Profile13, 0)]->ComputeSegmentTrajectoryTimes(current_state, target_state, boundary_condition, joint_index);
            switch (profile_generator_output)
            {
            case ProfileCode::ValidProfile:
                break;
            case ProfileCode::Profile14:
                profile_generator_output_ = profiles[make_pair(ProfileCode::Profile14, 0)]->ComputeSegmentTrajectoryTimes(current_state, target_state, boundary_condition, joint_index);
                FuncForBothRows_SingleJoint(profile_generator_output_, current_state, target_state, boundary_condition, joint_index);
                break;
            case ProfileCode::Profile15:
                profile_generator_output_ = profiles[make_pair(ProfileCode::Profile15, 0)]->ComputeSegmentTrajectoryTimes(current_state, target_state, boundary_condition, joint_index);
                FuncForBothRows_SingleJoint(profile_generator_output_, current_state, target_state, boundary_condition, joint_index);
                break;
            case ProfileCode::Profile16:
                profile_generator_output_ = profiles[make_pair(ProfileCode::Profile16, 0)]->ComputeSegmentTrajectoryTimes(current_state, target_state, boundary_condition, joint_index);
                FuncForLastProfile_SingleJoint(profile_generator_output_, current_state, target_state, boundary_condition, joint_index);
                break;
            case ProfileCode::PositiveVariantB:
                DecisionTree_1DOF(current_state, target_state, boundary_condition, joint_index);
                break;
            case ProfileCode::NegativeVariantB:
                DecisionTree_1DOF(current_state, target_state, boundary_condition, joint_index);
                break;
            }
        }
    }
}

template <int DOF>
void OwlOtg::OwlOtg1dof<DOF>::Generate_JointProfile(Eigen::Matrix<double, DOF, 3> &current_state,
                                                    const Eigen::Matrix<double, DOF, 3> &target_state,
                                                    Eigen::Matrix<double, DOF, 3> &boundary_condition, int joint_index)
{
    DecisionTree_1DOF(current_state, target_state, boundary_condition, joint_index);
    int ii = -1;
    if (profile_sequence[joint_index].empty())
    {
        Eigen::Vector4d state(target_state(joint_index, 0), target_state(joint_index, 1), target_state(joint_index, 2), -1.0);
        profile_point[joint_index] = state;
    }
    else
        for (const auto &individual_profile : profile_sequence[joint_index])
        {
            ii++;
            switch (individual_profile)
            {
            case ValidProfileCode::Profile9:
                cout << "9" << endl;
                profiles[make_pair(ProfileCode::Profile9, 0)]->ComputeProfile(joint_index);
                break;
            case ValidProfileCode::Profile10:
                cout << "10" << endl;
                profiles[make_pair(ProfileCode::Profile10, 0)]->ComputeProfile(joint_index);
                break;
            case ValidProfileCode::Profile11:
                cout << "11" << endl;
                profiles[make_pair(ProfileCode::Profile11, 0)]->ComputeProfile(joint_index);
                break;
            case ValidProfileCode::Profile12:
                cout << "12" << endl;
                profiles[make_pair(ProfileCode::Profile12, 0)]->ComputeProfile(joint_index);
                break;
            case ValidProfileCode::Profile13:
                cout << "13" << endl;
                profiles[make_pair(ProfileCode::Profile13, 0)]->ComputeProfile(joint_index);
                break;
            case ValidProfileCode::Profile14:
                cout << "14" << endl;
                profiles[make_pair(ProfileCode::Profile14, 0)]->ComputeProfile(joint_index);
                break;
            case ValidProfileCode::Profile15:
                cout << "15" << endl;
                profiles[make_pair(ProfileCode::Profile15, 0)]->ComputeProfile(joint_index);
                break;
            case ValidProfileCode::Profile16:
                cout << "16" << endl;
                profiles[make_pair(ProfileCode::Profile16, 0)]->ComputeProfile(joint_index);
                break;
            case ValidProfileCode::PositiveVariantB:
                cout << "+B" << endl;
                profiles[make_pair(ProfileCode::PositiveVariantB, ii)]->ComputeProfile(joint_index);
                break;
            case ValidProfileCode::NegativeVariantB:
                cout << "-B" << endl;
                profiles[make_pair(ProfileCode::NegativeVariantB, ii)]->ComputeProfile(joint_index);
                break;
            }
        }
}

template <int DOF>
void OwlOtg::OwlOtg1dof<DOF>::Generate_JointNextState(Eigen::Matrix<double, DOF, 3> &current_state,
                                                      const Eigen::Matrix<double, DOF, 3> &target_state,
                                                      Eigen::Matrix<double, DOF, 3> &boundary_condition,
                                                      const UserInput &user_input, double time_of_interest,
                                                      int index, int joint_index)
{
    if (user_input == UserInput::ChangedVelocity)
    {
        cout << "input changed by user at t = " << time_of_interest << endl;
        // cout << "initial state before calling DT = " << current_state.adjoint() << endl;
        DecisionTree_1DOF(current_state, target_state, boundary_condition, joint_index);
        Compute_JointTotalTrajectoryTime(joint_index);
        // cout << "total trajectory time = " << trajectory_time[joint_index] << endl;
        cout << "------------------------------------------------------------------" << endl;
    }
    // time_tracker
    state_is_generated[joint_index] = false;
    // std::ofstream error_log("/home/akarshan/OTG_2/owl_otg/one_dof_otg/text_files/error_log.txt",
    //                         std::ios::out | std::ios::app);
    if (profile_sequence[joint_index].empty())
    {
        Eigen::Vector4d state(target_state(joint_index, 0), target_state(joint_index, 1), target_state(joint_index, 2), time_of_interest);
        profile_point[joint_index] = state;
    }
    else
        for (const auto &individual_profile : profile_sequence[joint_index])
        {
            index++;
            switch (individual_profile)
            {
                // user_time_tracker[joint_index] = time when user changes vm
            case ValidProfileCode::Profile9:
                // cout << "9" << endl;
                if (time_of_interest <= user_time_tracker[joint_index] + profiles[make_pair(ProfileCode::Profile9, 0)]->T(joint_index, 0) + zero_time[joint_index][index])
                    profiles[make_pair(ProfileCode::Profile9, 0)]->ComputeNextState(time_of_interest, user_time_tracker[joint_index] + zero_time[joint_index][index], joint_index);
                break;
            case ValidProfileCode::Profile10:
                // cout << "10" << endl;
                if (time_of_interest <= user_time_tracker[joint_index] + profiles[make_pair(ProfileCode::Profile10, 0)]->T(joint_index, 0) + zero_time[joint_index][index])
                    profiles[make_pair(ProfileCode::Profile10, 0)]->ComputeNextState(time_of_interest, user_time_tracker[joint_index] + zero_time[joint_index][index], joint_index);
                break;
            case ValidProfileCode::Profile11:
                // cout << "11" << endl;
                if (time_of_interest <= user_time_tracker[joint_index] + profiles[make_pair(ProfileCode::Profile11, 0)]->T(joint_index, 0) + zero_time[joint_index][index])
                    profiles[make_pair(ProfileCode::Profile11, 0)]->ComputeNextState(time_of_interest, user_time_tracker[joint_index] + zero_time[joint_index][index], joint_index);
                break;
            case ValidProfileCode::Profile12:
                // cout << "12" << endl;
                if (time_of_interest <= user_time_tracker[joint_index] + profiles[make_pair(ProfileCode::Profile12, 0)]->T(joint_index, 0) + zero_time[joint_index][index])
                    profiles[make_pair(ProfileCode::Profile12, 0)]->ComputeNextState(time_of_interest, user_time_tracker[joint_index] + zero_time[joint_index][index], joint_index);
                break;
            case ValidProfileCode::Profile13:
                // cout << "13" << endl;
                if (time_of_interest <= user_time_tracker[joint_index] + profiles[make_pair(ProfileCode::Profile13, 0)]->T(joint_index, 0) + zero_time[joint_index][index])
                    profiles[make_pair(ProfileCode::Profile13, 0)]->ComputeNextState(time_of_interest, user_time_tracker[joint_index] + zero_time[joint_index][index], joint_index);
                break;
            case ValidProfileCode::Profile14:
                // cout << "14" << endl;
                if (time_of_interest <= user_time_tracker[joint_index] + profiles[make_pair(ProfileCode::Profile14, 0)]->T(joint_index, 0) + zero_time[joint_index][index])
                    profiles[make_pair(ProfileCode::Profile14, 0)]->ComputeNextState(time_of_interest, user_time_tracker[joint_index] + zero_time[joint_index][index], joint_index);
                break;
            case ValidProfileCode::Profile15:
                // cout << "15" << endl;
                if (time_of_interest <= user_time_tracker[joint_index] + profiles[make_pair(ProfileCode::Profile15, 0)]->T(joint_index, 0) + zero_time[joint_index][index])
                    profiles[make_pair(ProfileCode::Profile15, 0)]->ComputeNextState(time_of_interest, user_time_tracker[joint_index] + zero_time[joint_index][index], joint_index);
                break;
            case ValidProfileCode::Profile16:
                // cout << "16" << endl;
                if (time_of_interest <= user_time_tracker[joint_index] + profiles[make_pair(ProfileCode::Profile16, 0)]->T(joint_index, 0) + zero_time[joint_index][index])
                    profiles[make_pair(ProfileCode::Profile16, 0)]->ComputeNextState(time_of_interest, user_time_tracker[joint_index] + zero_time[joint_index][index], joint_index);
                break;
            case ValidProfileCode::PositiveVariantB:
                // if (user_time_tracker[joint_index] > 6.996)
                //     error_log << "PositiveVariantB - index : " << index << endl;
                if (time_of_interest <= user_time_tracker[joint_index] + profiles[make_pair(ProfileCode::PositiveVariantB, index)]->T(joint_index, 0) + zero_time[joint_index][index])
                    profiles[make_pair(ProfileCode::PositiveVariantB, index)]->ComputeNextState(time_of_interest, user_time_tracker[joint_index] + zero_time[joint_index][index], joint_index);
                break;
            case ValidProfileCode::NegativeVariantB:
                // cout << "NegativeVariantB" << endl;
                if (time_of_interest <= user_time_tracker[joint_index] + profiles[make_pair(ProfileCode::NegativeVariantB, index)]->T(joint_index, 0) + zero_time[joint_index][index])
                    profiles[make_pair(ProfileCode::NegativeVariantB, index)]->ComputeNextState(time_of_interest, user_time_tracker[joint_index] + zero_time[joint_index][index], joint_index);
                break;
            }
            if (state_is_generated[joint_index])
                break;
            // if (user_time_tracker[joint_index] > 6.996)
            //     error_log << "error" << endl;
        }
    // cout << "loop done" << endl;
}

template <int DOF>
void OwlOtg::OwlOtg1dof<DOF>::Compute_JointTotalTrajectoryTime(int joint_index)
{
    trajectory_time[joint_index] = 0.0;
    int index = -1;
    if (!profile_sequence[joint_index].empty())
        for (const auto &individual_profile : profile_sequence[joint_index])
        {
            index++;
            switch (individual_profile)
            {
            case ValidProfileCode::Profile9:
                // cout << "9 " << " initial state = " << profiles[make_pair(ProfileCode::Profile9, 0)]->xi(joint_index, 0) << " " << profiles[make_pair(ProfileCode::Profile9, 0)]->vi(joint_index, 0) << " " << profiles[make_pair(ProfileCode::Profile9, 0)]->ai(joint_index, 0) << endl;
                trajectory_time[joint_index] += profiles[make_pair(ProfileCode::Profile9, 0)]->T(joint_index, 0);
                num_of_segments[joint_index] = 7;
                break;
            case ValidProfileCode::Profile10:
                // cout << "10" << " initial state = " << profiles[make_pair(ProfileCode::Profile10, 0)]->xi(joint_index, 0) << " " << profiles[make_pair(ProfileCode::Profile10, 0)]->vi(joint_index, 0) << " " << profiles[make_pair(ProfileCode::Profile10, 0)]->ai(joint_index, 0) << endl;
                trajectory_time[joint_index] += profiles[make_pair(ProfileCode::Profile10, 0)]->T(joint_index, 0);
                num_of_segments[joint_index] = 6;
                break;
            case ValidProfileCode::Profile11:
                // cout << "11" << " initial state = " << profiles[make_pair(ProfileCode::Profile11, 0)]->xi(joint_index, 0) << " " << profiles[make_pair(ProfileCode::Profile11, 0)]->vi(joint_index, 0) << " " << profiles[make_pair(ProfileCode::Profile11, 0)]->ai(joint_index, 0) << endl;
                trajectory_time[joint_index] += profiles[make_pair(ProfileCode::Profile11, 0)]->T(joint_index, 0);
                num_of_segments[joint_index] = 6;
                break;
            case ValidProfileCode::Profile12:
                // cout << "12" << " initial state = " << profiles[make_pair(ProfileCode::Profile12, 0)]->xi(joint_index, 0) << " " << profiles[make_pair(ProfileCode::Profile12, 0)]->vi(joint_index, 0) << " " << profiles[make_pair(ProfileCode::Profile12, 0)]->ai(joint_index, 0) << endl;
                trajectory_time[joint_index] += profiles[make_pair(ProfileCode::Profile12, 0)]->T(joint_index, 0);
                num_of_segments[joint_index] = 5;
                break;
            case ValidProfileCode::Profile13:
                // cout << "13" << " initial state = " << profiles[make_pair(ProfileCode::Profile13, 0)]->xi(joint_index, 0) << " " << profiles[make_pair(ProfileCode::Profile13, 0)]->vi(joint_index, 0) << " " << profiles[make_pair(ProfileCode::Profile13, 0)]->ai(joint_index, 0) << endl;
                trajectory_time[joint_index] += profiles[make_pair(ProfileCode::Profile13, 0)]->T(joint_index, 0);
                num_of_segments[joint_index] = 7;
                break;
            case ValidProfileCode::Profile14:
                // cout << "14" << " initial state = " << profiles[make_pair(ProfileCode::Profile14, 0)]->xi(joint_index, 0) << " " << profiles[make_pair(ProfileCode::Profile14, 0)]->vi(joint_index, 0) << " " << profiles[make_pair(ProfileCode::Profile14, 0)]->ai(joint_index, 0) << endl;
                trajectory_time[joint_index] += profiles[make_pair(ProfileCode::Profile14, 0)]->T(joint_index, 0);
                num_of_segments[joint_index] = 6;
                break;
            case ValidProfileCode::Profile15:
                // cout << "15" << " initial state = " << profiles[make_pair(ProfileCode::Profile15, 0)]->xi(joint_index, 0) << " " << profiles[make_pair(ProfileCode::Profile15, 0)]->vi(joint_index, 0) << " " << profiles[make_pair(ProfileCode::Profile15, 0)]->ai(joint_index, 0) << endl;
                trajectory_time[joint_index] += profiles[make_pair(ProfileCode::Profile15, 0)]->T(joint_index, 0);
                num_of_segments[joint_index] = 6;
                break;
            case ValidProfileCode::Profile16:
                // cout << "16" << " initial state = " << profiles[make_pair(ProfileCode::Profile16, 0)]->xi(joint_index, 0) << " " << profiles[make_pair(ProfileCode::Profile16, 0)]->vi(joint_index, 0) << " " << profiles[make_pair(ProfileCode::Profile16, 0)]->ai(joint_index, 0) << endl;
                trajectory_time[joint_index] += profiles[make_pair(ProfileCode::Profile16, 0)]->T(joint_index, 0);
                num_of_segments[joint_index] = 5;
                break;
            case ValidProfileCode::PositiveVariantB:
                // cout << "PositiveVariantB" << " initial state = " << profiles[make_pair(ProfileCode::PositiveVariantB, index)]->xi(joint_index, 0) << " " << profiles[make_pair(ProfileCode::PositiveVariantB, index)]->vi(joint_index, 0) << " " << profiles[make_pair(ProfileCode::PositiveVariantB, index)]->ai(joint_index, 0) << endl;
                trajectory_time[joint_index] += profiles[make_pair(ProfileCode::PositiveVariantB, index)]->T(joint_index, 0);
                break;
            case ValidProfileCode::NegativeVariantB:
                // cout << "NegativeVariantB" << " initial state = " << profiles[make_pair(ProfileCode::NegativeVariantB, index)]->xi(joint_index, 0) << " " << profiles[make_pair(ProfileCode::NegativeVariantB, index)]->vi(joint_index, 0) << " " << profiles[make_pair(ProfileCode::NegativeVariantB, index)]->ai(joint_index, 0) << endl;
                trajectory_time[joint_index] += profiles[make_pair(ProfileCode::NegativeVariantB, index)]->T(joint_index, 0);
                break;
            }
        }
    cout << "profile_sequence[joint_index].size = " << profile_sequence[joint_index].size() << endl;
}
