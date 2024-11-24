#include "otg.h"
#include <fstream>
#include <cmath>

using namespace OwlOtg;
/*
lets say we have n profiles. P1,P2,P3,.....,Pn
zero_time[joint_index] = 0,T1,T1+T2,.............,T1+T2+T3+.........+Tn-1
user_time_tracker = time from start whenever input changes
*/

// tj1 and tb_j1 are different
template <int DOF>
ProfileCode PositiveVarientB<DOF>::ComputeSegmentTrajectoryTimes(Eigen::Matrix<double, DOF, 3> &current_state,
                                                                 const Eigen::Matrix<double, DOF, 3> &target_state,
                                                                 Eigen::Matrix<double, DOF, 3> &boundary_condition, int joint_index)
{
    // cout << "DT: +B" << endl;
    double xt, vt, at, t = 0.0;
    this->xi(joint_index, 0) = current_state(joint_index, 0), this->vi(joint_index, 0) = current_state(joint_index, 1), this->ai(joint_index, 0) = current_state(joint_index, 2);
    this->x_target(joint_index, 0) = target_state(0), this->v_target(joint_index, 0) = target_state(1);
    this->vm(joint_index, 0) = boundary_condition(joint_index, 0), this->am(joint_index, 0) = boundary_condition(joint_index, 1), this->jm(joint_index, 0) = boundary_condition(joint_index, 2);
    // cout << "initial state +B: " << current_state << endl;

    this->tj1(joint_index, 0) = (this->am(joint_index, 0) + this->ai(joint_index, 0)) / this->jm(joint_index, 0);
    xt = this->xi(joint_index, 0);
    vt = this->vi(joint_index, 0);
    at = this->ai(joint_index, 0);
    double del_v = this->vm(joint_index, 0) / 10.0;
    while (vt > (this->vm(joint_index, 0) - del_v))
    {
        if (at <= -this->am(joint_index, 0))
        {
            xt = 1.0 / 3 * this->jm(joint_index, 0) * pow(this->tj1(joint_index, 0), 3) - 1.0 / 2 * this->am(joint_index, 0) * pow(t, 2) + (this->ai(joint_index, 0) + this->am(joint_index, 0)) * t * this->tj1(joint_index, 0) - 1.0 / 2 * (this->jm(joint_index, 0) * t + this->ai(joint_index, 0) + this->am(joint_index, 0)) * pow(this->tj1(joint_index, 0), 2) + t * this->vi(joint_index, 0) + this->xi(joint_index, 0);
            vt = -1.0 / 2 * this->jm(joint_index, 0) * pow(this->tj1(joint_index, 0), 2) - this->am(joint_index, 0) * t + (this->ai(joint_index, 0) + this->am(joint_index, 0)) * this->tj1(joint_index, 0) + this->vi(joint_index, 0);
            at = -this->am(joint_index, 0);
            if (t - this->tbd(joint_index, 0) > 0.0)
            {
                varB_seg[joint_index] = VariantBSegments::DoubleSegment;
                this->tbc(joint_index, 0) = t - this->tbd(joint_index, 0);
            }
            else
            {
                varB_seg[joint_index] = VariantBSegments::SingleSegment;
                this->tbc(joint_index, 0) = t;
            }
        }
        else
        {
            // this will run 1st
            varB_seg[joint_index] = VariantBSegments::SingleSegment;
            xt = -1.0 / 6 * this->jm(joint_index, 0) * pow(t, 3) + 1.0 / 2 * this->ai(joint_index, 0) * pow(t, 2) + t * this->vi(joint_index, 0) + this->xi(joint_index, 0);
            vt = -1.0 / 2 * this->jm(joint_index, 0) * pow(t, 2) + this->ai(joint_index, 0) * t + this->vi(joint_index, 0);
            at = (-this->jm(joint_index, 0) * t) + this->ai(joint_index, 0);
            // code word
            if (at > this->am(joint_index, 0))
                at = this->am(joint_index, 0);
            else if (at < -this->am(joint_index, 0))
                at = -this->am(joint_index, 0);
            this->tbd(joint_index, 0) = t;
        }
        t += control_cycle;
    }

    this->T(joint_index, 0) = this->tbc(joint_index, 0) + this->tbd(joint_index, 0);
    joint_variant_b_time[joint_index] += this->T(joint_index, 0);
    current_state(joint_index, 0) = xt;
    current_state(joint_index, 1) = vt;
    current_state(joint_index, 2) = at;
    // cout << "after applying +B, current_state : " << current_state << endl;
    // if B is called n times, zero_time has n elements
    zero_time[joint_index].push_back(zero_time[joint_index][zero_time[joint_index].size() - 1] + this->T(joint_index, 0));
    profile_sequence[joint_index].push_back(ValidProfileCode::PositiveVariantB);
    // cout << "this->tbd(joint_index, 0) " << this->tbd(joint_index, 0) << " this->tbc(joint_index, 0) " << this->tbc(joint_index, 0) << "  +T " << this->T(joint_index, 0) << endl;
    return ProfileCode::ValidProfile;
}

template <int DOF>
double PositiveVarientB<DOF>::CalculateTime(int joint_index, double dist, double &t_prev)
{
    return 0.0;
}

template <int DOF>
void PositiveVarientB<DOF>::HelpingFun2Gen_State(double &xt, double &vt, double &at, double t, int joint_index)
{
    state_is_generated[joint_index] = true;
    // cout << "pos B t = " << t << endl;
    if (varB_seg[joint_index] == VariantBSegments::DoubleSegment)
    {
        if (t <= this->tbd(joint_index, 0))
        {
            xt = -1.0 / 6 * this->jm(joint_index, 0) * pow(t, 3) + 1.0 / 2 * this->ai(joint_index, 0) * pow(t, 2) + t * this->vi(joint_index, 0) + this->xi(joint_index, 0);
            vt = -1.0 / 2 * this->jm(joint_index, 0) * pow(t, 2) + this->ai(joint_index, 0) * t + this->vi(joint_index, 0);
            at = (-this->jm(joint_index, 0) * t) + this->ai(joint_index, 0);
            if (at > this->am(joint_index, 0))
                at = this->am(joint_index, 0);
            else if (at < -this->am(joint_index, 0))
                at = -this->am(joint_index, 0);
        }
        else if (t <= this->T(joint_index, 0))
        {
            xt = 1.0 / 3 * this->jm(joint_index, 0) * pow(this->tj1(joint_index, 0), 3) - 1.0 / 2 * this->am(joint_index, 0) * pow(t, 2) + (this->ai(joint_index, 0) + this->am(joint_index, 0)) * t * this->tj1(joint_index, 0) - 1.0 / 2 * (this->jm(joint_index, 0) * t + this->ai(joint_index, 0) + this->am(joint_index, 0)) * pow(this->tj1(joint_index, 0), 2) + t * this->vi(joint_index, 0) + this->xi(joint_index, 0);
            vt = -1.0 / 2 * this->jm(joint_index, 0) * pow(this->tj1(joint_index, 0), 2) - this->am(joint_index, 0) * t + (this->ai(joint_index, 0) + this->am(joint_index, 0)) * this->tj1(joint_index, 0) + this->vi(joint_index, 0);
            at = -this->am(joint_index, 0);
        }
    }
    else if (varB_seg[joint_index] == VariantBSegments::SingleSegment && this->tbd(joint_index, 0) < 0.0005)
    {
        xt = 1.0 / 3 * this->jm(joint_index, 0) * pow(this->tj1(joint_index, 0), 3) - 1.0 / 2 * this->am(joint_index, 0) * pow(t, 2) + (this->ai(joint_index, 0) + this->am(joint_index, 0)) * t * this->tj1(joint_index, 0) - 1.0 / 2 * (this->jm(joint_index, 0) * t + this->ai(joint_index, 0) + this->am(joint_index, 0)) * pow(this->tj1(joint_index, 0), 2) + t * this->vi(joint_index, 0) + this->xi(joint_index, 0);
        vt = -1.0 / 2 * this->jm(joint_index, 0) * pow(this->tj1(joint_index, 0), 2) - this->am(joint_index, 0) * t + (this->ai(joint_index, 0) + this->am(joint_index, 0)) * this->tj1(joint_index, 0) + this->vi(joint_index, 0);
        at = -this->am(joint_index, 0);
    }
    else if (varB_seg[joint_index] == VariantBSegments::SingleSegment && this->tbc(joint_index, 0) < 0.0005)
    {
        xt = -1.0 / 6 * this->jm(joint_index, 0) * pow(t, 3) + 1.0 / 2 * this->ai(joint_index, 0) * pow(t, 2) + t * this->vi(joint_index, 0) + this->xi(joint_index, 0);
        vt = -1.0 / 2 * this->jm(joint_index, 0) * pow(t, 2) + this->ai(joint_index, 0) * t + this->vi(joint_index, 0);
        at = (-this->jm(joint_index, 0) * t) + this->ai(joint_index, 0);
        if (at > this->am(joint_index, 0))
            at = this->am(joint_index, 0);
        else if (at < -this->am(joint_index, 0))
            at = -this->am(joint_index, 0);
    }
}

template <int DOF>
void PositiveVarientB<DOF>::ComputeProfile(int joint_index)
{
    std::ofstream log_file("/home/akarshan/OTG_2/owl_otg/one_dof_otg/text_files/plot_data.txt",
                           std::ios::out | std::ios::app);
    double xt, vt, at;
    Eigen::Vector4d single_point;
    for (double t = 0.0; t <= this->T(joint_index, 0); t += 0.002)
    {
        HelpingFun2Gen_State(xt, vt, at, t, joint_index);
        single_point << xt, vt, at, t + t0[joint_index];
        single_joint_profile.push_back(single_point);
        if (plot_data[joint_index])
            log_file << xt << " " << vt << " " << at << " " << t + t0[joint_index] << endl;
    }

    xt = 1.0 / 3 * this->jm(joint_index, 0) * pow(this->tj1(joint_index, 0), 3) - 1.0 / 2 * this->am(joint_index, 0) * pow(this->T(joint_index, 0), 2) + (this->ai(joint_index, 0) + this->am(joint_index, 0)) * this->T(joint_index, 0) * this->tj1(joint_index, 0) - 1.0 / 2 * (this->jm(joint_index, 0) * this->T(joint_index, 0) + this->ai(joint_index, 0) + this->am(joint_index, 0)) * pow(this->tj1(joint_index, 0), 2) + this->T(joint_index, 0) * this->vi(joint_index, 0) + this->xi(joint_index, 0);
    vt = -1.0 / 2 * this->jm(joint_index, 0) * pow(this->tj1(joint_index, 0), 2) - this->am(joint_index, 0) * this->T(joint_index, 0) + (this->ai(joint_index, 0) + this->am(joint_index, 0)) * this->tj1(joint_index, 0) + this->vi(joint_index, 0);
    at = -this->am(joint_index, 0);

    single_point << xt, vt, at, this->T(joint_index, 0) + t0[joint_index];
    single_joint_profile.push_back(single_point);
    n_profiles[joint_index] = (single_joint_profile);
    if (plot_data[joint_index])
        log_file << xt << " " << vt << " " << at << " " << this->T(joint_index, 0) + t0[joint_index] << endl;

    t0[joint_index] += this->T(joint_index, 0);
}

template <int DOF>
ProfileCode NegativeVarientB<DOF>::ComputeSegmentTrajectoryTimes(Eigen::Matrix<double, DOF, 3> &current_state,
                                                                 const Eigen::Matrix<double, DOF, 3> &target_state,
                                                                 Eigen::Matrix<double, DOF, 3> &boundary_condition, int joint_index)
{
    // cout << "DT: -B" << endl;
    double xt, vt, at, t = 0.0;
    this->xi(joint_index, 0) = current_state(joint_index, 0), this->vi(joint_index, 0) = current_state(joint_index, 1), this->ai(joint_index, 0) = current_state(joint_index, 2);
    this->x_target(joint_index, 0) = target_state(0), this->v_target(joint_index, 0) = target_state(1);
    this->vm(joint_index, 0) = boundary_condition(joint_index, 0), this->am(joint_index, 0) = boundary_condition(joint_index, 1), this->jm(joint_index, 0) = boundary_condition(joint_index, 2);
    // cout << "initial state -B: " << current_state << endl;
    this->tj1(joint_index, 0) = (this->am(joint_index, 0) - this->ai(joint_index, 0)) / this->jm(joint_index, 0);
    xt = this->xi(joint_index, 0);
    vt = this->vi(joint_index, 0);
    at = this->ai(joint_index, 0);
    double del_v = this->vm(joint_index, 0) / 10.0;

    while (vt < -(this->vm(joint_index, 0) - del_v))
    {
        if (at >= this->am(joint_index, 0))
        {
            xt = -1.0 / 3.0 * this->jm(joint_index, 0) * pow(this->tj1(joint_index, 0), 3) + 1.0 / 2.0 * this->am(joint_index, 0) * pow(t, 2) + (this->ai(joint_index, 0) - this->am(joint_index, 0)) * t * this->tj1(joint_index, 0) + 1.0 / 2.0 * (this->jm(joint_index, 0) * t - this->ai(joint_index, 0) + this->am(joint_index, 0)) * pow(this->tj1(joint_index, 0), 2) + t * this->vi(joint_index, 0) + this->xi(joint_index, 0);
            vt = 1.0 / 2.0 * this->jm(joint_index, 0) * pow(this->tj1(joint_index, 0), 2) + this->am(joint_index, 0) * t + (this->ai(joint_index, 0) - this->am(joint_index, 0)) * this->tj1(joint_index, 0) + this->vi(joint_index, 0);
            at = this->am(joint_index, 0);
            if (t - this->tba(joint_index, 0) > 0.0)
            {
                varB_seg[joint_index] = VariantBSegments::DoubleSegment;
                this->tbc(joint_index, 0) = t - this->tba(joint_index, 0);
            }
            else
            {
                varB_seg[joint_index] = VariantBSegments::SingleSegment;
                this->tbc(joint_index, 0) = t;
            }
        }
        else
        {
            varB_seg[joint_index] = VariantBSegments::SingleSegment;
            xt = 1.0 / 6.0 * this->jm(joint_index, 0) * pow(t, 3) + 1.0 / 2.0 * this->ai(joint_index, 0) * pow(t, 2) + t * this->vi(joint_index, 0) + this->xi(joint_index, 0);
            vt = 1.0 / 2.0 * this->jm(joint_index, 0) * pow(t, 2) + this->ai(joint_index, 0) * t + this->vi(joint_index, 0);
            at = (this->jm(joint_index, 0) * t) + this->ai(joint_index, 0);
            if (at > this->am(joint_index, 0))
                at = this->am(joint_index, 0);
            else if (at < -this->am(joint_index, 0))
                at = -this->am(joint_index, 0);
            this->tba(joint_index, 0) = t;
        }
        t += control_cycle;
    }

    this->T(joint_index, 0) = this->tbc(joint_index, 0) + this->tba(joint_index, 0);
    joint_variant_b_time[joint_index] += this->T(joint_index, 0);
    current_state(joint_index, 0) = xt;
    current_state(joint_index, 1) = vt;
    current_state(joint_index, 2) = at;
    // cout << "after applying -B, current_state : " << current_state << endl;
    zero_time[joint_index].push_back(zero_time[joint_index][zero_time[joint_index].size() - 1] + this->T(joint_index, 0));
    profile_sequence[joint_index].push_back(ValidProfileCode::NegativeVariantB);
    // cout << "this->tba(joint_index, 0) " << this->tba(joint_index, 0) << " this->tbc(joint_index, 0) " << this->tbc(joint_index, 0) << "   -T " << this->T(joint_index, 0) << endl;
    return ProfileCode::ValidProfile;
}

template <int DOF>
void NegativeVarientB<DOF>::HelpingFun2Gen_State(double &xt, double &vt, double &at, double t, int joint_index)
{
    state_is_generated[joint_index] = true;
    // cout << "neg B t = " << t << endl;
    if (varB_seg[joint_index] == VariantBSegments::DoubleSegment)
    {
        if (t <= this->tba(joint_index, 0))
        {
            xt = 1.0 / 6.0 * this->jm(joint_index, 0) * pow(t, 3) + 1.0 / 2.0 * this->ai(joint_index, 0) * pow(t, 2) + t * this->vi(joint_index, 0) + this->xi(joint_index, 0);
            vt = 1.0 / 2.0 * this->jm(joint_index, 0) * pow(t, 2) + this->ai(joint_index, 0) * t + this->vi(joint_index, 0);
            at = (this->jm(joint_index, 0) * t) + this->ai(joint_index, 0);
            if (at > this->am(joint_index, 0))
                at = this->am(joint_index, 0);
            else if (at < -this->am(joint_index, 0))
                at = -this->am(joint_index, 0);
        }
        else if (t <= this->T(joint_index, 0))
        {
            xt = -1.0 / 3.0 * this->jm(joint_index, 0) * pow(this->tj1(joint_index, 0), 3) + 1.0 / 2.0 * this->am(joint_index, 0) * pow(t, 2) + (this->ai(joint_index, 0) - this->am(joint_index, 0)) * t * this->tj1(joint_index, 0) + 1.0 / 2.0 * (this->jm(joint_index, 0) * t - this->ai(joint_index, 0) + this->am(joint_index, 0)) * pow(this->tj1(joint_index, 0), 2) + t * this->vi(joint_index, 0) + this->xi(joint_index, 0);
            vt = 1.0 / 2.0 * this->jm(joint_index, 0) * pow(this->tj1(joint_index, 0), 2) + this->am(joint_index, 0) * t + (this->ai(joint_index, 0) - this->am(joint_index, 0)) * this->tj1(joint_index, 0) + this->vi(joint_index, 0);
            at = this->am(joint_index, 0);
        }
    }
    else if (varB_seg[joint_index] == VariantBSegments::SingleSegment && this->tba(joint_index, 0) < 0.0005)
    {
        xt = -1.0 / 3.0 * this->jm(joint_index, 0) * pow(this->tj1(joint_index, 0), 3) + 1.0 / 2.0 * this->am(joint_index, 0) * pow(t, 2) + (this->ai(joint_index, 0) - this->am(joint_index, 0)) * t * this->tj1(joint_index, 0) + 1.0 / 2.0 * (this->jm(joint_index, 0) * t - this->ai(joint_index, 0) + this->am(joint_index, 0)) * pow(this->tj1(joint_index, 0), 2) + t * this->vi(joint_index, 0) + this->xi(joint_index, 0);
        vt = 1.0 / 2.0 * this->jm(joint_index, 0) * pow(this->tj1(joint_index, 0), 2) + this->am(joint_index, 0) * t + (this->ai(joint_index, 0) - this->am(joint_index, 0)) * this->tj1(joint_index, 0) + this->vi(joint_index, 0);
        at = this->am(joint_index, 0);
    }
    else if (varB_seg[joint_index] == VariantBSegments::SingleSegment && this->tbc(joint_index, 0) < 0.0005)
    {
        xt = 1.0 / 6.0 * this->jm(joint_index, 0) * pow(t, 3) + 1.0 / 2.0 * this->ai(joint_index, 0) * pow(t, 2) + t * this->vi(joint_index, 0) + this->xi(joint_index, 0);
        vt = 1.0 / 2.0 * this->jm(joint_index, 0) * pow(t, 2) + this->ai(joint_index, 0) * t + this->vi(joint_index, 0);
        at = (this->jm(joint_index, 0) * t) + this->ai(joint_index, 0);
        if (at > this->am(joint_index, 0))
            at = this->am(joint_index, 0);
        else if (at < -this->am(joint_index, 0))
            at = -this->am(joint_index, 0);
    }
}

template <int DOF>
double NegativeVarientB<DOF>::CalculateTime(int joint_index, double dist, double &t_prev)
{
    return 0.0;
}

template <int DOF>
void NegativeVarientB<DOF>::ComputeProfile(int joint_index)
{
    std::ofstream log_file("/home/akarshan/OTG_2/owl_otg/one_dof_otg/text_files/plot_data.txt",
                           std::ios::out | std::ios::app);
    double xt, vt, at;
    Eigen::Vector4d single_point;
    for (double t = 0.0; t <= this->T(joint_index, 0); t += 0.002)
    {
        HelpingFun2Gen_State(xt, vt, at, t, joint_index);
        single_point << xt, vt, at, t + t0[joint_index];
        single_joint_profile.push_back(single_point);
        if (plot_data[joint_index])
            log_file << xt << " " << vt << " " << at << " " << t + t0[joint_index] << endl;
    }

    xt = -1.0 / 3.0 * this->jm(joint_index, 0) * pow(this->tj1(joint_index, 0), 3) + 1.0 / 2.0 * this->am(joint_index, 0) * pow(this->T(joint_index, 0), 2) + (this->ai(joint_index, 0) - this->am(joint_index, 0)) * this->T(joint_index, 0) * this->tj1(joint_index, 0) + 1.0 / 2.0 * (this->jm(joint_index, 0) * this->T(joint_index, 0) - this->ai(joint_index, 0) + this->am(joint_index, 0)) * pow(this->tj1(joint_index, 0), 2) + this->T(joint_index, 0) * this->vi(joint_index, 0) + this->xi(joint_index, 0);
    vt = 1.0 / 2.0 * this->jm(joint_index, 0) * pow(this->tj1(joint_index, 0), 2) + this->am(joint_index, 0) * this->T(joint_index, 0) + (this->ai(joint_index, 0) - this->am(joint_index, 0)) * this->tj1(joint_index, 0) + this->vi(joint_index, 0);
    at = this->am(joint_index, 0);

    single_point << xt, vt, at, this->T(joint_index, 0) + t0[joint_index];
    single_joint_profile.push_back(single_point);
    n_profiles[joint_index] = (single_joint_profile);
    if (plot_data[joint_index])
        log_file << xt << " " << vt << " " << at << " " << this->T(joint_index, 0) + t0[joint_index] << endl;

    t0[joint_index] += this->T(joint_index, 0);
}
