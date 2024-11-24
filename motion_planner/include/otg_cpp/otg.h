#ifndef OTG_H
#define OTG_H

#include <iostream>
#include <vector>
#include <memory>
#include <map>
#include <eigen3/Eigen/Dense>

using namespace std;

namespace OwlOtg
{
    // user_time_tracker -> time measured from start at which user changes the boundary condition
    // trajectory_time -> total trajectory time of the single entire profile (min=1 and max = 2) in offline mode
    // lets say we have n profiles. P1,P2,P3,.....,Pn
    // if there are 2 profiles, zero_time = starting time for each of the 2 profiles.
    // zero_time = 0,T1,T1+T2,.............,T1+T2+T3+.........+Tn-1
    // user_time_tracker = time from start whenever input changes

    extern double control_cycle;
    extern vector<double> a_min, v_min;
    extern vector<bool> plot_data, state_is_generated;
    extern vector<double> trajectory_time, user_time_tracker, joint_variant_b_time, joint_variant_a_time;
    extern vector<int> global_dt_counter, no_sol, variant_b_index, num_of_segments;

    enum class UserInput
    {
        ChangedVelocity,
        DidNotChangeVelocity
    };

    enum class ProfileCode
    {
        ValidProfile,
        Profile9,
        Profile10,
        Profile11,
        Profile12,
        Profile13,
        Profile14,
        Profile15,
        Profile16,
        PositiveVariantB,
        NegativeVariantB,
        ErrorProfile
    };

    enum class ValidProfileCode
    {
        Profile9,
        Profile10,
        Profile11,
        Profile12,
        Profile13,
        Profile14,
        Profile15,
        Profile16,
        PositiveVariantB,
        NegativeVariantB
    };

    enum class VariantBSegments
    {
        SingleSegment,
        DoubleSegment
    };

    extern vector<vector<Eigen::Vector4d>> invalid_profile;
    extern vector<vector<ValidProfileCode>> profile_sequence;
    extern vector<vector<Eigen::Vector4d>> profile_matrix, n_profiles; // vector of profile_point
    extern vector<Eigen::Vector4d> profile_point, single_joint_profile;
    extern vector<vector<double>> zero_time;
    extern vector<double> t0;

    template <typename T>
    inline T FindMaxWithIndex(const vector<T> &vect, int &index)
    {
        T max = vect[0];
        index = 0;
        for (int i = 1; i < vect.size(); i++)
        {
            if (vect[i] >= max)
            {
                max = vect[i];
                index = i;
            }
        }
        return max;
    }

    template <int DOF>
    class ProfileBase
    {
    protected:
    public:
        Eigen::Matrix<double, DOF, 1> vm, am, jm;
        Eigen::Matrix<double, DOF, 1> x_target, v_target;
        Eigen::Matrix<double, DOF, 1> ap1, ap2;

        Eigen::Matrix<double, DOF, 1> T;
        Eigen::Matrix<double, DOF, 1> xi, vi, ai;
        Eigen::Matrix<double, DOF, 1> tba, tbc, tbd;
        Eigen::Matrix<double, DOF, 1> tb_j1, tb_c, tb_j2;
        Eigen::Matrix<double, DOF, 1> tj1, ta, tj1_, tc, tj2, td, tj2_;

        virtual ProfileCode ComputeSegmentTrajectoryTimes(Eigen::Matrix<double, DOF, 3> &current_state,
                                                          const Eigen::Matrix<double, DOF, 3> &target_state,
                                                          Eigen::Matrix<double, DOF, 3> &boundary_condition, int joint_index) = 0;

        virtual void ComputeProfile(int joint_index) = 0;

        virtual void HelpingFun2Gen_State(double &xt, double &vt, double &at, double t, int joint_index) = 0;

        virtual double CalculateTime(int joint_index, double dist, double &t_prev ) = 0;

        ProfileCode TargetStateAchieved(const Eigen::Matrix<double, DOF, 3> &target_state,
                                        double xt, double vt, double at, int joint_index);

        void ComputeNextState(double time_of_interest, double zero_time, int joint_index);

        virtual ~ProfileBase() = default;
    };

    template <int DOF>
    class Profile9 : public ProfileBase<DOF>
    {
    public:
        ProfileCode ComputeSegmentTrajectoryTimes(
            Eigen::Matrix<double, DOF, 3> &current_state,
            const Eigen::Matrix<double, DOF, 3> &target_state,
            Eigen::Matrix<double, DOF, 3> &boundary_condition, int joint_index) override;

        void HelpingFun2Gen_State(double &xt, double &vt, double &at, double t, int joint_index) override;

        void ComputeProfile(int joint_index) override;

        double CalculateTime(int joint_index, double dist, double &t_prev ) override;
    };

    template <int DOF>
    class Profile10 : public ProfileBase<DOF>
    {
    public:
        ProfileCode ComputeSegmentTrajectoryTimes(
            Eigen::Matrix<double, DOF, 3> &current_state,
            const Eigen::Matrix<double, DOF, 3> &target_state,
            Eigen::Matrix<double, DOF, 3> &boundary_condition, int joint_index) override;

        void HelpingFun2Gen_State(double &xt, double &vt, double &at, double t, int joint_index) override;

        void ComputeProfile(int joint_index) override;

        double CalculateTime(int joint_index, double dist, double &t_prev ) override;
    };

    template <int DOF>
    class Profile11 : public ProfileBase<DOF>
    {
    public:
        ProfileCode ComputeSegmentTrajectoryTimes(
            Eigen::Matrix<double, DOF, 3> &current_state,
            const Eigen::Matrix<double, DOF, 3> &target_state,
            Eigen::Matrix<double, DOF, 3> &boundary_condition, int joint_index);

        void HelpingFun2Gen_State(double &xt, double &vt, double &at, double t, int joint_index) override;

        void ComputeProfile(int joint_index) override;

        double CalculateTime(int joint_index, double dist, double &t_prev ) override;
    };

    template <int DOF>
    class Profile12 : public ProfileBase<DOF>
    {
    public:
        ProfileCode ComputeSegmentTrajectoryTimes(
            Eigen::Matrix<double, DOF, 3> &current_state,
            const Eigen::Matrix<double, DOF, 3> &target_state,
            Eigen::Matrix<double, DOF, 3> &boundary_condition, int joint_index) override;

        void HelpingFun2Gen_State(double &xt, double &vt, double &at, double t, int joint_index) override;

        void ComputeProfile(int joint_index) override;

        double CalculateTime(int joint_index, double dist, double &t_prev ) override;
    };

    template <int DOF>
    class Profile13 : public ProfileBase<DOF>
    {
    public:
        ProfileCode ComputeSegmentTrajectoryTimes(
            Eigen::Matrix<double, DOF, 3> &current_state,
            const Eigen::Matrix<double, DOF, 3> &target_state,
            Eigen::Matrix<double, DOF, 3> &boundary_condition, int joint_index) override;

        void HelpingFun2Gen_State(double &xt, double &vt, double &at, double t, int joint_index) override;

        void ComputeProfile(int joint_index) override;

        double CalculateTime(int joint_index, double dist, double &t_prev ) override;
    };

    template <int DOF>
    class Profile14 : public ProfileBase<DOF>
    {
    public:
        ProfileCode ComputeSegmentTrajectoryTimes(
            Eigen::Matrix<double, DOF, 3> &current_state,
            const Eigen::Matrix<double, DOF, 3> &target_state,
            Eigen::Matrix<double, DOF, 3> &boundary_condition, int joint_index) override;

        void HelpingFun2Gen_State(double &xt, double &vt, double &at, double t, int joint_index) override;

        void ComputeProfile(int joint_index) override;

        double CalculateTime(int joint_index, double dist, double &t_prev ) override;
    };

    template <int DOF>
    class Profile15 : public ProfileBase<DOF>
    {
    public:
        ProfileCode ComputeSegmentTrajectoryTimes(
            Eigen::Matrix<double, DOF, 3> &current_state,
            const Eigen::Matrix<double, DOF, 3> &target_state,
            Eigen::Matrix<double, DOF, 3> &boundary_condition, int joint_index) override;

        void HelpingFun2Gen_State(double &xt, double &vt, double &at, double t, int joint_index) override;

        void ComputeProfile(int joint_index) override;

        double CalculateTime(int joint_index, double dist, double &t_prev ) override;
    };

    template <int DOF>
    class Profile16 : public ProfileBase<DOF>
    {
    public:
        ProfileCode ComputeSegmentTrajectoryTimes(
            Eigen::Matrix<double, DOF, 3> &current_state,
            const Eigen::Matrix<double, DOF, 3> &target_state,
            Eigen::Matrix<double, DOF, 3> &boundary_condition, int joint_index) override;

        void HelpingFun2Gen_State(double &xt, double &vt, double &at, double t, int joint_index) override;

        void ComputeProfile(int joint_index) override;

        double CalculateTime(int joint_index, double dist, double &t_prev ) override;
    };

    template <int DOF>
    class PositiveVarientB : public ProfileBase<DOF>
    {
    private:
        VariantBSegments varB_seg[DOF];

    public:
        Eigen::Matrix<double, DOF, 3> vaiantB_currentstate_forProfileComputation;
        ProfileCode ComputeSegmentTrajectoryTimes(
            Eigen::Matrix<double, DOF, 3> &current_state,
            const Eigen::Matrix<double, DOF, 3> &target_state,
            Eigen::Matrix<double, DOF, 3> &boundary_condition, int joint_index) override;

        void HelpingFun2Gen_State(double &xt, double &vt, double &at, double t, int joint_index) override;

        void ComputeProfile(int joint_index) override;

        double CalculateTime(int joint_index, double dist, double &t_prev ) override;
    };

    template <int DOF>
    class NegativeVarientB : public ProfileBase<DOF>
    {
    private:
        VariantBSegments varB_seg[DOF];

    public:
        Eigen::Matrix<double, DOF, 3> vaiantB_currentstate_forProfileComputation;
        ProfileCode ComputeSegmentTrajectoryTimes(
            Eigen::Matrix<double, DOF, 3> &current_state,
            const Eigen::Matrix<double, DOF, 3> &target_state,
            Eigen::Matrix<double, DOF, 3> &boundary_condition, int joint_index) override;

        void HelpingFun2Gen_State(double &xt, double &vt, double &at, double t, int joint_index) override;

        void ComputeProfile(int joint_index) override;

        double CalculateTime(int joint_index, double dist, double &t_prev ) override;
    };

    template <int DOF>
    class OwlOtg1dof
    {
    protected:
        map<pair<ProfileCode, int>, unique_ptr<ProfileBase<DOF>>> profiles;

        void FuncForLastProfile_SingleJoint(ProfileCode &expr,
                                            Eigen::Matrix<double, DOF, 3> &current_state,
                                            const Eigen::Matrix<double, DOF, 3> &target_state,
                                            Eigen::Matrix<double, DOF, 3> &boundary_condition, int joint_index);

        void FuncForBothRows_SingleJoint(ProfileCode &expr,
                                         Eigen::Matrix<double, DOF, 3> &current_state,
                                         const Eigen::Matrix<double, DOF, 3> &target_state,
                                         Eigen::Matrix<double, DOF, 3> &boundary_condition, int joint_index);

        void DecisionTree_1DOF(Eigen::Matrix<double, DOF, 3> &current_state,
                               const Eigen::Matrix<double, DOF, 3> &target_state,
                               Eigen::Matrix<double, DOF, 3> &boundary_condition, int joint_index);

        void Compute_JointTotalTrajectoryTime(int joint_index);

    public:
        OwlOtg1dof(double control_frequency, vector<double> v_min_, vector<double> a_min_, vector<bool> plot_data_);
        void Reset_JointTime(int joint_index);
        void Generate_JointNextState(Eigen::Matrix<double, DOF, 3> &current_state,
                                     const Eigen::Matrix<double, DOF, 3> &target_state,
                                     Eigen::Matrix<double, DOF, 3> &boundary_condition,
                                     const UserInput &user_input, double time_of_interest,
                                     int index, int joint_index);
        void Generate_JointProfile(Eigen::Matrix<double, DOF, 3> &current_state,
                                   const Eigen::Matrix<double, DOF, 3> &target_state,
                                   Eigen::Matrix<double, DOF, 3> &boundary_condition, int joint_index);
    };

    template <int DOF>
    class OwlOtgNDof
    {
    private:
        OwlOtg1dof<1> otg_instance;

    public:
        OwlOtgNDof(double control_frequency, vector<double> v_min_, vector<double> a_min_, vector<bool> plot_data_);
        void Reset_Time();
        vector<Eigen::Vector4d> Generate_NextState(Eigen::Matrix<double, DOF, 1> &current_position,
                                                   double &current_speed, double &current_acc, double target_speed,
                                                   const Eigen::Matrix<double, DOF, 1> &target_position,
                                                   Eigen::Matrix<double, 1, 3> &boundary_condition,
                                                   const UserInput &user_input, double time_of_interest,
                                                   int index);
        void Generate_Profile(Eigen::Matrix<double, DOF, 1> &current_position,
                              double &current_speed, double &current_acc, double target_speed,
                              const Eigen::Matrix<double, DOF, 1> &target_position,
                              Eigen::Matrix<double, 1, 3> &boundary_condition);
    };
}

#include "OTG.tpp"
#include "Profile9.tpp"
#include "Profile10.tpp"
#include "Profile11.tpp"
#include "Profile12.tpp"
#include "Profile13.tpp"
#include "Profile14.tpp"
#include "Profile15.tpp"
#include "Profile16.tpp"
#include "VariantBProfiles.tpp"
#include "OWLOTG.tpp"

#endif