#include "otg.h"
#include <fstream>
#include <cmath>
#include <algorithm>

using namespace std;
using namespace OwlOtg;

template <int DOF>
OwlOtgNDof<DOF>::OwlOtgNDof(double control_frequency, vector<double> v_min_, vector<double> a_min_, vector<bool> plot_data_) : otg_instance(control_frequency, v_min_, a_min_, plot_data_) {}

template <int DOF>
void OwlOtgNDof<DOF>::Reset_Time()
{
    otg_instance.Reset_JointTime(0);
}

template <int DOF>
vector<Eigen::Vector4d> OwlOtgNDof<DOF>::Generate_NextState(Eigen::Matrix<double, DOF, 1> &current_position,
                                                            double &current_speed, double &current_acc, double target_speed,
                                                            const Eigen::Matrix<double, DOF, 1> &target_position,
                                                            Eigen::Matrix<double, 1, 3> &boundary_condition,
                                                            const UserInput &user_input, double time_of_interest,
                                                            int index)
{
    double sum = 0.0;
    for (int i = 0; i < DOF; i++)
        sum += pow(target_position(i, 0) - current_position(i, 0), 2);

    double r_mag = sqrt(sum);
    Eigen::Matrix<double, DOF, 1> r_cap;
    for (int i = 0; i < DOF; i++)
        r_cap(i, 0) = (target_position(i, 0) - current_position(i, 0)) / r_mag;

    Eigen::Matrix<double, 1, 3> current_state, target_state;
    current_state << 0.0, current_speed, current_acc;
    target_state << r_mag, target_speed, 0.0;

    otg_instance.Generate_JointNextState(current_state, target_state, boundary_condition, user_input, time_of_interest, index, 0);
    Eigen::Matrix<double, DOF, 1> r_current, v_current, a_current;

    for (int i = 0; i < DOF; i++)
    {
        r_current = current_position + (profile_point[0](0) * r_cap);
        v_current = (profile_point[0](1) * r_cap);
        a_current = (profile_point[0](2) * r_cap);
    }

    vector<Eigen::Vector4d> otg_point;
    otg_point.resize(DOF);
    for (int i = 0; i < DOF; i++)
        otg_point[i] << r_current(i), v_current(i), a_current(i), time_of_interest;
    return otg_point;
}

template <int DOF>
void OwlOtgNDof<DOF>::Generate_Profile(Eigen::Matrix<double, DOF, 1> &current_position,
                                       double &current_speed, double &current_acc, double target_speed,
                                       const Eigen::Matrix<double, DOF, 1> &target_position,
                                       Eigen::Matrix<double, 1, 3> &boundary_condition)
{
    double sum = 0.0;
    for (int i = 0; i < DOF; i++)
        sum += pow(target_position(i, 0) - current_position(i, 0), 2);

    double r_mag = sqrt(sum);
    Eigen::Matrix<double, DOF, 1> r_cap;
    for (int i = 0; i < DOF; i++)
        r_cap(i, 0) = (target_position(i, 0) - current_position(i, 0)) / r_mag;

    Eigen::Matrix<double, 1, 3> current_state, target_state;
    current_state << 0.0, current_speed, current_acc;
    target_state << r_mag, target_speed, 0.0;

    otg_instance.Generate_JointProfile(current_state, target_state, boundary_condition, 0);
    // --------> ans is stored in n_profiles[0]
    string path_to_file;
    Eigen::Matrix<double, DOF, 1> r_current, v_current, a_current;
    for (const auto &single_dimension_point : n_profiles[0])
    {
        r_current = current_position + (single_dimension_point(0) * r_cap);
        v_current = (single_dimension_point(1) * r_cap);
        a_current = (single_dimension_point(2) * r_cap);

        for (int i = 0; i < DOF; i++)
        {
            if (i == 0)
                path_to_file = "/home/akarshan/OTG_2/owl_otg/one_dof_otg/text_files/N_dof/one.txt";
            else if (i == 1)
                path_to_file = "/home/akarshan/OTG_2/owl_otg/one_dof_otg/text_files/N_dof/two.txt";
            else if (i == 2)
                path_to_file = "/home/akarshan/OTG_2/owl_otg/one_dof_otg/text_files/N_dof/three.txt";
            else if (i == 3)
                path_to_file = "/home/akarshan/OTG_2/owl_otg/one_dof_otg/text_files/N_dof/four.txt";
            else if (i == 4)
                path_to_file = "/home/akarshan/OTG_2/owl_otg/one_dof_otg/text_files/N_dof/five.txt";
            else if (i == 5)
                path_to_file = "/home/akarshan/OTG_2/owl_otg/one_dof_otg/text_files/N_dof/six.txt";
            std::ofstream log_file(path_to_file, std::ios::out | std::ios::app);
            log_file << r_current(i) << " " << v_current(i) << " " << a_current(i) << " " << single_dimension_point(3) << endl;
        }
    }
}