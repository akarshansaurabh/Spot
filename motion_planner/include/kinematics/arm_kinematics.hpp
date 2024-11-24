#ifndef ARM_KINEMATICS_HPP
#define ARM_KINEMATICS_HPP

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chain.hpp>

#include "miscellaneous/conversion.hpp"

#include "maths/commonmathssolver.hpp"
#include "maths/curves.hpp"

using namespace std;

namespace ArmKinematics
{
    enum class ArmLinkNames
    {
        link1,
        link2,
        link34,
        link5,
        link6
    };

    class ArmKinematicsSolver : public rclcpp::Node
    {
    private:
        int dof_num;
        KDL::Tree robot_tree_;
        KDL::Chain chain_;
        std::string root_link_;
        std::string tip_link_;

        std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
        std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;

        bool SolveIKIntermediate(const KDL::Frame &end_effector_pose);

    public:
        KDL::Frame sixDpose;
        KDL::JntArray initial_guess, q, q_home;
        Eigen::Matrix4d pose_6_wrt_0;
        ArmKinematicsSolver(const std::string &urdf_param, const std::string &root_link, const std::string &tip_link, string str);
        int GetDOF();
        void SolveFK(const KDL::JntArray &joint_positions);
        bool SolveIK(const KDL::Frame &target_pose);
    };

    void InitializeArmSolverInstances();
    extern std::string root_link, tip_link, urdf_param;
    extern std::map<ArmKinematics::ArmLinkNames, std::shared_ptr<ArmKinematics::ArmKinematicsSolver>> arm_solver;
}

#endif