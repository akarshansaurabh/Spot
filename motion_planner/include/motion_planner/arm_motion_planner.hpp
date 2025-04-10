#ifndef ARM_MOTION_PLANNER_HPP
#define ARM_MOTION_PLANNER_HPP

#include "maths/commonmathssolver.hpp"
#include "maths/curves.hpp"
#include "kinematics/arm_kinematics.hpp"
#include "kinematics/kinematics.hpp"
#include <eigen3/Eigen/Dense>

using namespace std;

namespace ArmMotionPlanner
{
    class PathPlanner
    {
    private:
        double precision;

    public:
        Eigen::Matrix4d T_e_wrt_b;
        vector<KDL::JntArray> arm_ik;

        PathPlanner(double precision_);
        void LinePlanner(const Eigen::Matrix4d &goal);
        void ArcPlanner(const Eigen::Matrix4d &p1, const Eigen::Matrix4d &p2, const Eigen::Matrix4d &p3, int num, const string &str);
        void BSplinePlanner(Curves::BSpline &bspline_curve, const Eigen::Matrix4d &goal, int num, Eigen::Matrix4d &pose1);
        void ChickenHeadPlanner(const Eigen::Matrix4d &goal, const vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_pose_array);
        vector<Eigen::Vector4d> GenerateBSPlineCP(const Eigen::Matrix4d &goal, Eigen::Matrix4d &current_pose, double z_shift, bool b);
        vector<Eigen::Vector4d> DancingGenerateBSPlineCP(vector<Eigen::Vector4d> &cp_i, const Eigen::Matrix4d &goal, double z_shift, double turning_radius, int index);

        void ReplacArmAngles(int num);
    };

}

#endif