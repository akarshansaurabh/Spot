#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "kinematics/kinematics.hpp"
#include "kinematics/arm_kinematics.hpp"

// #include "motion_planner/posture.hpp"
// #include "motion_planner/posture_v2.hpp"
// #include "motion_planner/gaits2.hpp"
#include "motion_planner/gaits_v3.hpp"

#include "com/com_solver.hpp"
#include "rviz2/markers.hpp"

#include "miscellaneous/conversion.hpp"

#include <eigen3/Eigen/Dense>

#include <sensor_msgs/msg/joint_state.hpp>
#include "custom_interfaces/srv/task_test.hpp"

#include <string>
#include <memory>

using namespace std;
int global_id = -1;

class Abc : public rclcpp::Node
{
public:
    Abc();
    void RPY(double t, int p, int y, int num, double r, const Eigen::Matrix4d &goal, string str, double dt);
    void RPYT(double angle, double dist, int r, int p, int y, int num, const Eigen::Matrix4d &goal, double dt, string str);
    void RP(double r, double p, int num, const Eigen::Matrix4d &goal, string str);
    void Translate_test(double x, double z, double d, int num, const Eigen::Matrix4d &goal, string str);
    void test3(double t, int num, const Eigen::Matrix4d &goal, string str);
    void test_circle(const Eigen::Matrix4d &goal);
    bool cb(const std::shared_ptr<custom_interfaces::srv::TaskTest::Request> req,
            std::shared_ptr<custom_interfaces::srv::TaskTest::Response> res);
    void PublishHomeState();
    vector<KDL::JntArray> LinePlanner(const Eigen::Matrix4d &goal, int num, string str, double del_t);
    vector<KDL::JntArray> ArcPlanner(const Eigen::Matrix4d &p1, const Eigen::Matrix4d &p2, const Eigen::Matrix4d &p3, int num, const string &str, double del_t);
    Eigen::Matrix4d T_e_wrt_b;

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointstate_pub_;
    rclcpp::Service<custom_interfaces::srv::TaskTest>::SharedPtr server_;
    // std::map<ArmKinematics::ArmLinkNames, std::shared_ptr<ArmKinematics::ArmKinematicsSolver>> ArmKinematics::arm_solver;
    // std::unique_ptr<COM::ArmSystemCOM> arm_com_solver;
    // std::unique_ptr<COM::LeggedSystemCOM> legs_com_solver;
    std::unique_ptr<COM::WholeSystemCOM> com_solver;
    std::shared_ptr<Markers::Rviz2Visualization> rviz2_ptr;

    void PublishStatesArmFixed(const vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_pose_array,
                               const vector<vector<Eigen::Vector3d>> &leg_ik, const KDL::JntArray &arm_ik, int del_t);
    void PublishStatesLegsFixed(const vector<KDL::JntArray> &arm_ik, int del_t,
                                const pair<Eigen::Vector4d, Eigen::Vector3d> &spot_pose,
                                const vector<Eigen::Vector3d> &leg_ik);
    void PublishStates(const vector<KDL::JntArray> &arm_ik,
                       const vector<vector<Eigen::Vector3d>> &leg_ik,
                       const vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_pose_array,
                       int del_t, vector<Eigen::Vector4d> &com_array);
    vector<KDL::JntArray> ComputeArmJointStates(const Eigen::Matrix4d &goal, const vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_pose_array);
};

Abc::Abc() : Node("test1")
{
    // T_e_wrt_b = ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->pose_6_wrt_0;
    com_solver = std::make_unique<COM::WholeSystemCOM>();
    rviz2_ptr = std::make_shared<Markers::Rviz2Visualization>();
    jointstate_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1000);
    server_ = this->create_service<custom_interfaces::srv::TaskTest>(
        "/test_service", std::bind(&Abc::cb, this, std::placeholders::_1, std::placeholders::_2));
}

void Abc::PublishStates(const vector<KDL::JntArray> &arm_ik,
                        const vector<vector<Eigen::Vector3d>> &leg_ik,
                        const vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_pose_array,
                        int del_t, vector<Eigen::Vector4d> &com_array)
{
    sensor_msgs::msg::JointState joint_states;
    int point_index = -1;

    // cout << "size 1 = " << leg_ik.size() << endl;
    // cout << "size 2 = " << Kinematics::kinematics_solver->stance_legs_wrt_world.size() << endl;
    for (const auto &i : arm_ik)
    {
        global_id++;
        point_index++;
        // cout << "id " << global_id << endl;
        joint_states.header.stamp = this->now();
        joint_states.name = {"translate_x", "translate_y", "translate_z", "rotate_roll", "rotate_pitch", "rotate_yaw",
                             "front_left_hip_x", "front_left_hip_y", "front_left_knee", "front_right_hip_x", "front_right_hip_y", "front_right_knee",
                             "rear_left_hip_x", "rear_left_hip_y", "rear_left_knee", "rear_right_hip_x", "rear_right_hip_y", "rear_right_knee",
                             "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4",
                             "arm_joint5", "arm_joint6", "arm_gripper"};
        joint_states.position.push_back(spot_pose_array[point_index].first(0));
        joint_states.position.push_back(spot_pose_array[point_index].first(1));
        joint_states.position.push_back(spot_pose_array[point_index].first(2));
        joint_states.position.push_back(spot_pose_array[point_index].second(0));
        joint_states.position.push_back(spot_pose_array[point_index].second(1));
        joint_states.position.push_back(spot_pose_array[point_index].second(2));
        joint_states.position.push_back(leg_ik[point_index][0](0));
        joint_states.position.push_back(leg_ik[point_index][0](1));
        joint_states.position.push_back(leg_ik[point_index][0](2));
        joint_states.position.push_back(leg_ik[point_index][3](0));
        joint_states.position.push_back(leg_ik[point_index][3](1));
        joint_states.position.push_back(leg_ik[point_index][3](2));
        joint_states.position.push_back(leg_ik[point_index][1](0));
        joint_states.position.push_back(leg_ik[point_index][1](1));
        joint_states.position.push_back(leg_ik[point_index][1](2));
        joint_states.position.push_back(leg_ik[point_index][2](0));
        joint_states.position.push_back(leg_ik[point_index][2](1));
        joint_states.position.push_back(leg_ik[point_index][2](2));
        joint_states.position.push_back(i(0));
        joint_states.position.push_back(i(1));
        joint_states.position.push_back(i(2));
        joint_states.position.push_back(i(3));
        joint_states.position.push_back(i(4));
        joint_states.position.push_back(i(5));
        joint_states.position.push_back(0.0);
        jointstate_pub_->publish(joint_states);
        joint_states.position.clear();
        // cout << "debug = " << Kinematics::kinematics_solver->stance_legs_wrt_world[point_index].size() << endl;
        // Eigen::Vector4d arm_com_wrt_w = arm_com_solver->ComputeArmCOM(arm_ik[point_index]);
        // Eigen::Vector4d legs_com_wrt_w = legs_com_solver->ComputeQuadCOM(i, spot_pose_array[point_index].first);
        // Eigen::Vector4d com_wrt_w = ((arm_com_solver->arm_mass * arm_com_wrt_w) + (legs_com_solver->quadruped_mass * legs_com_wrt_w)) / (arm_com_solver->arm_mass + legs_com_solver->quadruped_mass);
        // Eigen::Vector4d com_wrt_w = com_solver->ComputeWholeSystemCOM(i, spot_pose_array[point_index], arm_ik[point_index]);
        // com_wrt_w(2) = -0.31;
        Eigen::Vector4d centroid = spot_pose_array[point_index].first;
        centroid(2) = -0.31;
        // cout << "com " << com_array.size() << " arm " << arm_ik.size() << " leg " << leg_ik.size() << endl;
        // rviz2_ptr->VisualizeEverything(Kinematics::kinematics_solver->stance_legs_wrt_world[point_index], com_array[point_index], centroid, 0.02);
        // cout << "size : " << Kinematics::kinematics_solver->stance_legs_wrt_world.size() << endl;
        // cout << "size2 : " << leg_ik.size() << endl;
        rclcpp::sleep_for(std::chrono::milliseconds(del_t));
        // cout << "both" << endl;
    }
}

void Abc::PublishStatesArmFixed(const vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_pose_array,
                                const vector<vector<Eigen::Vector3d>> &leg_ik, const KDL::JntArray &arm_ik, int del_t)
{
    int point_index = -1;
    for (const auto &i : leg_ik)
    {
        point_index++;
        sensor_msgs::msg::JointState joint_solution;

        joint_solution.header.stamp = this->now();
        joint_solution.name = {"translate_x", "translate_y", "translate_z", "rotate_roll", "rotate_pitch", "rotate_yaw",
                               "front_left_hip_x", "front_left_hip_y", "front_left_knee", "front_right_hip_x", "front_right_hip_y", "front_right_knee",
                               "rear_left_hip_x", "rear_left_hip_y", "rear_left_knee", "rear_right_hip_x", "rear_right_hip_y", "rear_right_knee",
                               "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4",
                               "arm_joint5", "arm_joint6", "arm_gripper"};
        // cout << "rviz pos = " << spot_pose_array[point_index].first.adjoint() << endl;
        joint_solution.position.push_back(spot_pose_array[point_index].first(0));
        joint_solution.position.push_back(spot_pose_array[point_index].first(1));
        joint_solution.position.push_back(spot_pose_array[point_index].first(2));
        joint_solution.position.push_back(spot_pose_array[point_index].second(0));
        joint_solution.position.push_back(spot_pose_array[point_index].second(1));
        joint_solution.position.push_back(spot_pose_array[point_index].second(2));
        joint_solution.position.push_back(i[0](0));
        joint_solution.position.push_back(i[0](1));
        joint_solution.position.push_back(i[0](2));
        joint_solution.position.push_back(i[3](0));
        joint_solution.position.push_back(i[3](1));
        joint_solution.position.push_back(i[3](2));
        joint_solution.position.push_back(i[1](0));
        joint_solution.position.push_back(i[1](1));
        joint_solution.position.push_back(i[1](2));
        joint_solution.position.push_back(i[2](0));
        joint_solution.position.push_back(i[2](1));
        joint_solution.position.push_back(i[2](2));
        joint_solution.position.push_back(arm_ik(0));
        joint_solution.position.push_back(arm_ik(1));
        joint_solution.position.push_back(arm_ik(2));
        joint_solution.position.push_back(arm_ik(3));
        joint_solution.position.push_back(arm_ik(4));
        joint_solution.position.push_back(arm_ik(5));
        joint_solution.position.push_back(0.0);
        jointstate_pub_->publish(joint_solution);
        joint_solution.position.clear();
        // rviz2_ptr->VisualizeSupportPolygon(Kinematics::kinematics_solver->stance_legs_wrt_world[point_index], 0.02);
        rclcpp::sleep_for(std::chrono::milliseconds(del_t));
        // cout << "arm fixed" << endl;
    }
}

void Abc::PublishStatesLegsFixed(const vector<KDL::JntArray> &arm_ik, int del_t,
                                 const pair<Eigen::Vector4d, Eigen::Vector3d> &spot_pose,
                                 const vector<Eigen::Vector3d> &leg_ik)
{
    sensor_msgs::msg::JointState home_states;
    int point_index = -1;
    for (const auto &ik : arm_ik)
    {
        point_index++;
        global_id++;
        // cout << "id " << global_id << endl;
        home_states.header.stamp = this->now();
        home_states.name = {"translate_x", "translate_y", "translate_z", "rotate_roll", "rotate_pitch", "rotate_yaw",
                            "front_left_hip_x", "front_left_hip_y", "front_left_knee", "front_right_hip_x", "front_right_hip_y", "front_right_knee",
                            "rear_left_hip_x", "rear_left_hip_y", "rear_left_knee", "rear_right_hip_x", "rear_right_hip_y", "rear_right_knee",
                            "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4",
                            "arm_joint5", "arm_joint6", "arm_gripper"};
        home_states.position.push_back(spot_pose.first(0));
        home_states.position.push_back(spot_pose.first(1));
        home_states.position.push_back(spot_pose.first(2));
        home_states.position.push_back(spot_pose.second(0));
        home_states.position.push_back(spot_pose.second(1));
        home_states.position.push_back(spot_pose.second(2));
        home_states.position.push_back(leg_ik[0](0));
        home_states.position.push_back(leg_ik[0](1));
        home_states.position.push_back(leg_ik[0](2));
        home_states.position.push_back(leg_ik[3](0));
        home_states.position.push_back(leg_ik[3](1));
        home_states.position.push_back(leg_ik[3](2));
        home_states.position.push_back(leg_ik[1](0));
        home_states.position.push_back(leg_ik[1](1));
        home_states.position.push_back(leg_ik[1](2));
        home_states.position.push_back(leg_ik[2](0));
        home_states.position.push_back(leg_ik[2](1));
        home_states.position.push_back(leg_ik[2](2));
        home_states.position.push_back(ik(0));
        home_states.position.push_back(ik(1));
        home_states.position.push_back(ik(2));
        home_states.position.push_back(ik(3));
        home_states.position.push_back(ik(4));
        home_states.position.push_back(ik(5));
        home_states.position.push_back(0.0);
        jointstate_pub_->publish(home_states);
        home_states.position.clear();
        // rviz2_ptr->VisualizeSupportPolygon(Kinematics::kinematics_solver->stance_legs_wrt_world[0], 0.02);

        // Eigen::Vector4d arm_com_wrt_w = arm_com_solver->ComputeArmCOM(ik);
        // Eigen::Vector4d legs_com_wrt_w = legs_com_solver->ComputeQuadCOM(leg_ik, spot_pose.first);
        // Eigen::Vector4d com_wrt_w = ((arm_com_solver->arm_mass * arm_com_wrt_w) + (legs_com_solver->quadruped_mass * legs_com_wrt_w)) / (arm_com_solver->arm_mass + legs_com_solver->quadruped_mass);
        // com_wrt_w(2) = -0.34;
        Eigen::Vector4d com_wrt_w = com_solver->ComputeWholeSystemCOM(leg_ik, spot_pose, ik);
        com_wrt_w(2) = -0.31;
        Eigen::Vector4d centroid = spot_pose.first;
        centroid(2) = -0.31;
        rviz2_ptr->VisualizeEverything(Kinematics::kinematics_solver->stance_legs_wrt_world[0], com_wrt_w, centroid, 0.02);
        rclcpp::sleep_for(std::chrono::milliseconds(del_t));
        // cout << "leg fixed" << endl;
    }
}

void Abc::PublishHomeState()
{
    // home state publlisher
    sensor_msgs::msg::JointState home_states;
    for (int i = 0; i < 10; i++)
    {
        global_id++;
        // cout << "id " << global_id << endl;
        home_states.header.stamp = this->now();
        home_states.name = {"translate_x", "translate_y", "translate_z", "rotate_roll", "rotate_pitch", "rotate_yaw",
                            "front_left_hip_x", "front_left_hip_y", "front_left_knee", "front_right_hip_x", "front_right_hip_y", "front_right_knee",
                            "rear_left_hip_x", "rear_left_hip_y", "rear_left_knee", "rear_right_hip_x", "rear_right_hip_y", "rear_right_knee",
                            "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4",
                            "arm_joint5", "arm_joint6", "arm_gripper"};
        home_states.position.push_back(0.0);
        home_states.position.push_back(0.0);
        home_states.position.push_back(0.175);
        home_states.position.push_back(0.0);
        home_states.position.push_back(0.0);
        home_states.position.push_back(0.0);
        home_states.position.push_back(0.0);
        home_states.position.push_back(0.785);
        home_states.position.push_back(-1.57);
        home_states.position.push_back(0.0);
        home_states.position.push_back(0.785);
        home_states.position.push_back(-1.57);
        home_states.position.push_back(0.0);
        home_states.position.push_back(0.785);
        home_states.position.push_back(-1.57);
        home_states.position.push_back(0.0);
        home_states.position.push_back(0.785);
        home_states.position.push_back(-1.57);
        home_states.position.push_back(1.57);
        home_states.position.push_back(-0.53);
        home_states.position.push_back(2.35);
        home_states.position.push_back(0.0);
        home_states.position.push_back(0.785);
        home_states.position.push_back(0.0);
        home_states.position.push_back(0.0);
        jointstate_pub_->publish(home_states);
        home_states.position.clear();
        Eigen::Vector3d l_t(0.0, 0.785, -1.57);
        vector<Eigen::Vector3d> leg_ik = {l_t, l_t, l_t, l_t};

        // Eigen::Vector4d arm_com_wrt_w = arm_com_solver->ComputeArmCOM(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->initial_guess);
        // Eigen::Vector4d legs_com_wrt_w = legs_com_solver->ComputeQuadCOM(leg_ik, Kinematics::state_tracker->body_xyz);
        // Eigen::Vector4d com_wrt_w = ((arm_com_solver->arm_mass * arm_com_wrt_w) + (legs_com_solver->quadruped_mass * legs_com_wrt_w)) / (arm_com_solver->arm_mass + legs_com_solver->quadruped_mass);
        // com_wrt_w(2) = -0.31;
        // cout << "final solution = " << arm_com_wrt_w.adjoint() << endl;
        Eigen::Vector4d com_wrt_w = com_solver->ComputeWholeSystemCOM(leg_ik, make_pair(Kinematics::state_tracker->body_xyz, Kinematics::state_tracker->body_rpy),
                                                                      ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->initial_guess);
        com_wrt_w(2) = -0.31;
        Eigen::Vector4d centroid = Kinematics::state_tracker->body_xyz;
        centroid(2) = -0.31;
        rviz2_ptr->VisualizeEverything(Kinematics::kinematics_solver->stance_legs_wrt_world[0], com_wrt_w, centroid, 0.02);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}

vector<KDL::JntArray> Abc::LinePlanner(const Eigen::Matrix4d &goal, int num, string str, double del_t)
{
    Curves::Line line(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->pose_6_wrt_0.block<4, 1>(0, 3), goal.block<4, 1>(0, 3));
    double precision = 0.005;
    vector<KDL::JntArray> arm_ik;
    double num1 = static_cast<double>(num);
    vector<pair<Eigen::Vector4d, Eigen::Quaterniond>> line_path = line.Generate6DPoints(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->pose_6_wrt_0, goal, precision);
    for (const auto &pose : line_path)
    {
        Eigen::Matrix4d pose_transform;
        pose_transform.block<4, 1>(0, 3) = pose.first;
        pose_transform.block<3, 3>(0, 0) = pose.second.toRotationMatrix();
        pose_transform.block<1, 3>(3, 0).setZero();
        ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->pose_6_wrt_0 = pose_transform;
        T_e_wrt_b = pose_transform;
        KDL::Frame kdl_frame = Conversions::Transform_2KDL(pose_transform);
        ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->SolveIK(kdl_frame);
        arm_ik.push_back(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->q);
    }
    if (str == "M")
    {
        vector<Eigen::Vector3d> leg_ik = {Kinematics::state_tracker->leg_states[0].theta, Kinematics::state_tracker->leg_states[1].theta,
                                          Kinematics::state_tracker->leg_states[2].theta, Kinematics::state_tracker->leg_states[3].theta};
        pair<Eigen::Vector4d, Eigen::Vector3d> spot_pose = make_pair(Kinematics::state_tracker->body_xyz, Kinematics::state_tracker->body_rpy);
        PublishStatesLegsFixed(arm_ik, del_t, spot_pose, leg_ik);
        arm_ik.clear();
    }

    return arm_ik;
}

vector<KDL::JntArray> Abc::ArcPlanner(const Eigen::Matrix4d &p1, const Eigen::Matrix4d &p2, const Eigen::Matrix4d &p3, int num, const string &str, double del_t)
{
    vector<KDL::JntArray> arm_ik;

    Curves::Arc arc(p1.block<4, 1>(0, 3), p2.block<4, 1>(0, 3), p3.block<4, 1>(0, 3), "C");
    arc.point_generation = "default";
    vector<pair<Eigen::Vector4d, Eigen::Quaterniond>> arc_path = arc.Generate6DPoints(p1, p2, p3, num);
    arc.point_generation = "";
    for (const auto &pose : arc_path)
    {
        Eigen::Matrix4d pose_transform;
        pose_transform.block<4, 1>(0, 3) = pose.first;
        pose_transform.block<3, 3>(0, 0) = pose.second.toRotationMatrix();
        pose_transform.block<1, 3>(3, 0).setZero();
        ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->pose_6_wrt_0 = pose_transform;

        KDL::Frame kdl_frame = Conversions::Transform_2KDL(pose_transform);
        ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->SolveIK(kdl_frame);
        arm_ik.push_back(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->q);
    }
    if (str == "M")
    {
        vector<Eigen::Vector3d> leg_ik = {Kinematics::state_tracker->leg_states[0].theta, Kinematics::state_tracker->leg_states[1].theta,
                                          Kinematics::state_tracker->leg_states[2].theta, Kinematics::state_tracker->leg_states[3].theta};
        cout << "num of loop = " << arm_ik.size() << endl;
        pair<Eigen::Vector4d, Eigen::Vector3d> spot_pose = make_pair(Kinematics::state_tracker->body_xyz, Kinematics::state_tracker->body_rpy);
        PublishStatesLegsFixed(arm_ik, del_t, spot_pose, leg_ik);
        arm_ik.clear();
    }

    return arm_ik;
}

void Abc::test_circle(const Eigen::Matrix4d &goal)
{
    vector<pair<Eigen::Vector4d, Eigen::Vector3d>> spot_pose_array;
    PostureControl::PostureControl posture_planner(2.0, 0.007);
    vector<Eigen::Vector4d> com_array;
    vector<vector<Eigen::Vector3d>> leg_configuration_path = posture_planner.PerformCircles(goal, 0.025, 2, com_array, spot_pose_array);
    // PublishStatesArmFixed(spot_pose_array, leg_configuration_path, 10);
    // BD
    PublishStates(posture_planner.arm_planner_solver->arm_ik, leg_configuration_path, spot_pose_array, 10, com_array);
    this->T_e_wrt_b = posture_planner.arm_planner_solver->T_e_wrt_b;
}

void Abc::Translate_test(double x, double z, double d, int num, const Eigen::Matrix4d &goal, string str)
{
    PostureControl::PostureControl posture_planner(0.5, 0.007);
    // Gaits::Gaits gait_solver(0.5, 0.007);
    for (int i = 0; i < 1; i++)
    {
        for (double rpv = d; rpv <= d; rpv += 3.0)
        {
            Eigen::Vector4d rpv1(-(x * rpv), 0.0, -(z * rpv), 1);
            Eigen::Vector4d rpv2((x * rpv), -0, (z * rpv), 1);
            Eigen::Vector4d rpv3((x * rpv), 0, -(z * rpv), 1);
            Eigen::Vector4d rpv4((x * rpv), -0, (z * rpv), 1);
            Eigen::Vector4d rpv5((x * rpv), 0, -(z * rpv), 1);
            Eigen::Vector4d rpv6((x * rpv), -0, (z * rpv), 1);
            Eigen::Vector4d rpv7((x * rpv), 0, -(z * rpv), 1);
            Eigen::Vector4d rpv8((x * rpv), -0, (z * rpv), 1);

            vector<Eigen::Vector4d> rpv_vector_ = {rpv1, rpv2, rpv3, rpv4, rpv5, rpv6, rpv7, rpv8}, rpv_vector;

            for (int i = 0; i < num; i++)
                rpv_vector.push_back(rpv_vector_[i]);
            vector<pair<Eigen::Vector4d, Eigen::Vector3d>> spot_pose_array;
            vector<Eigen::Vector4d> com_array;
            vector<vector<Eigen::Vector3d>> leg_configuration_path = posture_planner.PerformTranslation(goal, rpv_vector, com_array, spot_pose_array, FootHold::Reset);
            if (str == "BD")
                PublishStates(posture_planner.arm_planner_solver->arm_ik, leg_configuration_path, spot_pose_array, 90, com_array);

            // Eigen::Matrix4d dummy_pose;
            // Eigen::Matrix4d pose1 = Conversions::KDL_2Transform(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->sixDpose);
            // vector<Eigen::Vector4d> cp = gait_solver.leg_planner_solver->arm_planner_solver->GenerateBSPlineCP(goal, dummy_pose, 0.1, true);
            // Curves::BSpline spine_arm(cp, 4);
            // gait_solver.leg_planner_solver->arm_planner_solver->arm_ik.clear();
            // gait_solver.leg_planner_solver->arm_planner_solver->BSplinePlanner(spine_arm, goal, spot_pose_array.size(), pose1);
            // arm_ik_array.insert(arm_ik_array.begin(), gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.begin(),
            //                     gait_solver->leg_planner_solver->arm_planner_solver->arm_ik.end());

            // if (str == "BD")
            //     PublishStates(posture_planner.arm_planner_solver->arm_ik, leg_configuration_path, spot_pose_array, 90, com_array);
            // if (str == "D")
            //     PublishStates(posture_planner.arm_planner_solver->arm_ik, leg_configuration_path, spot_pose_array, 90, com_array);
            // else
            //     PublishStatesArmFixed(spot_pose_array, leg_configuration_path, ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->initial_guess, 50);
        }
    }
    this->T_e_wrt_b = posture_planner.arm_planner_solver->T_e_wrt_b;
    this->T_e_wrt_b = ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->pose_6_wrt_0;
}

void Abc::RP(double r, double p, int num, const Eigen::Matrix4d &goal, string str)
{
    PostureControl::PostureControl posture_planner(0.5, 0.005);
    vector<Eigen::Vector4d> com_array;

    vector<int> seq1 = {0, 1, 2};

    Eigen::Vector3d rpy1(r, p, 0);
    Eigen::Vector3d rpy2(-r, -p, 0);
    Eigen::Vector3d rpy3(r, p, 0);
    Eigen::Vector3d rpy4(-r, -p, 0);
    Eigen::Vector3d rpy5(r, p, 0);
    Eigen::Vector3d rpy6(-r, -p, 0);
    Eigen::Vector3d rpy7(r, p, 0);
    Eigen::Vector3d rpy8(-r, -p, 0);
    Eigen::Vector3d rpy9(0.0, 0.0, 0.0);
    vector<Eigen::Vector3d> rpy_vector_ = {rpy1, rpy2, rpy3, rpy4, rpy5, rpy6, rpy7, rpy8, rpy9}, rpy_vector;
    vector<vector<int>> seq_vector_ = {seq1, seq1, seq1, seq1, seq1, seq1, seq1, seq1, seq1}, seq_vector;

    for (int i = 0; i < num - 1; i++)
    {
        rpy_vector.push_back(rpy_vector_[i]);
        seq_vector.push_back(seq_vector_[i]);
    }
    if (num > 3)
    {
        rpy_vector.push_back(rpy9);
        seq_vector.push_back(seq1);
    }
    vector<pair<Eigen::Vector4d, Eigen::Vector3d>> spot_pose_array;
    vector<vector<Eigen::Vector3d>> leg_configuration_path = posture_planner.PerformRPY(goal, rpy_vector, seq_vector, com_array, spot_pose_array, FootHold::Reset);

    if (str == "BD")
        PublishStates(posture_planner.arm_planner_solver->arm_ik, leg_configuration_path, spot_pose_array, 1.5, com_array);
    else
        PublishStatesArmFixed(spot_pose_array, leg_configuration_path, ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->initial_guess, 8);
    // this->T_e_wrt_b = posture_planner.arm_planner_solver->T_e_wrt_b;
    this->T_e_wrt_b = ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->pose_6_wrt_0;
}

void Abc::RPY(double t, int p, int y, int num, double r, const Eigen::Matrix4d &goal, string str, double dt)
{
    // PostureControl::PostureControl posture_planner(0.5, 0.005);
    Gaits::Gaits gait_solver(0.1, 0.0025);
    vector<Eigen::Vector4d> com_array;
    vector<int> seq1 = {2 * y, 1 * p, 0};
    vector<int> seq2 = {2 * y, 1 * p, 0};
    vector<int> seq3 = {2 * y, 1 * p, 0};
    vector<int> seq4 = {2 * y, 1 * p, 0};
    vector<int> seq5 = {2 * y, 1 * p, 0};
    vector<int> seq6 = {2 * y, 1 * p, 0};
    vector<int> seq7 = {2 * y, 1 * p, 0};
    vector<int> seq8 = {2 * y, 1 * p, 0};
    vector<int> seq9 = {2 * y, 1 * p, 0};

    for (double angle = t; angle <= t; angle += 3.0)
    {
        Eigen::Vector3d rpy1(r, angle, angle);
        Eigen::Vector3d rpy2(r, -angle, -angle);
        Eigen::Vector3d rpy3(0.0, angle, angle);
        Eigen::Vector3d rpy4(0.0, -angle, -angle);
        Eigen::Vector3d rpy5(0.0, angle, angle);
        Eigen::Vector3d rpy6(0.0, -angle, -angle);
        Eigen::Vector3d rpy7(0.0, angle, angle);
        Eigen::Vector3d rpy8(0.0, -angle, -angle);
        Eigen::Vector3d rpy9(0.0, 0.0, 0.0);
        vector<Eigen::Vector3d> rpy_vector_ = {rpy1, rpy2, rpy3, rpy4, rpy5, rpy6, rpy7, rpy8, rpy9}, rpy_vector;
        vector<vector<int>> seq_vector_ = {seq1, seq2, seq3, seq4, seq5, seq6, seq7, seq8, seq9}, seq_vector;
        // gait_solver.leg_planner_solver->PerformRPY
        for (int i = 0; i < num - 1; i++)
        {
            rpy_vector.push_back(rpy_vector_[i]);
            seq_vector.push_back(seq_vector_[i]);
        }
        if (num > 3)
        {
            rpy_vector.push_back(rpy9);
            seq_vector.push_back(seq9);
        }
        vector<pair<Eigen::Vector4d, Eigen::Vector3d>> spot_pose_array;
        vector<vector<Eigen::Vector3d>> leg_configuration_path = gait_solver.leg_planner_solver->PerformRPY(goal, rpy_vector, seq_vector, com_array, spot_pose_array, FootHold::Reset);
        if (str == "BD")
            PublishStates(gait_solver.leg_planner_solver->arm_planner_solver->arm_ik, leg_configuration_path, spot_pose_array, dt, com_array);

        if (str == "D")
        {
            vector<Eigen::Vector4d> cp = gait_solver.leg_planner_solver->arm_planner_solver->GenerateBSPlineCP(goal, this->T_e_wrt_b, 0.1, false);
            Curves::BSpline spine_arm(cp, 4);
            gait_solver.leg_planner_solver->arm_planner_solver->arm_ik.clear();
            gait_solver.leg_planner_solver->arm_planner_solver->BSplinePlanner(spine_arm, goal, spot_pose_array.size(), this->T_e_wrt_b);
            PublishStates(gait_solver.leg_planner_solver->arm_planner_solver->arm_ik, leg_configuration_path, spot_pose_array, dt, com_array);
        }
        else if (str != "BD")
            PublishStatesArmFixed(spot_pose_array, leg_configuration_path, ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->initial_guess, 8);
        // this->T_e_wrt_b = posture_planner.arm_planner_solver->T_e_wrt_b;
        this->T_e_wrt_b = ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->pose_6_wrt_0;
    }
}

void Abc::RPYT(double angle, double dist, int r, int p, int y, int num, const Eigen::Matrix4d &goal, double dt, string str)
{
    Gaits::Gaits gait_solver(0.1, 0.0025);
    vector<Eigen::Vector4d> com_array;
    vector<int> seq1 = {2 * y, 1 * p, 0 * r};
    vector<int> seq2 = {2 * y, 1 * p, 0 * r};
    vector<int> seq3 = {2 * y, 1 * p, 0 * r};
    vector<int> seq4 = {2 * y, 1 * p, 0 * r};
    vector<int> seq5 = {2 * y, 1 * p, 0 * r};
    vector<int> seq6 = {2 * y, 1 * p, 0 * r};
    vector<int> seq7 = {2 * y, 1 * p, 0 * r};
    vector<int> seq8 = {2 * y, 1 * p, 0 * r};
    vector<int> seq9 = {2 * y, 1 * p, 0 * r};

    // for (double angle = t; angle <= t; angle += 3.0)
    // {
    Eigen::Vector3d rpy1(0.0, Kinematics::state_tracker->body_rpy(1), angle);
    Eigen::Vector3d rpy2(0.0, Kinematics::state_tracker->body_rpy(1), -angle);
    Eigen::Vector3d rpy3(0.0, Kinematics::state_tracker->body_rpy(1), angle);
    Eigen::Vector3d rpy4(0.0, Kinematics::state_tracker->body_rpy(1), -angle);
    Eigen::Vector3d rpy5(0.0, Kinematics::state_tracker->body_rpy(1), angle);
    Eigen::Vector3d rpy6(0.0, Kinematics::state_tracker->body_rpy(1), -angle);
    Eigen::Vector3d rpy7(0.0, Kinematics::state_tracker->body_rpy(1), angle);
    Eigen::Vector3d rpy8(0.0, Kinematics::state_tracker->body_rpy(1), -angle);
    Eigen::Vector3d rpy9(0.0, 0.0, 0.0);
    vector<Eigen::Vector3d> rpy_vector_ = {rpy1, rpy2, rpy3, rpy4, rpy5, rpy6, rpy7, rpy8, rpy9}, rpy_vector;
    vector<vector<int>> seq_vector_ = {seq1, seq2, seq3, seq4, seq5, seq6, seq7, seq8, seq9}, seq_vector;
    // gait_solver.leg_planner_solver->PerformRPY
    for (int i = 0; i < num - 1; i++)
    {
        rpy_vector.push_back(rpy_vector_[i]);
        seq_vector.push_back(seq_vector_[i]);
    }
    if (num > 3)
    {
        rpy_vector.push_back(rpy9);
        seq_vector.push_back(seq9);
    }
    vector<pair<Eigen::Vector4d, Eigen::Vector3d>> spot_pose_array;
    vector<vector<Eigen::Vector3d>> leg_configuration_path;
    Eigen::Vector4d rpv(0, 0, dist, 1.0);
    for (int i = 0; i < rpy_vector.size(); i++)
    {
        // Eigen::Vector3d &rpy, const vector<int> &seq, Eigen::Vector4d &rp,
        //                                                                vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_pose_array,
        //                                                                const FootHold &strategy
        vector<vector<Eigen::Vector3d>> lik = gait_solver.leg_planner_solver->GenerateRPYTranslation(rpy_vector[i], seq_vector[i], rpv, spot_pose_array, FootHold::Vertical);
        if (leg_configuration_path.empty())
            leg_configuration_path.insert(leg_configuration_path.begin(), lik.begin(), lik.end());
        else
            leg_configuration_path.insert(leg_configuration_path.end(), lik.begin(), lik.end());
    }
    gait_solver.leg_planner_solver->arm_planner_solver->arm_ik.clear();
    gait_solver.leg_planner_solver->arm_planner_solver->ChickenHeadPlanner(goal, spot_pose_array);
    // arm_ik.insert(arm_ik.end(), leg_planner_solver->arm_planner_solver->arm_ik.begin(), leg_planner_solver->arm_planner_solver->arm_ik.end());

    if (str == "BD")
        PublishStates(gait_solver.leg_planner_solver->arm_planner_solver->arm_ik, leg_configuration_path, spot_pose_array, dt, com_array);

    vector<Eigen::Vector4d> cp = gait_solver.leg_planner_solver->arm_planner_solver->GenerateBSPlineCP(goal, this->T_e_wrt_b, 0.1, false);
    Curves::BSpline spine_arm(cp, 4);
    gait_solver.leg_planner_solver->arm_planner_solver->arm_ik.clear();
    gait_solver.leg_planner_solver->arm_planner_solver->BSplinePlanner(spine_arm, goal, spot_pose_array.size(), this->T_e_wrt_b);

    if (str == "D")
        PublishStates(gait_solver.leg_planner_solver->arm_planner_solver->arm_ik, leg_configuration_path, spot_pose_array, dt, com_array);
    else if (str != "BD")
        PublishStatesArmFixed(spot_pose_array, leg_configuration_path, ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->initial_guess, 8);
    // this->T_e_wrt_b = posture_planner.arm_planner_solver->T_e_wrt_b;
    this->T_e_wrt_b = gait_solver.leg_planner_solver->arm_planner_solver->T_e_wrt_b;
    // T_e_wrt_b = gait_solver.leg_planner_solver->arm_planner_solver->T_e_wrt_b;
    // }
}

void Abc::test3(double t, int num, const Eigen::Matrix4d &goal, string str)
{
    PostureControl::PostureControl posture_planner(0.5, 0.005);
    vector<Eigen::Vector4d> com_array;
    vector<pair<Eigen::Vector4d, Eigen::Vector3d>> spot_pose_array;
    vector<vector<Eigen::Vector3d>> leg_configuration_path = posture_planner.Perform2AxisBodyRotation(goal, t, num, com_array, spot_pose_array);
    if (str == "BD")
        PublishStates(posture_planner.arm_planner_solver->arm_ik, leg_configuration_path, spot_pose_array, 9, com_array);
    else
        PublishStatesArmFixed(spot_pose_array, leg_configuration_path, ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->initial_guess, 9);
    this->T_e_wrt_b = posture_planner.arm_planner_solver->T_e_wrt_b;
}

vector<KDL::JntArray> Abc::ComputeArmJointStates(const Eigen::Matrix4d &goal, const vector<pair<Eigen::Vector4d, Eigen::Vector3d>> &spot_pose_array)
{
    vector<KDL::JntArray> arm_ik = LinePlanner(goal, 100, "", 10);
    vector<Eigen::Vector3d> leg_ik = {Kinematics::state_tracker->leg_states[0].theta, Kinematics::state_tracker->leg_states[1].theta,
                                      Kinematics::state_tracker->leg_states[2].theta, Kinematics::state_tracker->leg_states[3].theta};

    PublishStatesLegsFixed(arm_ik, 20, spot_pose_array[0], leg_ik);
    arm_ik.clear();

    // vector<KDL::JntArray> arm_ik;
    Eigen::Matrix4d T1wrtW, TBwrt1, TEwrtB = goal, TEwrtW;
    T1wrtW.block<4, 1>(0, 3) = spot_pose_array[0].first;
    T1wrtW.block<3, 3>(0, 0) = CommonMathsSolver::OrientationNTransformaton::ComputeR(spot_pose_array[0].second);
    T1wrtW.block<1, 3>(3, 0).setZero();
    TBwrt1 << 1, 0, 0, 0.29,
        0, 1, 0, 0,
        0, 0, 1, 0.08,
        0, 0, 0, 1;
    TEwrtW = T1wrtW * TBwrt1 * TEwrtB; // TEwrtW = constant

    for (const auto &spot_pose : spot_pose_array)
    {
        T1wrtW.block<4, 1>(0, 3) = spot_pose.first;
        T1wrtW.block<3, 3>(0, 0) = CommonMathsSolver::OrientationNTransformaton::ComputeR(spot_pose.second);
        T1wrtW.block<1, 3>(3, 0).setZero();

        TEwrtB = (T1wrtW * TBwrt1).inverse() * TEwrtW;
        T_e_wrt_b = TEwrtB;
        ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->pose_6_wrt_0 = TEwrtB;
        KDL::Frame kdl_frame = Conversions::Transform_2KDL(TEwrtB);
        ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->SolveIK(kdl_frame);
        arm_ik.push_back(ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->q);
    }
    return arm_ik;
}

bool Abc::cb(const std::shared_ptr<custom_interfaces::srv::TaskTest::Request> req,
             std::shared_ptr<custom_interfaces::srv::TaskTest::Response> res)
{
    res->flag = true;
    vector<Eigen::Vector4d> com_array;
    vector<KDL::JntArray> walking_arm_ik;
    Gaits::Gaits gait_solver(0.1, 0.0025);
    Gaits::ComplexPostures complex_posture_solver1(0.05, 0.0025), complex_posture_solver2(0.1, 0.0025);
    vector<pair<Eigen::Vector4d, Eigen::Vector3d>> spot_pose_array;
    vector<vector<Eigen::Vector3d>> leg_configuration_path;
    vector<KDL::JntArray> arm_ik;
    double distance = 0.2, stride_h = 0.2, stride_s = 0.1;
    Eigen::Vector3d direction(1, 0, 0);

    Eigen::Matrix4d goal, goal1, goal2, goal3, goal_bd, goal_pd;
    goal << -1, 0, 0, 0.4, // away from me
        0, -1, 0, -0.0,
        0, 0, 1, -0.10,
        0, 0, 0, 1;

    goal << 0, 1, 0, 0.7, // front
        -1, 0, 0, -0.0,
        0, 0, 1, -0.3,
        0, 0, 0, 1;

    goal.block<3, 3>(0, 0) = ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->pose_6_wrt_0.block<3, 3>(0, 0);
    goal.block<1, 3>(3, 0).setZero();
    goal(0, 3) = 0.7;
    goal(1, 3) = 0.0;
    goal(2, 3) = -0.3;
    goal(3, 3) = 1.0;
    // vector<KDL::JntArray> arm_move = LinePlanner(goal, 5, "M", 3);
    // rclcpp::sleep_for(std::chrono::milliseconds(500));
    goal1 << 1, 0, 0, -0.3,
        0, 1, 0, 0.0,
        0, 0, 1, 0.2,
        0, 0, 0, 1;
    // arm_move = LinePlanner(goal, 5, "M", 15);
    // rclcpp::sleep_for(std::chrono::milliseconds(500));
    // goal2 << 0, 1, 0, -0.1,
    //     0, 0, -1, 0.55,
    //     -1, 0, 0, -0.1,
    //     0, 0, 0, 1;

    goal2 << 0, 1, 0, 0.4,
        0, 0, -1, 0.3,
        -1, 0, 0, -0.1,
        0, 0, 0, 1;

    goal3 << 0, 1, 0, 0.4,
        0, 0, -1, -0.3,
        -1, 0, 0, -0.1,
        0, 0, 0, 1;

    // arm_move = LinePlanner(goal, 5, "M", 15);
    // rclcpp::sleep_for(std::chrono::milliseconds(500));
    goal_bd << 0, 1, 0, 0.7,
        0, 0, -1, 0,
        -1, 0, 0, -0.15,
        0, 0, 0, 1;

    goal_pd << 0, 1, 0, 0.7,
        0, 0, -1, 0,
        -1, 0, 0, -0.0,
        0, 0, 0, 1;
    // vector<KDL::JntArray> arm_move = LinePlanner(goal, 5, "M", 15);
    // arm_move = LinePlanner(goal, 5, "M", 15);
    // arm_move = ArcPlanner(goal, goal1, goal2, 500, "M", 15);
    // vector<KDL::JntArray> arm_move = LinePlanner(goal3, 5, "M", 10);
    // Translate_test(0, 1, 0.01, 1, ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->pose_6_wrt_0, "");
    // Translate_test(0, 1, 0.1, 1, T_e_wrt_b, "BD");
    // Translate_test(0, 1, 0.13, 1, gait_solver.leg_planner_solver->arm_planner_solver->T_e_wrt_b, "BD");
    // // bowing
    // RPY(15.0, 1, 0, 2, 0, goal3, "D", 15);
    // RPY(15.0, 0, 1, 2, 0, goal3, "BD", 5);
    // RPYT(15, 0.15, 0, 0, 1, 7, T_e_wrt_b, 7.5, "BD");

    // leg_configuration_path = gait_solver.DancingCrawlGait(T_e_wrt_b, com_array, walking_arm_ik, distance, stride_h, 1, 10, "", spot_pose_array);
    // PublishStates(walking_arm_ik, leg_configuration_path, spot_pose_array, 4.5, com_array);
    // leg_configuration_path.clear();
    // spot_pose_array.clear();
    // walking_arm_ik.clear();
    // com_array.clear();
    // T_e_wrt_b = gait_solver.leg_planner_solver->arm_planner_solver->T_e_wrt_b;
    // cout << "error 1 pose " << T_e_wrt_b << endl;
    // RPY(0.0, 1, 1, 2, 0, T_e_wrt_b, "BD", 3.5);
    // cout << "error 2 pose " << T_e_wrt_b << endl;
    // RPY(15.0, 1, 0, 2, 0, T_e_wrt_b, "BD", 3.5);
    // RPYT(15, 0.15, 0, 0, 1, 7, T_e_wrt_b, 7.5, "BD");
    Translate_test(0, 1, 0.01, 1, ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->pose_6_wrt_0, "");
    Translate_test(0, 1, 0.11, 1, T_e_wrt_b, "BD");
    RPY(15.0, 1, 0, 2, 0, goal3, "D", 15);
    RPY(15.0, 0, 1, 2, 0, goal3, "BD", 5);
    // spot dance
    Eigen::Vector3d rpy2(0, 0, D2R * (-30)), rpy3(0, 0, D2R * (-15));
    leg_configuration_path = complex_posture_solver1.PYT_Dance(0.3, 0.075, -15, 0.125, spot_pose_array, walking_arm_ik, goal2, goal3, rpy2, rpy3);
    PublishStates(walking_arm_ik, leg_configuration_path, spot_pose_array, 2.5, com_array);
    leg_configuration_path.clear();
    spot_pose_array.clear();
    walking_arm_ik.clear();
    com_array.clear();

    // RP(0.5, 1, 2, goal3, "BD");
    // for (int j = 0; j < 10; j++)
    //     if (j % 2 == 0 && j == 0)
    //         for (int i = 1; i <= 20; i++)
    //             RP(i / 2.0, i, 2, T_e_wrt_b, "BD");
    //     else if (j % 2 == 1)
    //         for (int i = 19; i >= -20; i--)
    //             RP(i / 2.0, i, 2, T_e_wrt_b, "BD");
    //     else
    //         for (int i = -19; i <= 20; i++)
    //             RP(i / 2.0, i, 2, T_e_wrt_b, "BD");
    // FORWARD CRAWL
    // leg_configuration_path = gait_solver.CrawlGait(goal, com_array, walking_arm_ik, distance, stride_h, 1, "", spot_pose_array);
    // PublishStates(walking_arm_ik, leg_configuration_path, spot_pose_array, 10, com_array);
    // leg_configuration_path.clear();
    // spot_pose_array.clear();
    // walking_arm_ik.clear();
    // com_array.clear();
    // RPY(double t, int p, int y, int num, double r, const Eigen::Matrix4d &goal, string str)
    // RPY(15, 1, 0, 7, 0, goal3, "BD");
    // // // // BACKWARD CRAWL
    // leg_configuration_path = gait_solver.ReverseCrawlGait(T_e_wrt_b, com_array, walking_arm_ik, 1 * distance, stride_h, 2, direction, spot_pose_array);
    // PublishStates(walking_arm_ik, leg_configuration_path, spot_pose_array, 20, com_array);
    // leg_configuration_path.clear();
    // spot_pose_array.clear();
    // walking_arm_ik.clear();
    // com_array.clear();
    // cout << "heyyyyyyyyyyyyyyyyyyy" << endl;
    // // // // SIDE POSITIVE
    // leg_configuration_path = gait_solver.CrawlGait(0.75 * distance, stride_h, 2, "SP", spot_pose_array);
    // arm_ik = ComputeArmJointStates(T_e_wrt_b, spot_pose_array);
    // PublishStates(arm_ik, leg_configuration_path, spot_pose_array, 20);
    // leg_configuration_path.clear();
    // spot_pose_array.clear();
    // arm_ik.clear();
    // // // PY
    // RPY(12.0, 1, 1, 5, 0, goal3, "BD");
    // // // // 2D PY
    // test3(12, 3, T_e_wrt_b, "BD");
    // // SPIN
    // //
    // PROBLEM
    // leg_configuration_path = gait_solver.CrawlSpinningGait(T_e_wrt_b, com_array, walking_arm_ik, -60.0, 0.1, 1.0, 5, spot_pose_array);
    // PublishStates(walking_arm_ik, leg_configuration_path, spot_pose_array, 5, com_array);
    // leg_configuration_path.clear();
    // spot_pose_array.clear();
    // walking_arm_ik.clear();
    // com_array.clear();
    // Translate_test(0, 1, 0.1, 1, T_e_wrt_b, "BD");
    // test_circle(T_e_wrt_b);
    // leg_configuration_path = gait_solver.CrawlGait(0.5 * distance, stride_h, 2, "SP", spot_pose_array);
    // arm_ik = ComputeArmJointStates(T_e_wrt_b, spot_pose_array);
    // PublishStates(arm_ik, leg_configuration_path, spot_pose_array, 25);
    // leg_configuration_path.clear();
    // spot_pose_array.clear();
    // arm_ik.clear();
    // leg_configuration_path = gait_solver.CrawlGait(1 * distance, stride_h, 2, "", spot_pose_array);
    // arm_ik = ComputeArmJointStates(T_e_wrt_b, spot_pose_array);
    // PublishStates(arm_ik, leg_configuration_path, spot_pose_array, 15);
    // leg_configuration_path.clear();
    // spot_pose_array.clear();
    // arm_ik.clear();
    // Translate_test(1, 1, 0.06, 1, goal, "BD");
    // rclcpp::sleep_for(std::chrono::milliseconds(2000));
    // Translate_test(0, 1, 0.15, 5, T_e_wrt_b, "BD");
    // // // rclcpp::sleep_for(std::chrono::milliseconds(2000));

    // leg_configuration_path = gait_solver.ReverseCrawlSpinningGait(T_e_wrt_b, com_array, walking_arm_ik, 90.0, 0.1, 1.0, 4, spot_pose_array);
    // PublishStates(walking_arm_ik, leg_configuration_path, spot_pose_array, 5, com_array);
    // leg_configuration_path.clear();
    // spot_pose_array.clear();
    // walking_arm_ik.clear();
    // com_array.clear();
    // Translate_test(0, 1, -0.1, 1, T_e_wrt_b, "BD");
    // Translate_test(0, 1, 0.15, 4, T_e_wrt_b, "BD");
    // leg_configuration_path = gait_solver.CrawlGait(1 * distance, stride_h, 2, "", spot_pose_array);
    // arm_ik = ComputeArmJointStates(T_e_wrt_b, spot_pose_array);
    // PublishStates(arm_ik, leg_configuration_path, spot_pose_array, 15);
    // leg_configuration_path.clear();
    // spot_pose_array.clear();
    // arm_ik.clear();
    // Translate_test(0, 1, -0.05, 1, T_e_wrt_b, "BD");
    // goal << 0, 1, 0, 0.0,
    //     0, 0, -1, -0.3,
    //     -1, 0, 0, 0.2,
    //     0, 0, 0, 1;
    // arm_move = LinePlanner(goal, 100, "M", 15);

    // goal << 1, 0, 0, 0.4,
    //     0, 0, 1, 0.0,
    //     0, -1, 0, 0.2,
    //     0, 0, 0, 1;
    // arm_move = LinePlanner(goal, 100, "M", 15);

    // goal << 0, 1, 0, 0.3,
    //     0, 0, -1, 0.0,
    //     -1, 0, 0, 0.2,
    //     0, 0, 0, 1;
    // arm_move = LinePlanner(goal, 100, "M", 30);

    // Eigen::Vector4d A(0, 0, 0, 1), B(1, 3, 0, 1), C(0, 4, 0, 1), D(-1, 4, 0, 1), E(-2, 3, 0, 1), F(-5, 2, 0, 1), G(-8, 0, 0, 1);
    // Curves::Bezzier bezzier(vector<Eigen::Vector4d>{A, B, C, D, E, F, G});
    // int i = 20;
    // vector<Eigen::Vector4d> points = bezzier.Generate3DPoints(i);
    // cout << "bezzier" << endl;
    // for (int i = 0; i < 19; i++)
    //     cout << CommonMathsSolver::Geometry::Distance2P(points[i], points[i + 1]) << endl;

    //********************************************************************************BD*************************************************************
    // Translate_test(0, 1, 0.01, 1, ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->pose_6_wrt_0, "");
    // Translate_test(0, 1, 0.075, 1, T_e_wrt_b, "BD");
    // for (int i = 0; i < 0; i++)
    // {
    //     leg_configuration_path = complex_posture_solver1.RP_Dance((i % 2 == 0) ? 1 : 2, 0.2, -0.1, 0.1, 7, 14, spot_pose_array, walking_arm_ik, goal);
    //     PublishStates(walking_arm_ik, leg_configuration_path, spot_pose_array, 10, com_array);
    //     leg_configuration_path.clear();
    //     walking_arm_ik.clear();
    //     spot_pose_array.clear();
    //     com_array.clear();
    //     // rclcpp::sleep_for(std::chrono::milliseconds(500));
    //     // if (i == 0)
    //     // {
    //     //     Translate_test(0, 1, 0.1, 1, T_e_wrt_b, "BD");
    //     //     test_circle(T_e_wrt_b);
    //     //     Translate_test(0, 1, -0.1, 1, T_e_wrt_b, "BD");
    //     // }
    // }

    // ********************************************** JUMPING DANCE ***************************************
    // Translate_test(0, 1, 0.01, 1, ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->pose_6_wrt_0, "");
    // Translate_test(0, 1, 0.075, 1, T_e_wrt_b, "BD");
    // leg_configuration_path = complex_posture_solver1.P_Dance(0, 0, 0.1, 0.1, 20, spot_pose_array, walking_arm_ik, goal_pd, goal1);
    // PublishStates(walking_arm_ik, leg_configuration_path, spot_pose_array, 5.0, com_array);
    // leg_configuration_path.clear();
    // walking_arm_ik.clear();
    // spot_pose_array.clear();
    // com_array.clear();
    // Translate_test(0, 1, 0.05, 1, complex_posture_solver1.gait_solver->leg_planner_solver->arm_planner_solver->T_e_wrt_b, "BD");
    // test_circle(T_e_wrt_b);

    // **************************************  BOWING *******************************************
    // Translate_test(0, 1, 0.01, 1, ArmKinematics::arm_solver[ArmKinematics::ArmLinkNames::link6]->pose_6_wrt_0, "");
    // Translate_test(0, 1, 0.075, 1, T_e_wrt_b, "BD");
    // string str11 = "LL";
    // leg_configuration_path = complex_posture_solver1.BowDown(0.35, 0.075, 10.0, 0.3, str11, 100, 0.1, 0.2, spot_pose_array, walking_arm_ik, goal_bd);
    // PublishStates(walking_arm_ik, leg_configuration_path, spot_pose_array, 10, com_array);
    // leg_configuration_path.clear();
    // walking_arm_ik.clear();
    // spot_pose_array.clear();
    // com_array.clear();
    // string str12 = "RL";
    // leg_configuration_path = complex_posture_solver2.BowDown(0.35, 0.05, 10.0, 0.3, str12, 100, 0.1, 0.2, spot_pose_array, walking_arm_ik, goal_bd);
    // gait_solver.leg_planner_solver->arm_planner_solver->ReplacArmAngles(spot_pose_array.size());
    // PublishStates(walking_arm_ik, leg_configuration_path, spot_pose_array, 10, com_array);
    // leg_configuration_path.clear();
    // spot_pose_array.clear();
    // walking_arm_ik.clear();
    // return true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Abc>();
    node->PublishHomeState();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
