#include "environment/Environment_1.hpp" 
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto visualizer = std::make_shared<EnvironmentSetUp::StairVisualizer>();

    Eigen::Vector3d dimensions1(0.3, 1.0, 0.15), first_point1(2.0, 0.0, -0.255);
    Eigen::Vector3d dimensions2(1.0, 0.45, 0.15), first_point2(0.0, -0.85, -0.255);
    Eigen::Vector3d dimensions3(0.3, 1.0, 0.15), first_point3(-2.0, 2.0, -0.255);

    Eigen::Vector3d position(0.0, 0.0, -0.34);
    Eigen::Vector3d dimensions4(11, 11, 0.01);

    Eigen::Vector3d position5(3.0, 3.0, -0.2275);
    Eigen::Vector3d dimensions5(2, 1, 0.20);

    Eigen::Vector3d position6(0.0, -5.5, 0.425);
    Eigen::Vector3d dimensions6(11, 0.01, 1.5);

    Eigen::Vector3d position7(-3.0, -3.0, 0.1);
    Eigen::Vector3d dimensions7(2, 1, 0.8);
    Eigen::Vector3d position12(-3.0, 1.0, 0.1);
    Eigen::Vector3d dimensions12(2, 1, 0.8);
    Eigen::Vector3d position8(-3.0, -1.0, -0.2);
    Eigen::Vector3d dimensions8(2, 3, 0.2);

    Eigen::Vector3d position9(0.0, -5.0, 1.175);
    Eigen::Vector3d dimensions9(11, 1, 0.01);
    Eigen::Vector3d position10(-5.0, -0.0, 1.175);
    Eigen::Vector3d dimensions10(1, 11, 0.01);
    Eigen::Vector3d position11(5.0, -2.5, 1.175);
    Eigen::Vector3d dimensions11(1, 6, 0.01);

    rclcpp::Rate r(10); // 1 Hz
    for (int i = 0; i < 10000; i++)
    {
        visualizer->PublishStairs(dimensions1, first_point1, "XZ", 0);
        r.sleep();
        visualizer->PublishStairs(dimensions2, first_point2, "yZ", 1);
        r.sleep();
        visualizer->PublishStairs(dimensions3, first_point3, "xZ", 2);
        r.sleep();
        visualizer->PublishSingleCuboid(position, dimensions4, 3);
        r.sleep();
        visualizer->PublishSingleCuboid(position5, dimensions5, 4);
        r.sleep();
        visualizer->PublishSingleCuboid(position6, dimensions6, 5);
        r.sleep();
        visualizer->PublishSingleCuboid(position7, dimensions7, 6);
        r.sleep();
        visualizer->PublishSingleCuboid(position8, dimensions8, 7);
        r.sleep();
        visualizer->PublishSingleCuboid(position9, dimensions9, 8);
        r.sleep();
        visualizer->PublishSingleCuboid(position10, dimensions10, 9);
        r.sleep();
        visualizer->PublishSingleCuboid(position11, dimensions11, 10);
        r.sleep();
        visualizer->PublishSingleCuboid(position12, dimensions12, 11);

        r.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
