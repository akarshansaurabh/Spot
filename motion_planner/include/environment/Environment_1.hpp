#ifndef ENVIRONMENT_1_HPP
#define ENVIRONMENT_1_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;

namespace EnvironmentSetUp
{
    class StairVisualizer : public rclcpp::Node
    {
    public:
        StairVisualizer();
        void PublishStairs(const Eigen::Vector3d &dimensions, const Eigen::Vector3d &first_point, const string &str, int id_);
        void PublishSingleCuboid(const Eigen::Vector3d &position, const Eigen::Vector3d &dimensions, int id_);

    private:
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    };
}

#endif
