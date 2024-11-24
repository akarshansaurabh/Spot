#ifndef MARKERS_HPP
#define MARKERS_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <eigen3/Eigen/Dense>

namespace Markers
{
    class Rviz2Visualization : public rclcpp::Node
    {
    private:
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
        visualization_msgs::msg::MarkerArray marker_array;
        int marker_id_;

    public:
        Rviz2Visualization();
        void VisualizeSupportPolygon(const std::vector<Eigen::Vector4d> &points_, double thickness);
        void VisualizePoint(const Eigen::Vector4d &point1_, const Eigen::Vector4d &point2_);
        void VisualizeEverything(const std::vector<Eigen::Vector4d> &points,
                                 const Eigen::Vector4d &point1_, const Eigen::Vector4d &point2_, double thickness);
    };
}

#endif