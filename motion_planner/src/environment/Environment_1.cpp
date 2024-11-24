#include "environment/Environment_1.hpp"

namespace EnvironmentSetUp
{
    StairVisualizer::StairVisualizer() : Node("stair_visualizer")
    {
        marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
    }

    void StairVisualizer::PublishStairs(const Eigen::Vector3d &dimensions, const Eigen::Vector3d &first_point, const string &str, int id_)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world_frame";
        marker.header.stamp = this->now();
        marker.ns = "stairs";
        marker.id = id_;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
      
        marker.pose.position.x = first_point(0);
        marker.pose.position.y = first_point(1);
        marker.pose.position.z = first_point(2);
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;

        marker.scale.x = dimensions(0);
        marker.scale.y = dimensions(1);
        marker.scale.z = dimensions(2);

        marker.color.r = 0.5f;
        marker.color.g = 0.5f;
        marker.color.b = 0.5f;
        marker.color.a = 1.0;

        const int num_stairs = 10;
        for (int i = 0; i < num_stairs; ++i)
        {
            geometry_msgs::msg::Point p;
            if (str == "XZ")
            {
                p.x = i * marker.scale.x;
                p.y = first_point(1);
                p.z = i * marker.scale.z;
            }
            else if (str == "yZ")
            {
                p.x = first_point(0);
                p.y = -i * marker.scale.y;
                p.z = i * marker.scale.z;
            }
            else if (str == "xZ")
            {
                p.x = -i * marker.scale.x;
                p.y = first_point(1);
                p.z = i * marker.scale.z;
            }
            marker.points.push_back(p);
        }

        marker_pub->publish(marker);
    }

    void StairVisualizer::PublishSingleCuboid(const Eigen::Vector3d &position, const Eigen::Vector3d &dimensions, int id_)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world_frame";
        marker.header.stamp = this->now();
        marker.ns = "single_cuboid";
        marker.id = id_;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = position(0);
        marker.pose.position.y = position(1);
        marker.pose.position.z = position(2);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = dimensions(0);
        marker.scale.y = dimensions(1);
        marker.scale.z = dimensions(2);

        marker.color.r = 0.5f;
        marker.color.g = 0.5f;
        marker.color.b = 0.5f;
        marker.color.a = 1.0;

        marker_pub->publish(marker);
    }
}
