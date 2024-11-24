#include "rviz2/markers.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace std;

namespace Markers
{
    Rviz2Visualization::Rviz2Visualization() : Node("rviz2_visualizer_node"), marker_id_(0)
    {
        marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 100);
    }

    void Rviz2Visualization::VisualizePoint(const Eigen::Vector4d &point1_, const Eigen::Vector4d &point2_)
    {
        geometry_msgs::msg::Point point1;
        point1.x = point1_(0);
        point1.y = point1_(1);
        point1.z = point1_(2);

        geometry_msgs::msg::Point point2;
        point2.x = point2_(0);
        point2.y = point2_(1);
        point2.z = point2_(2);

        if (marker_id_ > 0)
        {
            visualization_msgs::msg::Marker delete_point_marker1;
            delete_point_marker1.header.frame_id = "world_frame";
            delete_point_marker1.header.stamp = this->get_clock()->now();
            delete_point_marker1.ns = "moving_point";
            delete_point_marker1.id = marker_id_ - 1;
            delete_point_marker1.action = visualization_msgs::msg::Marker::DELETE;
            marker_array.markers.push_back(delete_point_marker1);

            visualization_msgs::msg::Marker delete_point_marker2;
            delete_point_marker2.header.frame_id = "world_frame";
            delete_point_marker2.header.stamp = this->get_clock()->now();
            delete_point_marker2.ns = "moving_point";
            delete_point_marker2.id = marker_id_ - 2;
            delete_point_marker2.action = visualization_msgs::msg::Marker::DELETE;
            marker_array.markers.push_back(delete_point_marker2);
        }

        // Create and publish the new marker
        visualization_msgs::msg::Marker point_marker1;
        point_marker1.header.frame_id = "world_frame";
        point_marker1.header.stamp = this->get_clock()->now();
        point_marker1.ns = "moving_point";
        point_marker1.id = marker_id_;
        point_marker1.type = visualization_msgs::msg::Marker::SPHERE;
        point_marker1.action = visualization_msgs::msg::Marker::ADD;

        point_marker1.pose.position = point1;

        point_marker1.scale.x = 0.02;
        point_marker1.scale.y = 0.02;
        point_marker1.scale.z = 0.02;

        point_marker1.color.a = 1.0;
        point_marker1.color.r = 1.0;
        point_marker1.color.g = 0.0;
        point_marker1.color.b = 0.0;
        marker_array.markers.push_back(point_marker1);

        visualization_msgs::msg::Marker point_marker2;
        point_marker2.header.frame_id = "world_frame";
        point_marker2.header.stamp = this->get_clock()->now();
        point_marker2.ns = "moving_point";
        point_marker2.id = marker_id_ + 1;
        point_marker2.type = visualization_msgs::msg::Marker::SPHERE;
        point_marker2.action = visualization_msgs::msg::Marker::ADD;

        point_marker2.pose.position = point2;

        point_marker2.scale.x = 0.02;
        point_marker2.scale.y = 0.02;
        point_marker2.scale.z = 0.02;

        point_marker2.color.a = 1.0;
        point_marker2.color.r = 1.0;
        point_marker2.color.g = 1.0;
        point_marker2.color.b = 1.0;

        marker_array.markers.push_back(point_marker2);
    }

    void Rviz2Visualization::VisualizeSupportPolygon(const std::vector<Eigen::Vector4d> &points_, double thickness)
    {
        std::vector<geometry_msgs::msg::Point> points;
        // cout << "id2 = " << marker_id_/2 << endl;
        for (const auto &p_ : points_)
        {
            geometry_msgs::msg::Point p;
            p.x = p_(0);
            p.y = p_(1);
            p.z = p_(2) + 0.01;
            points.push_back(p);
        }

        if (marker_id_ > 0)
        {
            visualization_msgs::msg::Marker delete_polygon_marker;
            delete_polygon_marker.header.frame_id = "world_frame";
            delete_polygon_marker.header.stamp = this->get_clock()->now();
            delete_polygon_marker.ns = "support_polygon";
            delete_polygon_marker.id = marker_id_ - 2;
            delete_polygon_marker.action = visualization_msgs::msg::Marker::DELETE;
            marker_array.markers.push_back(delete_polygon_marker);
        }

        // visualization_msgs::msg::Marker polygon_marker;
        // polygon_marker.header.frame_id = "world_frame"; // Replace with your frame ID
        // polygon_marker.header.stamp = this->now();
        // polygon_marker.ns = "support_polygon";
        // polygon_marker.id = marker_id_;
        // polygon_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        // polygon_marker.action = visualization_msgs::msg::Marker::ADD;

        // // Set the marker properties
        // polygon_marker.scale.x = 1.0;
        // polygon_marker.scale.y = 1.0;
        // polygon_marker.scale.z = 1.0;
        // if (points_.size() == 4)
        // {
        //     polygon_marker.color.r = 0.0;
        //     polygon_marker.color.g = 0.0;
        //     polygon_marker.color.b = 1.0;
        // }
        // else
        // {
        //     polygon_marker.color.r = 0.0;
        //     polygon_marker.color.g = 1.0;
        //     polygon_marker.color.b = 0.0;
        // }
        // polygon_marker.color.a = 1.0;

        // // Compute normal to the plane
        // Eigen::Vector3d unit_normal;
        // if (points.size() == 3)
        // {
        //     // Calculate the normal for the triangle ie p1p2 and p1p3
        //     Eigen::Vector3d v1(points[1].x - points[0].x, points[1].y - points[0].y, points[1].z - points[0].z);
        //     Eigen::Vector3d v2(points[2].x - points[0].x, points[2].y - points[0].y, points[2].z - points[0].z);
        //     unit_normal = v1.cross(v2).normalized();
        //     cout << "unit normal 3 = " << unit_normal.adjoint() << endl;
        // }
        // else if (points.size() == 4)
        // {
        //     // Calculate the normal for the quadrilateral (assuming it's planar) ie p1p2 and p1p4
        //     Eigen::Vector3d v1(points[1].x - points[0].x, points[1].y - points[0].y, points[1].z - points[0].z);
        //     Eigen::Vector3d v2(points[3].x - points[0].x, points[3].y - points[0].y, points[3].z - points[0].z);
        //     unit_normal = v1.cross(v2).normalized();
        //     cout << "unit normal 4 = " << unit_normal.adjoint() << endl;
        // }

        // // Add triangles to the marker
        // for (size_t i = 0; i < points.size(); ++i)
        // {
        //     // Triangle vertices
        //     geometry_msgs::msg::Point p1 = points[i];
        //     geometry_msgs::msg::Point p2 = points[(i + 1) % points.size()];
        //     geometry_msgs::msg::Point p3 = points[(i + 2) % points.size()];
        //     // cout << "zzzzzzzzzz " << p1.z << " " << p2.z << " " << p3.z << endl;
        //     // geometry_msgs::msg::Point p1_, p2_, p3_;

        //     polygon_marker.points.push_back(p1);
        //     polygon_marker.points.push_back(p2);
        //     polygon_marker.points.push_back(p3);

        //     double z1 = p1.z, z2 = p2.z, z3 = p3.z;
        //     // Add thickness by offsetting along the normal
        //     p1.x += thickness * unit_normal.x();
        //     p1.y += thickness * unit_normal.y();
        //     p1.z += thickness * unit_normal.z();
        //     if (p1.z < z1)
        //         p1.z -= 2 * thickness * unit_normal.z();

        //     p2.x += thickness * unit_normal.x();
        //     p2.y += thickness * unit_normal.y();
        //     p2.z += thickness * unit_normal.z();
        //     if (p2.z < z2)
        //         p2.z -= 2 * thickness * unit_normal.z();

        //     p3.x += thickness * unit_normal.x();
        //     p3.y += thickness * unit_normal.y();
        //     p3.z += thickness * unit_normal.z();
        //     if (p3.z < z3)
        //         p3.z -= 2 * thickness * unit_normal.z();

        //     polygon_marker.points.push_back(p1);
        //     polygon_marker.points.push_back(p2);
        //     polygon_marker.points.push_back(p3);
        // }
        // marker_array.markers.push_back(polygon_marker);
        // remove these lines later on
        // marker_pub->publish(marker_array);
        // marker_id_++;

        visualization_msgs::msg::Marker polygon_marker;
        polygon_marker.header.frame_id = "world_frame"; // Replace with your frame ID
        polygon_marker.header.stamp = this->now();
        polygon_marker.ns = "support_polygon";
        polygon_marker.id = marker_id_;
        polygon_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        polygon_marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the marker properties
        polygon_marker.scale.x = 0.01;
        polygon_marker.scale.y = 0.01;
        polygon_marker.scale.z = 0.01;
        if (points_.size() == 4)
        {
            polygon_marker.color.r = 0.0;
            polygon_marker.color.g = 0.0;
            polygon_marker.color.b = 1.0;
        }
        else
        {
            polygon_marker.color.r = 0.0;
            polygon_marker.color.g = 1.0;
            polygon_marker.color.b = 0.0;
        }
        polygon_marker.color.a = 1.0;

        for (const auto &point : points)
            polygon_marker.points.push_back(point);
        if (!points.empty())
            polygon_marker.points.push_back(points[0]);
        marker_array.markers.push_back(polygon_marker);
    }

    void Rviz2Visualization::VisualizeEverything(const std::vector<Eigen::Vector4d> &points,
                                                 const Eigen::Vector4d &point1_, const Eigen::Vector4d &point2_, double thickness)
    {
        // marker_array.markers.clear();
        VisualizePoint(point1_, point2_);
        VisualizeSupportPolygon(points, thickness);
        marker_pub->publish(marker_array);
        marker_id_ += 2;
    }
}