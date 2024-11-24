#ifndef CURVES_HPP
#define CURVES_HPP

#include <iostream>
#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "commonmathssolver.hpp"

using namespace std;

namespace Curves
{
    enum class CurveType
    {
        Line,
        Arc,
        Bezier,
        Blending,
        B_Spline
    };

    // this is an interface because all the methods are pure virtual methods
    class Curves
    {
    public:
        double path_lenght;
        virtual Eigen::Vector4d GenerateSinglePoint(double param_t) = 0;
        virtual vector<Eigen::Vector4d> Generate3DPoints(int &num) = 0;
    };

    struct ArcAngles
    {
        double theta1, angle_bw_radius;
        int directional_index;
    };

    class Arc : public Curves
    {
    private:
        double yaw, h;

    protected:
        Eigen::Vector3d center;
        Eigen::Matrix4d T;
        Eigen::Vector4d C_wrt_plane;

    public:
        string point_generation;
        double radius;
        ArcAngles angle_info;
        Arc(const Eigen::Vector4d &P1, const Eigen::Vector4d &P2, const Eigen::Vector4d &P3, string str);
        Eigen::Vector4d GenerateSinglePoint(double param_t) override;
        vector<Eigen::Vector4d> Generate3DPoints(int &num) override;
        vector<pair<Eigen::Vector4d, Eigen::Quaterniond>> Generate6DPoints(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2,
                                                                           const Eigen::Matrix4d &pose3, int &num);
        double ComputeTheta(const Eigen::Vector4d &xyz);
        void SetValue(double t, double z);
    };

    class Bezzier : public Curves
    {
    protected:
    public:
        vector<Eigen::Vector4d> control_points_for_bezzier_curve;

        Bezzier(const vector<Eigen::Vector4d> &control_points);
        Eigen::Vector4d GenerateSinglePoint(double param_t) override;
        vector<Eigen::Vector4d> Generate3DPoints(int &num) override;
    };

    class BSpline : public Curves
    {
    protected:
        int degree, k, n;
        double BSplineBlendingFunc(int i, int k, int n, double u);

    public:
        vector<Eigen::Vector4d> control_points_for_bspline_curve;

        BSpline(const vector<Eigen::Vector4d> &control_points, int d);
        Eigen::Vector4d GenerateSinglePoint(double param_t) override;
        vector<Eigen::Vector4d> Generate3DPoints(int &num) override;
        vector<pair<Eigen::Vector4d, Eigen::Quaterniond>> Generate6DPoints(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2, int &num);
    };

    class Line : public Curves
    {
    protected:
        Eigen::Vector4d P1, P2;

    public:
        Line(const Eigen::Vector4d &P1, const Eigen::Vector4d &P2);
        Eigen::Vector4d GenerateSinglePoint(double param_t) override;
        vector<Eigen::Vector4d> Generate3DPoints(int &num) override;
        vector<Eigen::Vector4d> Generate3DPoints(double &precision);
        vector<pair<Eigen::Vector4d, Eigen::Quaterniond>> Generate6DPoints(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2, double &precision);
    };
}

#endif