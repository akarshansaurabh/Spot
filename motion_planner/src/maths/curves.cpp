#include "maths/curves.hpp"

namespace Curves
{
    Line::Line(const Eigen::Vector4d &P1, const Eigen::Vector4d &P2)
    {
        this->P1 = P1;
        this->P2 = P2;
        path_lenght = CommonMathsSolver::Geometry::Distance2P(P1, P2);
    }

    Eigen::Vector4d Line::GenerateSinglePoint(double param_t)
    {
        double i = param_t / path_lenght;
        if (path_lenght == 0)
            i = 0.0;
        return (P1 + (i * (P2 - P1)));
    }

    vector<Eigen::Vector4d> Line::Generate3DPoints(int &num)
    {
        double precision = path_lenght / num;

        vector<Eigen::Vector4d> line_points;
        for (int i = 0; i <= num; i++)
            line_points.push_back(GenerateSinglePoint(i * precision));
        return line_points;
    }

    vector<Eigen::Vector4d> Line::Generate3DPoints(double &precision)
    {
        int num = path_lenght / precision;
        if (num > 0)
            precision = path_lenght / num;
        else if (path_lenght > 0)
            precision = path_lenght;
        num = path_lenght / precision;
        if (path_lenght <= 0.01)
        {
            precision = 0.0;
            num = 0;
        }

        vector<Eigen::Vector4d> line_points;
        for (int i = 0; i <= num; i++)
            line_points.push_back(GenerateSinglePoint(i * precision));
        return line_points;
    }

    vector<pair<Eigen::Vector4d, Eigen::Quaterniond>> Line::Generate6DPoints(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2, double &precision)
    {
        vector<Eigen::Vector4d> line_xyzs = Generate3DPoints(precision);
        Eigen::Quaterniond Q1(pose1.block<3, 3>(0, 0)), Q2(pose2.block<3, 3>(0, 0));
        vector<pair<Eigen::Vector4d, Eigen::Quaterniond>> ans;
        vector<Eigen::Quaterniond> line_qs;

        if (line_xyzs.size() > 1)
        {
            line_qs = CommonMathsSolver::OrientationNTransformaton::GenerateQuaternions(Q1, Q2, line_xyzs.size());
            for (int i = 0; i < line_xyzs.size(); i++)
                ans.push_back(make_pair(line_xyzs[i], line_qs[i]));
        }
        else if (line_xyzs.size() == 1 && (fabs(Q2.x() - Q1.x()) > 0.01 || fabs(Q2.y() - Q1.y()) > 0.01 || fabs(Q2.z() - Q1.z()) > 0.01 || fabs(Q2.w() - Q1.w()) > 0.01))
        {
            line_qs = CommonMathsSolver::OrientationNTransformaton::GenerateQuaternions(Q1, Q2, 100);
            for (int i = 0; i < 100; i++)
                ans.push_back(make_pair(pose1.block<4, 1>(0, 3), line_qs[i]));
        }
        return ans;
    }

    //*********************************************** BEZZIER *******************************************************
    Bezzier::Bezzier(const vector<Eigen::Vector4d> &control_points) : control_points_for_bezzier_curve(control_points)
    {
        Eigen::Vector4d P, P_prev = control_points[0];
        double t;
        this->path_lenght = 0.0;
        for (int i = 1; i <= 500; i++)
        {
            t = i / 500;
            P = (pow((1 - t), 3) * control_points[0]) + (3 * (pow((1 - t), 2)) * t * control_points[1]) + (3 * (1 - t) * t * t * control_points[2]) + (t * t * t * control_points[3]);
            Eigen::Vector3d p(P(0), P(1), P(2)), p_prev(P_prev(0), P_prev(1), P_prev(2));
            this->path_lenght += (p - p_prev).norm();
            P_prev = P;
        }
    }
    Eigen::Vector4d Bezzier::GenerateSinglePoint(double param_t)
    {
        Eigen::Vector4d P = (pow((1 - param_t), 3) * control_points_for_bezzier_curve[0]) +
                            (3 * (pow((1 - param_t), 2)) * param_t * control_points_for_bezzier_curve[1]) +
                            (3 * (1 - param_t) * param_t * param_t * control_points_for_bezzier_curve[2]) +
                            (param_t * param_t * param_t * control_points_for_bezzier_curve[3]);
        return P;
    }
    vector<Eigen::Vector4d> Bezzier::Generate3DPoints(int &num)
    {
        double t;
        Eigen::Vector4d P;
        vector<Eigen::Vector4d> entire_bezier;

        for (int i = 1; i <= num; i++)
        {
            t = (double)i / num;
            P = (pow((1 - t), 3) * control_points_for_bezzier_curve[0]) +
                (3 * (pow((1 - t), 2)) * t * control_points_for_bezzier_curve[1]) +
                (3 * (1 - t) * t * t * control_points_for_bezzier_curve[2]) +
                (t * t * t * control_points_for_bezzier_curve[3]);
            entire_bezier.push_back(P);
        }
        return entire_bezier;
    }

    BSpline::BSpline(const vector<Eigen::Vector4d> &control_points, int d) : degree(d)
    {
        int n_cp = control_points.size();
        // cout << n_cp << endl;
        int num_cp = n_cp + (2 * (degree - 1));
        for (int i = 0; i < (d - 1); i++)
            control_points_for_bspline_curve.push_back(control_points[0]);
        for (int i = 0; i < n_cp; i++)
            control_points_for_bspline_curve.push_back(control_points[i]);
        for (int i = 0; i < (d - 1); i++)
            control_points_for_bspline_curve.push_back(control_points[n_cp - 1]);

        this->k = d + 1; // k = order
        this->n = num_cp - 1;

        // int num_ = 200;
        // vector<Eigen::Vector4d> bspline_points = this->Generate3DPoints(num_);

        // this->path_lenght = 0.0;
        // Eigen::Vector4d p_prev = control_points_for_bspline_curve[0];
        // for (const auto &point : bspline_points)
        // {
        //     this->path_lenght += CommonMathsSolver::Geometry::Distance2P(point, p_prev);
        //     p_prev = point;
        // }
    }

    Eigen::Vector4d BSpline::GenerateSinglePoint(double param_t)
    {
        Eigen::Vector4d P;
        return P;
    }

    vector<Eigen::Vector4d> BSpline::Generate3DPoints(int &num)
    {
        double i_max = (double)n + (double)k;
        double du = (i_max - (2 * degree)) / num;
        vector<Eigen::Vector4d> ans;
        for (double u = du + degree; u <= i_max - degree; u += du)
        {
            Eigen::Vector4d Pi(0, 0, 0, 0);
            for (int i = 0; i <= n; i++)
                Pi = Pi + (control_points_for_bspline_curve[i] * BSplineBlendingFunc(i, k, n, u)); //* Blending_func(i, k, n, u));
            Pi(3) = 1.0;
            ans.push_back(Pi);
        }
        return ans;
    }

    vector<pair<Eigen::Vector4d, Eigen::Quaterniond>> BSpline::Generate6DPoints(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2, int &num)
    {
        vector<Eigen::Vector4d> bspline_xyzs = Generate3DPoints(num);
        Eigen::Quaterniond Q1(pose1.block<3, 3>(0, 0)), Q2(pose2.block<3, 3>(0, 0));
        vector<pair<Eigen::Vector4d, Eigen::Quaterniond>> ans;
        vector<Eigen::Quaterniond> bspline_qs;

        bspline_qs = CommonMathsSolver::OrientationNTransformaton::GenerateQuaternions(Q1, Q2, bspline_xyzs.size());
        for (int i = 0; i < bspline_xyzs.size(); i++)
            ans.push_back(make_pair(bspline_xyzs[i], bspline_qs[i]));

        return ans;
    }

    double BSpline::BSplineBlendingFunc(int i, int k, int n, double u)
    {
        double B;
        if (u >= i && u < (i + k))
            if (k == 1)
            {
                if (u >= i && u <= (i + 1))
                    B = 1;
                else
                    B = 0;
            }
            else
                B = (((u - i) / (k - 1)) * BSplineBlendingFunc(i, (k - 1), n, u)) + ((i + k - u) / (k - 1)) * BSplineBlendingFunc((i + 1), (k - 1), n, u);
        else
            B = 0.0;
        return B;
    }

    //***********************************************ARC*******************************************************
    Arc::Arc(const Eigen::Vector4d &P1, const Eigen::Vector4d &P2, const Eigen::Vector4d &P3, string str)
    {
        Eigen::Vector3d p1(P1(0), P1(1), P1(2)), p2(P2(0), P2(1), P2(2)), p3(P3(0), P3(1), P3(2));
        double l = (p3 - p1).norm();
        Eigen::Vector4d pp1(0.0, 0.0, 0.0, 1.0), pp2, pp3(l, 0.0, 0.0, 1.0);

        Eigen::Vector3d z_cap = CommonMathsSolver::Vectors3D::FindUnitNormal(p1, p2, p3);
        Eigen::Vector3d pm1pm3 = p3 - p1;
        Eigen::Vector3d x_cap = pm1pm3 / pm1pm3.norm();
        Eigen::Vector3d cp = z_cap.cross(x_cap);
        Eigen::Vector3d y_cap = cp / cp.norm();

        T << x_cap(0), y_cap(0), z_cap(0), p1(0),
            x_cap(1), y_cap(1), z_cap(1), p1(1),
            x_cap(2), y_cap(2), z_cap(2), p1(2),
            0.0, 0.0, 0.0, 1.0;
        Eigen::Vector4d P2_(p2(0), p2(1), p2(2), 1);
        pp2 = T.inverse() * P2_;

        if (pp2(1) < 0.0)
        {
            T << x_cap(0), -y_cap(0), z_cap(0), p1(0),
                x_cap(1), -y_cap(1), z_cap(1), p1(1),
                x_cap(2), -y_cap(2), z_cap(2), p1(2),
                0.0, 0.0, 0.0, 1.0;
            pp2 = T.inverse() * P2_;
        }

        double slope = (pp2(1) - pp3(1)) / (pp2(0) - pp3(0));
        double lambda = -pp2(0) / slope;
        double g = -(pp2(0) + pp3(0) + (lambda * slope)) / 2;
        double f = (lambda - pp2(1) - pp3(1)) / 2;
        double c = (pp2(0) * pp3(0)) + (lambda * slope * pp3(0));
        radius = sqrt(pow(g, 2) + pow(f, 2) - c);

        C_wrt_plane << -g, -f, 0.0, 1.0;
        Eigen::Vector4d center_base = T * C_wrt_plane;
        Eigen::Vector3d center_(C_wrt_plane(0), C_wrt_plane(1), C_wrt_plane(2));
        Eigen::Vector3d pp1_(pp1(0), pp1(1), pp1(2));
        Eigen::Vector3d pp2_(pp2(0), pp2(1), pp2(2));
        Eigen::Vector3d pp3_(pp3(0), pp3(1), pp3(2));

        Eigen::Vector3d r1 = pp1_ - center_;
        Eigen::Vector3d r2 = pp2_ - center_;
        Eigen::Vector3d r3 = pp3_ - center_;
        double theta = acos(r1.dot(r2) / pow(radius, 2)) + acos(r3.dot(r2) / pow(radius, 2));
        double theta1, theta3;
        if (theta <= M_PI)
        {
            theta1 = M_PI - abs(atan((pp1(1, 0) + f) / (pp1(0, 0) + g)));
            theta3 = abs(atan((pp3(1, 0) + f) / (pp3(0, 0) + g)));
        }
        else if (theta > M_PI)
        {
            theta1 = M_PI + abs(atan((pp1(1, 0) + f) / (pp1(0, 0) + g)));
            theta3 = atan((pp3(1, 0) + f) / (pp3(0, 0) + g));
        }

        int n;
        if (theta3 - theta1 < 0.0)
            n = -1.0;
        else
            n = 1.0;

        this->center(0) = center_base(0);
        this->center(1) = center_base(1);
        this->center(2) = center_base(2);
        angle_info.theta1 = theta1;
        if (str == "C")
            angle_info.angle_bw_radius = theta;
        else
            angle_info.angle_bw_radius = 2 * M_PI;
        angle_info.directional_index = n;
        path_lenght = radius * angle_info.angle_bw_radius;
    }
    void Arc::SetValue(double t, double z)
    {
        yaw = t;
        h = z;
    }

    vector<pair<Eigen::Vector4d, Eigen::Quaterniond>> Arc::Generate6DPoints(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2,
                                                                            const Eigen::Matrix4d &pose3, int &num)
    {
        vector<Eigen::Vector4d> arc_xyzs = Generate3DPoints(num);
        Eigen::Quaterniond Q1(pose1.block<3, 3>(0, 0)), Q3(pose3.block<3, 3>(0, 0));
        vector<pair<Eigen::Vector4d, Eigen::Quaterniond>> ans;
        vector<Eigen::Quaterniond> arc_qs;

        if (arc_xyzs.size() > 1)
        {
            arc_qs = CommonMathsSolver::OrientationNTransformaton::GenerateQuaternions(Q1, Q3, arc_xyzs.size());
            for (int i = 0; i < arc_xyzs.size(); i++)
                ans.push_back(make_pair(arc_xyzs[i], arc_qs[i]));
        }
        else if (arc_xyzs.size() == 1 && (fabs(Q3.x() - Q1.x()) > 0.01 || fabs(Q3.y() - Q1.y()) > 0.01 || fabs(Q3.z() - Q1.z()) > 0.01 || fabs(Q3.w() - Q1.w()) > 0.01))
        {
            arc_qs = CommonMathsSolver::OrientationNTransformaton::GenerateQuaternions(Q1, Q3, 100);
            for (int i = 0; i < 100; i++)
                ans.push_back(make_pair(pose1.block<4, 1>(0, 3), arc_qs[i]));
        }
        return ans;
    }

    Eigen::Vector4d Arc::GenerateSinglePoint(double param_t)
    {
        Eigen::Vector4d P_wrt_plane, P;
        int n = angle_info.directional_index;
        if (point_generation == "default")
        {
            P_wrt_plane << C_wrt_plane(0) + (radius * cos(angle_info.theta1 + (n * param_t))),
                C_wrt_plane(1) + (radius * sin(angle_info.theta1 + (n * param_t))),
                0.0, 1.0;
            return T * P_wrt_plane;
        }
        P << center(0) + (radius * cos(param_t)),
            center(1) + (radius * sin(param_t)),
            h, 1.0;
        return P;
    }

    double Arc::ComputeTheta(const Eigen::Vector4d &xyz)
    {
        double sint = (xyz(1) - center(1)) / radius;
        double cost = (xyz(0) - center(0)) / radius;
        return atan2(sint, cost); // atan2(y,x)
    }
    vector<Eigen::Vector4d> Arc::Generate3DPoints(int &num)
    {
        vector<Eigen::Vector4d> entire_arc;
        double d_theta = angle_info.angle_bw_radius / num;

        for (double angle = 0.0; angle <= (angle_info.angle_bw_radius + 0.00005); angle += d_theta)
            entire_arc.push_back(Arc::GenerateSinglePoint(angle));
        return entire_arc;
    }
}