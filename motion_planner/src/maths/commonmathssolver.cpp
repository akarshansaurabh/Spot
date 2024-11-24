#include "maths/commonmathssolver.hpp"

namespace CommonMathsSolver
{
    double Kinematics::SelectDesiredSolution(const Eigen::Vector3d &Li_prev, int jnt_index, const vector<double> &possibleIK_sol, bool &ik_exists)
    {
        double d1, d2, Li_Solution;
        if (possibleIK_sol.size() == 2)
        {
            d1 = possibleIK_sol[0] - Li_prev(jnt_index);
            d2 = possibleIK_sol[1] - Li_prev(jnt_index);
            if (d1 <= d2)
                Li_Solution = possibleIK_sol[0];
            else
                Li_Solution = possibleIK_sol[1];
        }
        else if (possibleIK_sol.size() == 1)
            Li_Solution = possibleIK_sol[0];
        else
        {
            Li_Solution = Li_prev(jnt_index);
            ik_exists = false;
        }

        return Li_Solution;
    }

    Eigen::Matrix3d OrientationNTransformaton::Compute_Rx(double t)
    {
        Eigen::Matrix3d rx;
        rx << 1, 0, 0,
            0, cos(t), -sin(t),
            0, sin(t), cos(t);
        return rx;
    }

    Eigen::Matrix3d OrientationNTransformaton::Compute_Ry(double t)
    {
        Eigen::Matrix3d ry;
        ry << cos(t), 0, sin(t),
            0, 1, 0,
            -sin(t), 0, cos(t);
        return ry;
    }

    Eigen::Matrix3d OrientationNTransformaton::Compute_Rz(double t)
    {
        Eigen::Matrix3d rz;
        rz << cos(t), -sin(t), 0,
            sin(t), cos(t), 0,
            0, 0, 1;
        return rz;
    }

    Eigen::Matrix3d OrientationNTransformaton::ComputeR(const Eigen::Vector3d &rpy)
    {
        double a = rpy(0), b = rpy(1), y = rpy(2);
        Eigen::Matrix3d target_rot;
        target_rot << (cos(b) * cos(y)), ((-cos(a) * sin(y)) + (sin(a) * sin(b) * cos(y))), ((sin(a) * sin(y)) + (cos(a) * sin(b) * cos(y))),
            (cos(b) * sin(y)), ((cos(a) * cos(y)) + (sin(a) * sin(b) * sin(y))), ((-sin(a) * cos(y)) + (cos(a) * sin(b) * sin(y))),
            (-sin(b)), ((cos(b) * sin(a))), ((cos(a) * cos(b)));
        return target_rot;
    }

    vector<Eigen::Quaterniond> OrientationNTransformaton::GenerateQuaternions(Eigen::Quaterniond &qi, Eigen::Quaterniond &qf, int num)
    {
        vector<Eigen::Quaterniond> quaternions;
        qi.normalize();
        qf.normalize();
        double angle = 2 * acos(qi.dot(qf));
        double dt = angle / (num - 1);

        for (int i = 0; i < num; i++)
            if (fabs(qf.x() - qi.x()) <= 0.00001 && fabs(qf.y() - qi.y()) <= 0.00001 && fabs(qf.z() - qi.z()) <= 0.00001 && fabs(qf.w() - qi.w()) <= 0.00001)
                quaternions.push_back(qf);
            else
                quaternions.push_back(qi.slerp((i * dt / angle), qf));
        return quaternions;
    }

    double SmoothCurves::BSplineBlendingFunction(int i, int k, double u)
    {
        double B;
        if (u >= i && u < (i + k))
        {
            if (k == 1)
                if (u >= i && u <= (i + 1))
                    B = 1;
                else
                    B = 0;
            else
                B = (((u - i) / (k - 1)) * BSplineBlendingFunction(i, (k - 1), u)) + ((i + k - u) / (k - 1)) * BSplineBlendingFunction((i + 1), (k - 1), u);
        }
        else
            B = 0.0;
        return B;
    }

    bool Geometry::PointIsInsideTriangle(const Eigen::Vector4d &A, const Eigen::Vector4d &B, const Eigen::Vector4d &C, const Eigen::Vector4d &P)
    {
        double A1 = ComputeTriangleArea(A, B, P);
        double A2 = ComputeTriangleArea(A, C, P);
        double A3 = ComputeTriangleArea(B, C, P);
        double A4 = ComputeTriangleArea(B, C, A);
        if (fabs(A4 - (A1 + A2 + A3)) < 0.01)
            return true;
        return false;
    }

    double Geometry::ComputeTriangleArea(const Eigen::Vector4d &A, const Eigen::Vector4d &B, const Eigen::Vector4d &C)
    {
        Line l(A, B);
        return 0.5 * Distance2P(A, B) * DistanceFromLine(l, C);
    }

    double Geometry::DistanceFromLine(const Line &L, const Eigen::Vector4d &P)
    {
        return fabs((L.A * P(0)) + (L.B * P(1)) + L.C) / sqrt((L.A * L.A) + (L.B * L.B));
    }

    vector<Eigen::Vector4d> Geometry::GenerateLine(const Eigen::Vector4d &p1, const Eigen::Vector4d &p2)
    {
        vector<Eigen::Vector4d> ans;
        Eigen::Vector4d pi;
        for (double i = 0.0; i <= 1.0; i += 0.01)
        {
            pi = p1 + (i * (p2 - p1));
            pi(3) = 1.0;
            ans.push_back(pi);
        }
        return ans;
    }

    Eigen::Vector4d Geometry::PoI_2L(const Line &L1, const Line &L2, double z)
    {
        double x = ((L2.B * L1.C) - (L1.B * L2.C)) / ((L1.B * L2.A) - (L1.A * L2.B));
        double y = ((L1.A * L2.C) - (L2.A * L1.C)) / ((L1.B * L2.A) - (L1.A * L2.B));
        if ((L1.B * L2.A) - (L1.A * L2.B) == 0)
            y = x = (double)INFINITY;
        Eigen::Vector4d ans(x, y, z, 1.0);
        return ans;
    }
    Eigen::Vector4d Geometry::ArbitraryPointOnLine(const Eigen::Vector4d &P1, const Eigen::Vector4d &P2, double r)
    {
        Eigen::Vector3d p1(P1(0), P1(1), P1(2)), p2(P2(0), P2(1), P2(2));
        Eigen::Vector3d p1p2 = p2 - p1;
        Eigen::Vector4d ans;

        if (p1p2.norm() > 0.0)
        {
            Eigen::Vector3d n_cap = p1p2 / p1p2.norm();
            ans << P1(0) + (r * n_cap(0)), P1(1) + (r * n_cap(1)), P1(2), 1.0;
        }
        else
            ans << P1(0), P1(1), P1(2), 1.0;
        return ans;
    }
    vector<Eigen::Vector4d> Geometry::GenerateCPForArc(const Eigen::Vector4d &A, const Eigen::Vector4d &B, const Eigen::Vector4d &C, double &r_percent)
    {
        double a = (B - C).norm();
        double b = (A - C).norm();
        double c = (B - A).norm();
        double theta = acos(((a * a) + (c * c) - (b * b)) / (2 * a * c));
        double r = r_percent * min(a, c) / 100.0;
        double R0 = r * tan(theta / 2);

        Geometry::Line T1(A, B), T2(B, C);
        Eigen::Vector4d P = ArbitraryPointOnLine(B, A, r);
        Eigen::Vector4d R = ArbitraryPointOnLine(B, C, r);
        Geometry::Line L1(P, -(1 / T1.m)), L2(R, -(1 / T2.m));
        Eigen::Vector4d Center = PoI_2L(L1, L2, A(2));
        Geometry::Line L3(B, Center);
        Eigen::Vector4d Q = ArbitraryPointOnLine(B, Center, ((B - Center).norm() - R0));
        vector<Eigen::Vector4d> vect = {P, Q, R};
        return vect;
    }

    double Geometry::Distance2P(const Eigen::Vector4d &P1, const Eigen::Vector4d &P2)
    {
        return sqrt(pow(P2(0) - P1(0), 2) + pow(P2(1) - P1(1), 2) + pow(P2(2) - P1(2), 2));
    }

    double MaxMin::FindMax(const vector<double> &vec)
    {
        double max1 = vec[0];
        for (int i = 1; i < vec.size(); i++)
            max1 = max(max1, vec[i]);
        return max1;
    }
    double MaxMin::FindMin(const vector<double> &vec)
    {
        double min1 = vec[0];
        for (int i = 1; i < vec.size(); i++)
            min1 = min(min1, vec[i]);
        return min1;
    }

    Eigen::Vector3d Vectors3D::FindUnitNormal(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3)
    {
        Eigen::Vector3d p2p1 = p1 - p2;
        Eigen::Vector3d p2p3 = p3 - p2;
        Eigen::Vector3d cp = p2p1.cross(p2p3);
        return (cp / cp.norm());
    }
}