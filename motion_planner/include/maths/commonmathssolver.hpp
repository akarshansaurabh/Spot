#ifndef COMMONMATHSSOLVER_HPP
#define COMMONMATHSSOLVER_HPP

#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>

using namespace std;

const double epsilon = 0.01;

namespace CommonMathsSolver
{
    namespace Vectors3D
    {
        Eigen::Vector3d FindUnitNormal(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3);
    }
    namespace Kinematics
    {
        double SelectDesiredSolution(const Eigen::Vector3d &Li_prev, int jnt_index, const vector<double> &possibleIK_sol, bool &ik_exists);
    }
    namespace OrientationNTransformaton
    {
        Eigen::Matrix3d Compute_Rx(double t);
        Eigen::Matrix3d Compute_Ry(double t);
        Eigen::Matrix3d Compute_Rz(double t);
        Eigen::Matrix3d ComputeR(const Eigen::Vector3d &rpy);
        vector<Eigen::Quaterniond> GenerateQuaternions(Eigen::Quaterniond &qi, Eigen::Quaterniond &qf, int num);
    }
    namespace SmoothCurves
    {
        double BSplineBlendingFunction(int i, int k, double u);
    }
    namespace Geometry
    {
        struct Line
        {
            double A, B, C, m, c;
            Line(const Eigen::Vector4d &p1, double M) : m(M)
            {
                if (abs(m) < 50.0)
                {
                    c = p1(1) - (m * p1(0));
                    A = m;
                    B = -1;
                    C = c;
                }
                else
                {
                    A = 1.0;
                    B = 0;
                    C = -p1(0);
                }
            }
            Line(const Eigen::Vector4d &p1, const Eigen::Vector4d &p2)
            {
                if (p1(0) != p2(0))
                {
                    m = (p2(1) - p1(1)) / (p2(0) - p1(0));
                    c = p1(1) - (m * p1(0));
                    A = m;
                    B = -1;
                    C = c;
                }
                else
                {
                    A = 1.0;
                    B = 0;
                    C = -p1(0);
                    m = (double)INFINITY;
                }
            }
        };
        Eigen::Vector4d PoI_2L(const Line &L1, const Line &L2, double z);
        vector<Eigen::Vector4d> GenerateLine(const Eigen::Vector4d &p1, const Eigen::Vector4d &p2);
        Eigen::Vector4d ArbitraryPointOnLine(const Eigen::Vector4d &P1, const Eigen::Vector4d &P2, double r);
        vector<Eigen::Vector4d> GenerateCPForArc(const Eigen::Vector4d &A, const Eigen::Vector4d &B, const Eigen::Vector4d &C, double &r_percent);
        bool PointIsInsideTriangle(const Eigen::Vector4d &A, const Eigen::Vector4d &B, const Eigen::Vector4d &C, const Eigen::Vector4d &P);
        double ComputeTriangleArea(const Eigen::Vector4d &A, const Eigen::Vector4d &B, const Eigen::Vector4d &C);
        double DistanceFromLine(const Line &L, const Eigen::Vector4d &P);
        double Distance2P(const Eigen::Vector4d &P1, const Eigen::Vector4d &P2);
    }
    namespace MaxMin
    {
        double FindMax(const vector<double> &vec);
        double FindMin(const vector<double> &vec);

    }
    namespace Equations
    {
        inline vector<double> SolveQuadraticEquation(double a, double b, double c, double Lmax = 10000000.0, double Lmin = -10000000.0)
        {
            vector<double> Solution;
            double D = pow(b, 2) - (4 * a * c);
            if (D > -epsilon)
            {
                if (fabs(D) < epsilon)
                    D = 0.0;
                double rootD = sqrt(D);
                double sol[2] = {((-b + rootD) / (2 * a)), ((-b - rootD) / (2 * a))};
                for (int i = 0; i < 2; i++)
                    if ((sol[i]) <= Lmax && (sol[i]) >= Lmin)
                        Solution.push_back(sol[i]);
            }
            return Solution;
        }
    }
}

#endif
