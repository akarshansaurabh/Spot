#include "kinematics/kinematics.hpp"

namespace Kinematics
{
    DHTransformationMatrix::DHTransformationMatrix(const Eigen::Matrix3d &dh_table, double t, int jnt_index)
    {
        double a = dh_table(jnt_index, 0);
        double r = dh_table(jnt_index, 1);
        double d = dh_table(jnt_index, 2);

        dht_matrix << cos(t), (-sin(t) * cos(a)), (sin(t) * sin(a)), (r * cos(t)),
            sin(t), (cos(t) * cos(a)), (-cos(t) * sin(a)), (r * sin(t)),
            0.0, sin(a), cos(a), d,
            0.0, 0.0, 0.0, 1.0;
    }
    void DHTransformationMatrix::Print()
    {
        cout << dht_matrix << endl;
    }

    std::unique_ptr<robot_state> state_tracker = std::make_unique<robot_state>();
    std::unique_ptr<Kinematics> kinematics_solver = std::make_unique<Kinematics>();

    Kinematics::Kinematics()
    {
        T0wrtW << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0.175,
            0, 0, 0, 1;

        T1wrt0.setIdentity();
        T2wrt1.setIdentity();

        TBwrt1 << 1, 0, 0, 0.29,
            0, 1, 0, 0,
            0, 0, 1, 0.08,
            0, 0, 0, 1;

        T0wrtW_inverse = T0wrtW.inverse();
        TBwrt1_inverse = TBwrt1.inverse();

        state_tracker = make_unique<robot_state>();
        state_tracker->body_xyz << 0.0, 0.0, 0.175, 1.0;
        state_tracker->body_orientation.setIdentity();
        state_tracker->body_rpy = state_tracker->body_orientation.eulerAngles(0, 1, 2);
        state_tracker->leg_states[0].foot_xyz << 0.3203, 0.16594, -0.515, 1.0;
        state_tracker->leg_states[1].foot_xyz << -0.275399, 0.16594, -0.515, 1.0;
        state_tracker->leg_states[2].foot_xyz << -0.275399, -0.16594, -0.515, 1.0;
        state_tracker->leg_states[3].foot_xyz << 0.3203, -0.16594, -0.515, 1.0;
        state_tracker->leg_states[0].theta << 0.0, 0.785, -1.57;
        state_tracker->leg_states[1].theta = state_tracker->leg_states[2].theta = state_tracker->leg_states[3].theta = state_tracker->leg_states[0].theta;

        legs[0].O << 0.29785, 0.055, 0.0, 1.0;
        legs[1].O << -0.29785, 0.055, 0.0, 1.0;
        legs[2].O << -0.29785, -0.055, 0.0, 1.0;
        legs[3].O << 0.29785, -0.055, 0.0, 1.0;

        legs[0].O_ << 0.29785, 0.16594, 0.0, 1.0;
        legs[1].O_ << -0.29785, 0.16594, 0.0, 1.0;
        legs[2].O_ << -0.29785, -0.16594, 0.0, 1.0;
        legs[3].O_ << 0.29785, -0.16594, 0.0, 1.0;

        legs[0].xyz = state_tracker->leg_states[0].foot_xyz;
        legs[1].xyz = state_tracker->leg_states[1].foot_xyz;
        legs[2].xyz = state_tracker->leg_states[2].foot_xyz;
        legs[3].xyz = state_tracker->leg_states[3].foot_xyz;

        legs[0].d0 = legs[1].d0 = 0.11094;
        legs[2].d0 = legs[3].d0 = -0.11094;
        legs[0].l1 = legs[1].l1 = legs[2].l1 = legs[3].l1 = 0.34825;
        legs[0].l2 = legs[1].l2 = legs[2].l2 = legs[3].l2 = 0.38;
        legs[0].r1 = legs[1].r1 = legs[2].r1 = legs[3].r1 = legs[3].l1 / 2.0;
        legs[0].r2 = legs[1].r2 = legs[2].r2 = legs[3].r2 = legs[3].l2 / 2.0;

        // Leg 0
        legs[0].dh_table << PI / 2, 0, 0,
            0, -legs[0].l1, legs[0].d0,
            0, -legs[0].l2, 0;
        legs[0].dh_table_com1 << PI / 2, 0, 0,
            0, -legs[0].r1, legs[0].d0,
            0, 0, 0;
        legs[0].dh_table_com2 << PI / 2, 0, 0,
            0, -legs[0].l1, legs[0].d0,
            0, -legs[0].r2, 0;

        // Leg 1
        legs[2].dh_table << PI / 2, 0, 0,
            0, -legs[2].l1, legs[2].d0,
            0, -legs[2].l2, 0;
        legs[2].dh_table_com1 << PI / 2, 0, 0,
            0, -legs[2].r1, legs[2].d0,
            0, 0, 0;
        legs[2].dh_table_com2 << PI / 2, 0, 0,
            0, -legs[2].l1, legs[2].d0,
            0, -legs[2].r2, 0;

        legs[1].dh_table = legs[0].dh_table;
        legs[3].dh_table = legs[2].dh_table;
        legs[1].dh_table_com1 = legs[0].dh_table_com1;
        legs[3].dh_table_com1 = legs[2].dh_table_com1;
        legs[1].dh_table_com2 = legs[0].dh_table_com2;
        legs[3].dh_table_com2 = legs[2].dh_table_com2;

        Eigen::Matrix3d R;
        R << 0, 0, 1,
            1, 0, 0,
            0, 1, 0;
        legs[0].T.block<3, 3>(0, 0) = legs[1].T.block<3, 3>(0, 0) = legs[2].T.block<3, 3>(0, 0) = legs[3].T.block<3, 3>(0, 0) = R;

        for (int i = 0; i < 4; i++)
        {
            legs[i].T.block<4, 1>(0, 3) = legs[i].O;
            legs[i].T.block<1, 3>(3, 0).setZero();
            legs[i].T_inverse = legs[i].T.inverse();
        }

        stance_legs_wrt_world.clear();
        vector<Eigen::Vector4d> stance_legs_wrt_world_home;
        stance_legs_wrt_world_home.resize(4);
        for (int i = 0; i < 4; i++)
            stance_legs_wrt_world_home[i] = T0wrtW * legs[i].xyz;
        stance_legs_wrt_world.push_back(stance_legs_wrt_world_home);
    }

    Eigen::Vector3d Kinematics::SolveIK(const Eigen::Vector4d &xyz_wrt_leg_frame, int leg_index)
    {
        double d0 = legs[leg_index].d0;
        double l1 = legs[leg_index].l1;
        double l2 = legs[leg_index].l2;

        double x = xyz_wrt_leg_frame(0);
        double y = xyz_wrt_leg_frame(1);
        double z = xyz_wrt_leg_frame(2);

        double x_ = sqrt((x * x) + (y * y) - (d0 * d0));
        double y_ = z;

        double alfa = acos(((l1 * l1) + (x_ * x_) + (y_ * y_) - (l2 * l2)) / (2 * l1 * sqrt((x_ * x_) + (y_ * y_))));
        double gama = atan2(y_, x_);

        double t1 = (PI / 2) - (atan2(fabs(y), fabs(x)) + atan2(fabs(d0), fabs(x_)));
        if (leg_index == 2 || leg_index == 3)
            t1 = -t1;
        double t2 = -(gama - alfa);
        double t3 = -acos(((x_ * x_) + (y_ * y_) - (l1 * l1) - (l2 * l2)) / (2 * l1 * l2));

        Eigen::Vector3d ans;
        // if (fabs(t1) > 0.79 || t2 > 2.3 || t2 < -0.9 || t3 > -0.25 || t3 < -2.79)
        //     ans = state_tracker->leg_states[leg_index].theta;
        // else
            ans << t1, t2, t3;
        return ans;
    }
    // To compute IK, I want l1,l2,l3,l4 wrt F1
    vector<Eigen::Vector3d> Kinematics::SolveIK(const Eigen::Vector4d &l1_xyz_wrt_o, const Eigen::Vector4d &l2_xyz_wrt_o,
                                                const Eigen::Vector4d &l3_xyz_wrt_o, const Eigen::Vector4d &l4_xyz_wrt_o)
    {
        // transform from o to oi
        Eigen::Vector4d l1_xyz_wrt_o1 = legs[0].T_inverse * l1_xyz_wrt_o; // wrt 1
        Eigen::Vector4d l2_xyz_wrt_o2 = legs[1].T_inverse * l2_xyz_wrt_o;
        Eigen::Vector4d l3_xyz_wrt_o3 = legs[2].T_inverse * l3_xyz_wrt_o;
        Eigen::Vector4d l4_xyz_wrt_o4 = legs[3].T_inverse * l4_xyz_wrt_o;
        // SolveIK
        state_tracker->leg_states[0].theta = SolveIK(l1_xyz_wrt_o1, 0);
        state_tracker->leg_states[1].theta = SolveIK(l2_xyz_wrt_o2, 1);
        state_tracker->leg_states[2].theta = SolveIK(l3_xyz_wrt_o3, 2);
        state_tracker->leg_states[3].theta = SolveIK(l4_xyz_wrt_o4, 3);

        vector<Eigen::Vector3d> ans = {state_tracker->leg_states[0].theta, state_tracker->leg_states[1].theta,
                                       state_tracker->leg_states[2].theta, state_tracker->leg_states[3].theta};

        state_tracker->leg_states[0].foot_xyz = l1_xyz_wrt_o; // wrt 1
        state_tracker->leg_states[1].foot_xyz = l2_xyz_wrt_o;
        state_tracker->leg_states[2].foot_xyz = l3_xyz_wrt_o;
        state_tracker->leg_states[3].foot_xyz = l4_xyz_wrt_o;

        return ans;
    }
    Eigen::Vector4d Kinematics::SolveFK(const Eigen::Vector3d &thetas, int leg_index, const LegLinks &link_info)
    {
        // find xyz wrt oi
        std::unique_ptr<DHTransformationMatrix> H01, H12, H23;
        if (link_info == LegLinks::all)
        {
            H01 = std::make_unique<DHTransformationMatrix>(legs[leg_index].dh_table, (PI / 2) + thetas(0), 0);
            H12 = std::make_unique<DHTransformationMatrix>(legs[leg_index].dh_table, thetas(1), 1);
            H23 = std::make_unique<DHTransformationMatrix>(legs[leg_index].dh_table, thetas(2), 2);
        }
        else if (link_info == LegLinks::link2)
        {
            H01 = std::make_unique<DHTransformationMatrix>(legs[leg_index].dh_table_com1, (PI / 2) + thetas(0), 0);
            H12 = std::make_unique<DHTransformationMatrix>(legs[leg_index].dh_table_com1, thetas(1), 1);
            H23 = std::make_unique<DHTransformationMatrix>(legs[leg_index].dh_table_com1, thetas(2), 2);
        }
        else if (link_info == LegLinks::link3)
        {
            H01 = std::make_unique<DHTransformationMatrix>(legs[leg_index].dh_table_com2, (PI / 2) + thetas(0), 0);
            H12 = std::make_unique<DHTransformationMatrix>(legs[leg_index].dh_table_com2, thetas(1), 1);
            H23 = std::make_unique<DHTransformationMatrix>(legs[leg_index].dh_table_com2, thetas(2), 2);
        }
        Eigen::Matrix4d T03 = H01->dht_matrix * H12->dht_matrix * H23->dht_matrix;

        // transform from oi to o
        return legs[leg_index].T * T03.block<4, 1>(0, 3);
    }
    vector<Eigen::Vector4d> Kinematics::SolveFK(const Eigen::Vector3d &l1_t, const Eigen::Vector3d l2_t,
                                                const Eigen::Vector3d &l3_t, const Eigen::Vector3d &l4_t, const LegLinks &link_info)
    {
        vector<Eigen::Vector4d> ans;
        ans.resize(4);
        if (link_info == LegLinks::all)
        {
            state_tracker->leg_states[0].foot_xyz = SolveFK(l1_t, 0, link_info);
            state_tracker->leg_states[1].foot_xyz = SolveFK(l2_t, 1, link_info);
            state_tracker->leg_states[2].foot_xyz = SolveFK(l3_t, 2, link_info);
            state_tracker->leg_states[3].foot_xyz = SolveFK(l4_t, 3, link_info);

            for (int i = 0; i < 4; i++)
                ans[i] = state_tracker->leg_states[i].foot_xyz;

            state_tracker->leg_states[0].theta = l1_t;
            state_tracker->leg_states[1].theta = l2_t;
            state_tracker->leg_states[2].theta = l3_t;
            state_tracker->leg_states[3].theta = l4_t;
        }
        else
        {
            ans[0] = SolveFK(l1_t, 0, link_info);
            ans[1] = SolveFK(l2_t, 1, link_info);
            ans[2] = SolveFK(l3_t, 2, link_info);
            ans[3] = SolveFK(l4_t, 3, link_info);
        }
        return ans;
    }
}
