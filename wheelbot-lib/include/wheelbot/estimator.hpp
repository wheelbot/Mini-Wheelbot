// MIT License

// Copyright (c) 2024 Henrik Hose
//               2023 Andrew Mitri

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef ESTIMATOR_HPP
#define ESTIMATOR_HPP

#include <eigen3/Eigen/Dense>
#include <math.h>
#include "wheelbot/gyro.hpp"
#include "wheelbot/accel.hpp"

template<
size_t N_IMUS=4,
size_t N_MOTORS=2
>
class Estimator{
public:

    Estimator(){}

    Eigen::Vector<float, 10>
    update(
            Eigen::Matrix<float, 3, N_IMUS> omega_B,
            Eigen::Matrix<float, 3, N_IMUS> a_B,
            Eigen::Matrix<float, 3, N_MOTORS> motor_states)
    {
        if (init){
            Eigen::Vector3f g_B = calculate_g_B(a_B, pivot_accel, X1);
            q_A = estimate_accel(g_B);
            q({0,1}) = q_A;
            q(2) = 0;
            init=false;
        }

        Eigen::Matrix<float, N_MOTORS,1> m_q_WR{motor_states(0, Eigen::all)};
        Eigen::Matrix<float, N_MOTORS,1> m_dq_WR{motor_states(1, Eigen::all)};
        Eigen::Matrix<float, N_MOTORS,1> m_ddq_WR{motor_states(2, Eigen::all)};
        // IIR filter potentially very jerky slow velocity reportings by motor
        // auto dq_WR_new = IIRFilter<Eigen::Matrix<float, N_MOTORS, 1>>(m_dq_WR, dq_WR, 0.1);
        // auto dq_WR_new = m_dq_WR;
        // Eigen::Matrix<float, N_MOTORS,1> tau_WR{motor_states(2, Eigen::all)};

        // numerical second derivative of encoder angles, this is even more jerky now :-/ can we use U?
        // ddq_WR = Eigen::Matrix<float, N_MOTORS, 1>::Zero();
        ddq_WR = m_ddq_WR;
        // ddq_WR = IIRFilter<Eigen::Matrix<float, N_MOTORS, 1>>((dq_WR_new-dq_WR)/dt, ddq_WR, 0.1);
        dq_WR = m_dq_WR;
        q_WR = m_q_WR;

        // calculate body rates from gyro measurements and integrate
        dq_G = estimate_gyro(omega_B, q);
        q_G = integrate(dq_G, q, dq, dt);

        // second derivative by numerically diff gyro angular rate and IIR filtering
        ddq = IIRFilter<Eigen::Vector3f>((dq_G - dq) / dt, ddq, 0.1);


        // Accel Estimation:
        pivot_accel = estimate_pivot_accel(q, dq_G, ddq, dq_WR(0), ddq_WR(0));
        pivot_accel = Eigen::Vector3f::Zero();
        Eigen::Vector3f g_B = calculate_g_B(a_B, pivot_accel, X1);
        q_A = estimate_accel(g_B);

        // float alpha_scale = alpha*(1-dq_G.norm());
        // alpha_scale = alpha_scale < 0 ? 0 : alpha_scale;
        float alpha_scale = alpha;
        // Complementary filter:
        q(0) = alpha_scale * q_A(0) + (1 - alpha_scale) * q_G(0);
        q(1) = alpha_scale * q_A(1) + (1 - alpha_scale) * q_G(1);
        q(2) = q_G(2);

        dq = dq_G;

        // const auto q_prop = q + 10e-3 * dq;

        return Eigen::Vector<float, 10>{q(2), q(0), q(1), dq(2), dq(0), dq(1), q_WR(0), dq_WR(0), q_WR(1), dq_WR(1)};
        // return Eigen::Vector<float, 10>{q_prop(2), q_prop(0), q_prop(1), dq(2), dq(0), dq(1), q_WR(0), dq_WR(0), q_WR(1), dq_WR(1)};
    }

    const Eigen::Matrix3f R_upside_down{
        {0, -1, 0},
        {-1, 0, 0},
        {0, 0, -1}};

    const Eigen::Matrix3f R01{
        {0, -1, 0},
        {-1, 0, 0},
        {0, 0, -1}};

    const Eigen::Matrix3f R23{
        {0, -1, 0},
        {1, 0, 0},
        {0, 0, 1}};

    const Eigen::Matrix3f R_Bi[4] = {R01, R01, R23, R23};

    const Eigen::Vector4f X1{-0.166896, -0.167463, 0.667201, 0.667159};

    // const Eigen::MatrixXd P{{1,1,1,1 },
                            // {-0.026,  0.026,   0.026,  -0.026 },
                            // {-0.0185, 0.0185,  -0.0185, 0.0185},
                            // {0.02138+0.032, 0.02138+0.032, -0.02138+0.032, -0.02138+0.032}};

    // const Eigen::MatrixXd X = P.transpose() * (P*P.transpose()).inverse();
    // const Eigen::Vector4f X1 = X.col(0);

    // const Eigen::Vector4f X1_{0.666724, 0.666866, -0.1668, -0.16679};

    Eigen::Matrix3f R1(float q1)
    {
        Eigen::Matrix3f R;
        R << 1, 0, 0,
            0, cos(q1), -sin(q1),
            0, sin(q1), cos(q1);
        return R;
    }

    Eigen::Matrix3f R2(float q2)
    {
        Eigen::Matrix3f R;
        R << cos(q2), 0, sin(q2),
            0, 1, 0,
            -sin(q2), 0, cos(q2);
        return R;
    }

    Eigen::Matrix3f R3(float q3)
    {
        Eigen::Matrix3f R;
        R << cos(q3), -sin(q3), 0,
            sin(q3), cos(q3), 0,
            0, 0, 1;
        return R;
    }

    Eigen::Matrix3f jacobian_w2euler(const float q1, const float q2)
    {
        Eigen::Matrix3f J;

        J << cos(q2), 0, sin(q2),
            sin(q2) * tan(q1), 1, -cos(q2) * tan(q1),
            -sin(q2) / cos(q1), 0, cos(q2) / cos(q1);

        return J;
    }

    Eigen::Vector3f average_vecs(Eigen::Matrix<float, 3, N_IMUS> m)
    {
        return m.rowwise().mean();
    }


    Eigen::Vector3f estimate_gyro(const Eigen::Matrix<float, 3, N_IMUS>& w_B, const Eigen::Vector3f& q)
    {
        Eigen::Matrix3f J = jacobian_w2euler(q(0), q(1));
        Eigen::Vector3f w_avg = average_vecs(w_B);
        return J * w_avg;
    }

    Eigen::Vector3f integrate(Eigen::Vector3f& dq, Eigen::Vector3f& past_q, Eigen::Vector3f& past_dq, const float& dt)
    {
        return (dq + past_dq) / 2.0 * dt  + past_q;
    }

    Eigen::Vector3f calculate_g_B(const Eigen::Matrix<float, 3, N_IMUS> m_B, const Eigen::Vector3f& pivot_acc, const Eigen::Matrix<float, N_IMUS, 1> X1)
    {
        const auto M = m_B.colwise() - pivot_acc;
        return M * X1;
    }

    Eigen::Vector2f estimate_accel(const Eigen::Vector3f& g_B)
    {
        Eigen::Vector2f q_A;

        q_A(0) = atan(g_B(1) / sqrt(g_B(0) * g_B(0) + g_B(2) * g_B(2)));

        q_A(1) = - atan(g_B(0) / g_B(2));

        return q_A;
    }

    Eigen::Vector3f estimate_pivot_accel(const Eigen::Vector3f& q, const Eigen::Vector3f& dq, const Eigen::Vector3f& ddq, const float dq4, const float ddq4)
    {
        Eigen::Vector3f ddp_WC, ddp_CI;

        const float c1 = cos(q(0));
        const float s1 = sin(q(0));

        Eigen::Vector3f temp1;
        temp1 << 2 * c1 * r * dq(0) * dq(2) + r * s1 * ddq(2),
            -c1 * r * ddq(0) + r * s1 * dq(0) * dq(0) + r * s1 * dq(2) * dq(2),
            -c1 * r * dq(0) * dq(0) - r * s1 * ddq(0);

        ddp_WC = R2(q(1)).transpose() * R1(q(0)).transpose() * temp1;

        Eigen::Vector3f temp2;
        temp2 << r * ddq4,
            r * dq(2) * dq4,
            0;

        ddp_CI = R2(q(1)).transpose() * R1(q(0)).transpose() * temp2;

        return ddp_WC + ddp_CI;
    }



    // Boolean if Wheelbot upside down:
    bool upside_down;

    // Pivot point acceleration:
    Eigen::Vector3f pivot_accel = Eigen::Vector3f::Zero();

    // Gyro Estimations:
    Eigen::Vector3f q_G = Eigen::Vector3f::Zero();
    Eigen::Vector3f dq_G = Eigen::Vector3f::Zero();

    // Accel Estimations:
    Eigen::Vector2f q_A = Eigen::Vector2f::Zero();

    // Euler Angles and derivatives:
    Eigen::Vector3f q = Eigen::Vector3f::Zero();
    Eigen::Vector3f dq = Eigen::Vector3f::Zero();
    Eigen::Vector3f ddq = Eigen::Vector3f::Zero();

    // Encoder:
    Eigen::Matrix<float, N_MOTORS, 1> q_WR   = Eigen::Matrix<float, N_MOTORS, 1>::Zero();
    Eigen::Matrix<float, N_MOTORS, 1> dq_WR  = Eigen::Matrix<float, N_MOTORS, 1>::Zero();
    Eigen::Matrix<float, N_MOTORS, 1> ddq_WR = Eigen::Matrix<float, N_MOTORS, 1>::Zero();

    // descretization timestep
    static constexpr float dt = 1.e-3;

    // Radius of wheelbot wheel in [m]:
    static constexpr float r = 32e-3;

    // Fusion parameter for complementary filter:
    static constexpr float alpha = 0.02;
    // static constexpr float alpha = 0.00;

    bool init = true;

};

#endif // ESTIMATOR_HPP
