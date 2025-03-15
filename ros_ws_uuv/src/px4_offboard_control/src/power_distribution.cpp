#include "power_distribution.h"

Eigen::Matrix<float, 6, 1> power_dis_UUV6::virtual2real_force(const Eigen::Matrix<float, 6, 1> &virtual_input)
{
    Eigen::Matrix<float, 6, 1> real_input;
    real_input = _dis_matrix_inv * virtual_input;
    return real_input;
}

Eigen::Matrix<float, 6 ,1> power_dis_UUV6::force2pwm(const Eigen::Matrix<float, 6, 1> &force_input, const float &battery_voltage, const float &T_min, const float &T_max)
{
    Eigen::Matrix<float, 6, 1> pwm_output;
    //float T_min = 0.0022 * battery_voltage * battery_voltage - 0.4984* battery_voltage + 1.3815;
    //float T_max = 0.0067 * battery_voltage * battery_voltage + 0.4115* battery_voltage - 0.4524;
    for (int i = 0; i < 6; ++i) {
        float T = force_input[i];
        if (T > T_max) T = T_max;
        if (T < T_min) T = T_min;
        float U = battery_voltage;
        float a = 0.0f, b = 0.0f, c = 0.0f;

        if (T >= 0) {  // 对应 X≥1500, T≥0 的情况
            a = 0.2697 * U * U - 5.3427 * U + 37.0661;
            b = -0.9432 * U * U + 19.8613 * U - 132.3692;
            c = 0.8143 * U * U -132.3692 * U + -132.3692 - T;
        } else {  // 对应 X<1500, T≤0 的情况
            a = -0.1345 * U * U + 2.4266 * U - 18.709;
            b = 0.3242 * U * U - 4.8592 * U + 42.975;
            c = -0.1875 * U * U + 1.9342 * U - 22.8845 - T;
        }

        // 求解二次方程 ax² + bx + c = 0
        float discriminant = b * b - 4 * a * c;
        if (discriminant >= 0) {
            float sqrt_disc = std::sqrt(discriminant);
            float x1 = (-b + sqrt_disc) / (2 * a);
            float x2 = (-b - sqrt_disc) / (2 * a);
            float valid_pwm = 0.0f;

            if (T >= 0) {  // 筛选 1500-2000
                bool x1_valid = x1 >= 1.5 && x1 <= 2;
                bool x2_valid = x2 >= 1.5 && x2 <= 2;
                if (x1_valid && x2_valid) valid_pwm = x1;
                else if (x1_valid) valid_pwm = x1;
                else if (x2_valid) valid_pwm = x2;
            } else {  // 筛选 1000-1500
                bool x1_valid = x1 >= 1 && x1 <= 1.5;
                bool x2_valid = x2 >= 1 && x2 <= 1.5;
                if (x1_valid && x2_valid) valid_pwm = x1;
                else if (x1_valid) valid_pwm = x1;
                else if (x2_valid) valid_pwm = x2;
            }

            pwm_output[i] = 2.f * valid_pwm - 3.f;
        } else {
            pwm_output[i] = 0.0f;  // 无解时赋默认值
        }
    }

    return pwm_output;
}

void power_dis_UUV6::Vector2array(const Eigen::Matrix<float, 6, 1> &input, float *output)
{
    for (int i = 0; i < input.size(); ++i) {
        output[i] = input[i];
    }
}

power_dis_UUV6::power_dis_UUV6()
{
    _dis_matrix <<
    0.707,   0.707,  -0.707,  -0.707,   0,       0,
    -0.707,  0.707,  -0.707,   0.707,   0,       0,
    0,       0,       0,       0,      -1,       1,
    0.06,  -0.06,    0.06,   -0.06,  -0.111,  -0.111,
    0.06,    0.06,   -0.06,   -0.06,    0,       0,
    -0.1888, 0.1888,  0.1888, -0.1888,   0,       0;
    //if (_dis_matrix.rows() == _dis_matrix.cols()) {
        // 方阵求逆
       // _dis_matrix_inv = _dis_matrix.inverse();
    //} else {
        // 非方阵求伪逆
        _dis_matrix_inv = _dis_matrix.completeOrthogonalDecomposition().pseudoInverse();
    //}
}
