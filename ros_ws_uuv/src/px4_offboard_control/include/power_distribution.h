# ifndef POWER_DISTRIBUTION_HPP
# define POWER_DISTRIBUTION_HPP

#include <Eigen/Dense>
#include <iostream>
#include <cmath>

class power_dis_UUV6{
    private:
        Eigen::Matrix<float, 6, 6> _dis_matrix;
        Eigen::Matrix<float, 6, 6> _dis_matrix_inv;
    public:
        power_dis_UUV6();
        ~power_dis_UUV6(){};
        Eigen::Matrix<float, 6, 1> virtual2real_force(const Eigen::Matrix<float, 6, 1> & virtual_input); 
        Eigen::Matrix<float, 6, 1> force2pwm(const Eigen::Matrix<float, 6, 1> & force_input, const float & battery_voltage, const float &T_min, const float &T_max);
        void Vector2array(const Eigen::Matrix<float, 6, 1> & input, float *output);
};

#endif