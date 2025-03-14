# ifndef POWER_DISTRIBUTION_HPP
# define POWER_DISTRIBUTION_HPP

#include <Eigen/Dense>
#include <iostream>

class power_dis_UUV6{
    private:
        Eigen::Matrix<float, 6, 6> _dis_matrix;
        Eigen::Matrix<float, 6, 6> _dis_matrix_inv;
    public:
        power_dis_UUV6();
        ~power_dis_UUV6(){};
        Eigen::Vector<float, 6> virtual2real_force(const Eigen::Vector<float, 6> & virtual_input); 
        Eigen::Vector<float, 6> force2pwm(const Eigen::Vector<float, 6> & force_input, const float & battery_voltage);
        void Vector2array(const Eigen::Vector<float, 6> & input, float *output);
};

#endif