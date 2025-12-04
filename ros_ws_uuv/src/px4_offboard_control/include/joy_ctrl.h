#ifndef JOY_CTRL_HPP
#define JOY_CTRL_HPP

#include <chrono>
#include <Eigen/Dense>
#include <iostream>

#include "power_distribution.h"
#include "adrc_siso.hpp"   // ★ 新增

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "px4_msgs/msg/actuator_motors.hpp"
#include "px4_msgs/msg/battery_status.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/sensor_baro.hpp"
#include "matrix/math.hpp"

using std::placeholders::_1;

using matrix::Eulerf;
using matrix::Quatf;
using matrix::Matrix3f;
using matrix::Vector3f;
using matrix::Dcmf;

using namespace std::chrono_literals;

class IncPID
{
public:
    float IncPIDCalc(float &CurrentPoint, float &SetPoint,
                     float Proportion, float Integral, float Derivative)
    {
        float iError = SetPoint - CurrentPoint;
        m_iIncpid += (Proportion * (iError - m_Error1)
                      + Integral * iError
                      + Derivative * (iError - 2 * m_Error1 + m_Error2));

        m_Error2 = m_Error1;
        m_Error1 = iError;
        return m_iIncpid;
    }

    IncPID() = default;
    ~IncPID() = default;

private:
    float m_Error1{0.0f};
    float m_Error2{0.0f};
    float m_iIncpid{0.0f};
};

class joy_ctrl : public rclcpp::Node
{
private:

    // ------------- 基本状态与手柄输入 -------------
    float _battery_voltage{14.0f};
    float _ctrl_input[8]{0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    bool  _input_update{false};
    bool  _depth_ctrl{false};
    bool  _yaw_ctrl{false};
    bool  _roll_ctrl{false};

    float _T_min{0.0f};
    float _T_max{0.0f};
    float _x_max{6.0f};
    float _y_max{6.0f};
    float _z_max{6.0f};
    float _roll_max{1.3f};
    float _yaw_max{4.5f};

    // ------------- 反馈量与设定值 -------------
    float _depth{0.0f};
    float _depth_set{0.5f};
    float _roll{0.0f};
    float _roll_set{0.0f};
    float _yaw{0.0f};
    float _yaw_set{0.0f};

    // ------------- 原有 PID 增益（仍保留做对比） -------------
    float _depth_p{1.0f};
    float _depth_i{0.0f};
    float _depth_d{0.0f};
    float _roll_p{1.0f};
    float _roll_i{0.0f};
    float _roll_d{0.0f};
    float _yaw_p{1.0f};
    float _yaw_i{0.0f};
    float _yaw_d{0.0f};

    power_dis_UUV6 _power_dis;

    IncPID m_IncPID_depth;
    IncPID m_IncPID_roll;
    IncPID m_IncPID_yaw;

    // ------------- 新增：ADRC 相关成员 -------------
    bool _use_adrc_roll{true};   // 是否使用 ADRC 控制 roll
    bool _use_adrc_yaw{true};    // 是否使用 ADRC 控制 yaw
    float _ctrl_dt{0.02f};       // 控制周期

    AdrcSiso _adrc_roll;
    AdrcSiso _adrc_yaw;

    // ------------- ROS 通信对象 -------------
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr Joy_sub_;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr BatteryStatus_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr VehicleAttitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::SensorBaro>::SharedPtr SensorBaro_sub_;

    rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr ActuatorMotors_pub_;

    rclcpp::TimerBase::SharedPtr Timer_Control_;

    // ------------- 回调函数 -------------
    void Joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void BatteryStatus_callback(const px4_msgs::msg::BatteryStatus::SharedPtr msg);
    void VehicleAttitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    void SensorBaro_callback(const px4_msgs::msg::SensorBaro::SharedPtr msg);

    void Timer_Control_callback();

    void Pub_ActuatorMotors(const float & x, const float & y, const float & z,
                            const float & roll, const float & pitch, const float & yaw);

    void init_parameters_and_adrc();   // ★ 新增：从参数服务器加载 ADRC 参数

public:
    joy_ctrl(std::string node_name);
    ~joy_ctrl() = default;
};

#endif
