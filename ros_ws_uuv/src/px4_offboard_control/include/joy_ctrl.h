#ifndef JOY_CTRL_HPP
#define JOY_CTRL_HPP

#include <chrono>
#include <Eigen/Dense>
#include <iostream>

#include "power_distribution.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "px4_msgs/msg/actuator_motors.hpp"
#include "px4_msgs/msg/battery_status.hpp"

using std::placeholders::_1;//传递参数的占位符，有一个参数时使用_1，有两个参数时使用_1,_2

using namespace std::chrono_literals;

class joy_ctrl:public rclcpp::Node
{
    private:

        float _battery_voltage{14.0f};
        float _ctrl_input[8]{0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f} ;
        bool _input_update{false};
        float _T_min{0.0f};
        float _T_max{0.0f};
        float _x_max{6.0f};
        float _y_max{6.0f};
        float _z_max{6.0f};
        float _roll_max{1.3f};
        float _yaw_max{4.5f};
        power_dis_UUV6 _power_dis;
    
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr Joy_sub_;
        rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr BatteryStatus_sub_;

        rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr ActuatorMotors_pub_;

        rclcpp::TimerBase::SharedPtr Timer_Control_;

        void Joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void BatteryStatus_callback(const px4_msgs::msg::BatteryStatus::SharedPtr msg);

        void Timer_Control_callback();

        void Pub_ActuatorMotors(const float & x, const float & y, const float & z, const float & roll, const float & pitch, const float & yaw );

    public:
        joy_ctrl(std::string node_name):Node(node_name){
            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
            rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        
            Joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&joy_ctrl::Joy_callback, this, _1));
            BatteryStatus_sub_ = this->create_subscription<px4_msgs::msg::BatteryStatus>("fmu/out/battery_status", qos, std::bind(&joy_ctrl::BatteryStatus_callback, this, _1));
        
            ActuatorMotors_pub_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>("fmu/in/actuator_motors", 10);
        
            Timer_Control_ = this->create_wall_timer(20ms, std::bind(&joy_ctrl::Timer_Control_callback, this));
        }
        ~joy_ctrl(){};
};

#endif