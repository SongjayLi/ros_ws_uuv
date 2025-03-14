#ifndef JOY_CTRL_HPP
#define JOY_CTRL_HPP

#include <chrono>

#include "power_distribution.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "px4_msgs/msg/actuator_motors.hpp"
#include "px4_msgs/msg/battery_status.hpp"

using std::placeholders::_1;//传递参数的占位符，有一个参数时使用_1，有两个参数时使用_1,_2

using namespace std::chrono_literals;

class joy_ctrl:public rclcpp::Node
{
    private:

        float _battery_voltage{0.0f};
        float _ctrl_input[8]{0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f} ;
        bool _input_update{false};
    
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr Joy_sub_;
        rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr BatteryStatus_sub_;

        rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr ActuatorMotors_pub_;

        rclcpp::TimerBase::SharedPtr Timer_Control_;

        void Joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void BatteryStatus_callback(const px4_msgs::msg::BatteryStatus::SharedPtr msg);

        void Timer_Control_callback();

        void Pub_ActuatorMotors(const float & x, const float & y, const float & z, const float & roll, const float & pitch, const float & yaw );

    public:
        joy_ctrl(std::string node_name);
        ~joy_ctrl(){};
};

#endif