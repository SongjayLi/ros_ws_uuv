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
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/sensor_baro.hpp"
#include "matrix/math.hpp"


using std::placeholders::_1;//传递参数的占位符，有一个参数时使用_1，有两个参数时使用_1,_2

using matrix::Eulerf;
using matrix::Quatf;
using matrix::Matrix3f;
using matrix::Vector3f;
using matrix::Dcmf;

using namespace std::chrono_literals;

class IncPID
{
public:
	float IncPIDCalc(float &CurrentPoint,float &SetPoint,float Proportion,float Integral,float Derivative) {
	float iError=SetPoint-CurrentPoint;                                     // 计算当前误差
	m_iIncpid += (Proportion * (iError - m_Error1)                  // P
		+ Integral * iError                                   // I
		+ Derivative * (iError - 2 * m_Error1 + m_Error2));  // D

	m_Error2 = m_Error1;                          // 存储误差，用于下次计算
	m_Error1 = iError;

	return(m_iIncpid);                                    // 返回增量值
	}
	IncPID(){};
	~IncPID(){};
private:


	float m_Error1 = 0;
	float m_Error2 = 0;
	float m_iIncpid = 0;
};

class joy_ctrl:public rclcpp::Node
{
    private:

        float _battery_voltage{14.0f};
        float _ctrl_input[8]{0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f} ;
        bool _input_update{false};
        bool _depth_ctrl{false};
        bool _yaw_ctrl{false};
        bool _roll_ctrl{false};
        float _T_min{0.0f};
        float _T_max{0.0f};//实时跟新推力最大值
        float _x_max{6.0f};//实时更新推力最小值
        float _y_max{6.0f};//控制增益
        float _z_max{6.0f};//控制增益
        float _roll_max{1.3f};//控制增益
        float _yaw_max{4.5f};//控制增益

        float _depth{0.0f};
        float _depth_set{0.5f};
        float _roll{0.0f};
        float _roll_set{0.0f};
        float _yaw{0.0f};
        float _yaw_set{0.0f};

        float _depth_p{1.0f};
        float _depth_i{0.000f};
        float _depth_d{0.000f};
        float _roll_p{1.00f};
        float _roll_i{0.000f};
        float _roll_d{0.000f};
        float _yaw_p{1.0f};
        float _yaw_i{0.000f};
        float _yaw_d{0.000f};

        power_dis_UUV6 _power_dis;

        IncPID m_IncPID_depth;
	    IncPID m_IncPID_roll;
	    IncPID m_IncPID_yaw;

    
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr Joy_sub_;
        rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr BatteryStatus_sub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr VehicleAttitude_sub_;
        rclcpp::Subscription<px4_msgs::msg::SensorBaro>::SharedPtr SensorBaro_sub_;

        rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr ActuatorMotors_pub_;

        rclcpp::TimerBase::SharedPtr Timer_Control_;

        void Joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void BatteryStatus_callback(const px4_msgs::msg::BatteryStatus::SharedPtr msg);
        void VehicleAttitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
        void SensorBaro_callback(const px4_msgs::msg::SensorBaro::SharedPtr msg);


        void Timer_Control_callback();

        void Pub_ActuatorMotors(const float & x, const float & y, const float & z, const float & roll, const float & pitch, const float & yaw );

    public:
        joy_ctrl(std::string node_name):Node(node_name){
            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
            rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        
            Joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&joy_ctrl::Joy_callback, this, _1));
            BatteryStatus_sub_ = this->create_subscription<px4_msgs::msg::BatteryStatus>("fmu/out/battery_status", qos, std::bind(&joy_ctrl::BatteryStatus_callback, this, _1));
            VehicleAttitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>("fmu/out/vehicle_attitude", qos, std::bind(&joy_ctrl::VehicleAttitude_callback, this, _1));
            SensorBaro_sub_ = this->create_subscription<px4_msgs::msg::SensorBaro>("fmu/out/sensor_baro", qos, std::bind(&joy_ctrl::SensorBaro_callback, this, _1));
        
            ActuatorMotors_pub_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>("fmu/in/actuator_motors", 10);
        
            Timer_Control_ = this->create_wall_timer(20ms, std::bind(&joy_ctrl::Timer_Control_callback, this));
        }
        ~joy_ctrl(){};
};

#endif