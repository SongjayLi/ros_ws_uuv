#ifndef _ros_joy_h
#define _ros_joy_h

#include <chrono>
#include "rclcpp/rclcpp.hpp"
//#include <QObject>
//#include <QSharedPointer>
#include "sensor_msgs/msg/joy.hpp"
#include "px4_msgs/msg/manual_control_setpoint.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"
#include "px4_msgs/msg/actuator_motors.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_angular_velocity.hpp"
#include "px4_msgs/msg/battery_status.hpp"
//#include "mainwindow.h"
#include <string>
#include "QVector"
#include "data_unit.h"
#include "matrix/math.hpp"

using std::placeholders::_1;//传递参数的占位符，有一个参数时使用_1，有两个参数时使用_1,_2
using namespace std::chrono_literals;

class MainWindow; // 前置声明

class joy_ctrl:public rclcpp::Node
{
private:

    QVector<DataWithTimestamp> m_list_pos;
    QVector<DataWithTimestamp> m_list_vel_b;
    QVector<DataWithTimestamp> m_list_att;
    QVector<DataWithTimestamp> m_list_ang_vel;
    matrix::Quaternionf m_now_q;

    void joy_ctrl_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    void timer_callback_manual_control_setpoint();
    void timer_callback_offboard_control();

    void ActuatorMotors_callback(const px4_msgs::msg::ActuatorMotors::SharedPtr msg);
    void sensor_combine_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg);
    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void vehicle_angular_velocity_callback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg);
    void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);

    void publish_manual_control_setpoint(uint8_t data_source, float roll, float pitch, float yaw, float throttle, bool sticks_moving);
    void publish_offboard_control_mode();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr Joy_sub_;
    rclcpp::Subscription<px4_msgs::msg::ActuatorMotors>::SharedPtr ActuatorMotors_sub_;
    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr SensorCombined_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr VehicleAttitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr VehicleLocalPosition_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr VehicleAngularVelocity_sub_;

    rclcpp::Publisher<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_control_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
	//rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

    rclcpp::TimerBase::SharedPtr timer_manual_control_setpoint;
    rclcpp::TimerBase::SharedPtr timer_offboard_control_mode;

    std::shared_ptr<MainWindow> m_MainWindows;

    void arm();//使用命令解锁机器人
	void disarm();//使用命令取消解锁机器人

    uint8_t m_offboard_setpoint_counter = 0;

public:
    _Float32 m_Ctrl_input[8] = {0,0,0,0,0,0,0,0};
    _Float32 m_Ctrl_input_old[8] = {0,0,0,0,0,0,0,0};
    uint32_t m_sample_time;
    uint32_t m_sample_time_old;
    bool m_sticks_moving = 0;
    joy_ctrl(std::string name);
    ~joy_ctrl() override{
        //delete m_MainWindows;
    };
    void start_offboard_control();
    void stop_offboard_control();
};

#endif