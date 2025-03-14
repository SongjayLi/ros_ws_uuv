#include "joy_ctrl.hpp"

void joy_ctrl::Joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    _ctrl_input[0] = msg->axes[0];//左摇杆左右：axes0 1 - -1
    _ctrl_input[1] = msg->axes[1];//左摇杆前后：axes1 1 - -1
    _ctrl_input[2] = msg->axes[2];//左肩键按下：axes2 1 - -1
    _ctrl_input[3] = msg->axes[3];//右摇杆左右：axes3 1 - -1
    _ctrl_input[4] = msg->axes[4];//右摇杆前后：axes4 1 - -1
    _ctrl_input[5] = msg->axes[5];//右肩键按下：axes5 1 - -1
    _ctrl_input[6] = msg->axes[6];
    _ctrl_input[7] = msg->axes[7];
    _input_update = true;
}

void joy_ctrl::BatteryStatus_callback(const px4_msgs::msg::BatteryStatus::SharedPtr msg)
{
    _battery_voltage = msg->voltage_filtered_v;
}

void joy_ctrl::Timer_Control_callback()
{
}

void joy_ctrl::Pub_ActuatorMotors(const float &x, const float &y, const float &z, const float &roll, const float &pitch, const float &yaw)
{
}

joy_ctrl::joy_ctrl(std::string node_name): Node(node_name)
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    Joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&joy_ctrl::Joy_callback, this, _1));
    BatteryStatus_sub_ = this->create_subscription<px4_msgs::msg::BatteryStatus>("fmu/out/battery_status", qos, std::bind(&joy_ctrl::BatteryStatus_callback, this, _1));

    ActuatorMotors_pub_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>("fmu/in/actuator_motors", 10);

    Timer_Control_ = this->create_wall_timer(50ms, std::bind(&joy_ctrl::Timer_Control_callback, this));
}
