#include "joy_ctrl.h"

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
    //RCLCPP_INFO(this->get_logger(), "更新按键值");
}

void joy_ctrl::BatteryStatus_callback(const px4_msgs::msg::BatteryStatus::SharedPtr msg)
{
    _battery_voltage = msg->voltage_filtered_v;
}

void joy_ctrl::Timer_Control_callback()
{
    if(_input_update){
        _input_update = false;
        _T_max = 0.0067 * _battery_voltage * _battery_voltage + 0.4115* _battery_voltage - 0.4524;
        _T_min = 0.0022 * _battery_voltage * _battery_voltage - 0.4984* _battery_voltage + 1.3815;
        Eigen::Matrix<float, 6, 1> Vforce_input;
        Vforce_input << 
        _ctrl_input[4]*_x_max, _ctrl_input[5]*_y_max, -_ctrl_input[1]*_z_max, 0, _ctrl_input[4]*_x_max * 0.0849, _ctrl_input[0]*_yaw_max;
        Eigen::Matrix<float, 6, 1> force_input = _power_dis.virtual2real_force(Vforce_input);
        float max_force = force_input.maxCoeff();
        float min_force = force_input.minCoeff();
        // 检查是否有推力大于最大或小于最小
        if (max_force > _T_max || min_force < _T_min) {
            // 计算缩放因子
            float scale = std::min(_T_max / max_force, (_T_min / min_force));
            // 等比缩放所有值
            force_input *= scale;
        }
        float pwm[6];
        _power_dis.Vector2array(_power_dis.force2pwm(force_input, _battery_voltage, _T_min, _T_max),pwm);
        Pub_ActuatorMotors(pwm[0],pwm[1],pwm[2],pwm[3],pwm[4],pwm[5]);
    }
}

void joy_ctrl::Pub_ActuatorMotors(const float &x, const float &y, const float &z, const float &roll, const float &pitch, const float &yaw)
{
    px4_msgs::msg::ActuatorMotors msg;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.timestamp_sample = this->get_clock()->now().nanoseconds() / 1000;
    msg.control[0] = x;
    msg.control[1] = y;
    msg.control[2] = z;
    msg.control[3] = roll;
    msg.control[4] = pitch;
    msg.control[5] = yaw;
    msg.control[6] = 0;
    msg.control[7] = 0;
    ActuatorMotors_pub_->publish(msg);
}


