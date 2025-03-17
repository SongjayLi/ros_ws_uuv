#include "joy_ctrl.h"

void joy_ctrl::Joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    _ctrl_input[0] = msg->axes[0];//左摇杆左右：axes0 1 - -1
    _ctrl_input[1] = msg->axes[1];//左摇杆前后：axes1 1 - -1
    _ctrl_input[2] = msg->axes[2];//右摇杆左右：axes2 1 - -1
    _ctrl_input[3] = msg->axes[3];//右摇杆前后：axes3 1 - -1
    _ctrl_input[4] = msg->axes[4];//右摇杆前后：axes4 1 - -1
    _ctrl_input[5] = msg->axes[5];//右肩键按下：axes5 1 - -1
    _ctrl_input[6] = msg->axes[6];
    _ctrl_input[7] = msg->axes[7];
    _depth_ctrl = msg->buttons[0];//X键按下：buttons0 1 - -1
    _yaw_ctrl = msg->buttons[1];//A键按下：buttons1 1 - -1
    _roll_ctrl = msg->buttons[2];//B键按下：buttons2 1 - -1
    _input_update = true;
    //RCLCPP_INFO(this->get_logger(), "更新按键值");
}

void joy_ctrl::BatteryStatus_callback(const px4_msgs::msg::BatteryStatus::SharedPtr msg)
{
    _battery_voltage = msg->voltage_filtered_v;
}

void joy_ctrl::VehicleAttitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
    Quatf q_att(msg->q[0],msg->q[1],msg->q[2],msg->q[3]);
    Eulerf euler_att(q_att);
    _roll = euler_att.phi();
    _yaw = euler_att.psi();
    std::cout << "roll: " << _roll << " yaw: " << _yaw << std::endl;
}

void joy_ctrl::SensorBaro_callback(const px4_msgs::msg::SensorBaro::SharedPtr msg)
{
    _depth = (msg->pressure-103000)/10000.0f;
}

void joy_ctrl::Timer_Control_callback()
{
    float depth_out;
    float roll_out;
    float yaw_out;
    if(_depth_ctrl){
        depth_out = m_IncPID_depth.IncPIDCalc(_depth,_depth_set,_depth_p,_depth_i,_depth_d);
        std::cout<<"深度控制:"<<depth_out<<std::endl;
    } else { depth_out =  -_ctrl_input[1]*_z_max;}
    if(_roll_ctrl){
        roll_out = m_IncPID_roll.IncPIDCalc(_roll,_roll_set,_roll_p,_roll_i,_roll_d);
        std::cout<<"横滚控制:"<<roll_out<<std::endl;
    } else { roll_out =  0;}
    if(_yaw_ctrl){
        yaw_out = m_IncPID_yaw.IncPIDCalc(_yaw,_yaw_set,_yaw_p,_yaw_i,_yaw_d);
        std::cout<<"航向控制:"<<yaw_out<<std::endl;
    } else { yaw_out =  -_ctrl_input[0]*_yaw_max;}
    if(_input_update || _depth_ctrl || _yaw_ctrl || _roll_ctrl){
        _input_update = false;
        _T_max = 0.0067 * _battery_voltage * _battery_voltage + 0.4115* _battery_voltage - 0.4524;
        _T_min = 0.0022 * _battery_voltage * _battery_voltage - 0.4984* _battery_voltage + 1.3815;

        Eigen::Matrix<float, 6, 1> Vforce_input;
        Vforce_input << 
        _ctrl_input[3]*_x_max, -_ctrl_input[2]*_y_max, depth_out, roll_out, _ctrl_input[3]*_x_max * 0.0849, yaw_out;
        //std::cout << "force_input: " << Vforce_input.transpose() << std::endl;
        Eigen::Matrix<float, 6, 1> force_input = _power_dis.virtual2real_force(Vforce_input);
        //std::cout << "force_input: " << force_input.transpose() << std::endl;
        float max_force = force_input.maxCoeff();
        float min_force = force_input.minCoeff();
        // 检查是否有推力大于最大或小于最小
        if (max_force > _T_max || min_force < _T_min) {
            // 计算缩放因子
            float scale = std::min(_T_max / max_force, (_T_min / min_force));
            // 等比缩放所有值
            force_input *= scale;
        }
        //std::cout << "force_input: " << force_input.transpose() << std::endl;
        float pwm[6];
        std::cout << "force_input: " << _power_dis.force2pwm(force_input, _battery_voltage, _T_min, _T_max).transpose() << std::endl;
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
    msg.control[6] = std::nanf("");
    msg.control[7] = std::nanf("");
    msg.control[8] = std::nanf("");
    msg.control[9] = std::nanf("");
    msg.control[10] = std::nanf("");
    msg.control[11] = std::nanf("");
    msg.reversible_flags = 255;
    ActuatorMotors_pub_->publish(msg);
}


