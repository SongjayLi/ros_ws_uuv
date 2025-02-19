#include "ros_joy.h"
#include "mainwindow.h"

void joy_ctrl::joy_ctrl_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    std::memcpy(m_Ctrl_input_old, m_Ctrl_input, sizeof(m_Ctrl_input));
    m_Ctrl_input[0] = msg->axes[0];//左摇杆左右：axes0 1 - -1
    m_Ctrl_input[1] = msg->axes[1];//左摇杆前后：axes1 1 - -1
    m_Ctrl_input[2] = msg->axes[2];//左肩键按下：axes2 1 - -1
    m_Ctrl_input[3] = msg->axes[3];//右摇杆左右：axes3 1 - -1
    m_Ctrl_input[4] = msg->axes[4];//右摇杆前后：axes4 1 - -1
    m_Ctrl_input[5] = msg->axes[5];//右肩键按下：axes5 1 - -1
    m_Ctrl_input[6] = msg->axes[6];
    m_Ctrl_input[7] = msg->axes[7];
    m_sample_time = msg->header.stamp.nanosec;
    m_sticks_moving = !std::equal(std::begin(m_Ctrl_input), std::end(m_Ctrl_input), std::begin(m_Ctrl_input_old));
    m_MainWindows->Update_input_num(m_Ctrl_input);
}

/**
 * @brief Callback function for manual control setpoint timer.
 * 
 * This function is called periodically to publish manual control setpoints.
 * It checks if the sample time has changed since the last callback. If it has,
 * it publishes the current control inputs. If not, it publishes a default setpoint.
 */
void joy_ctrl::timer_callback_manual_control_setpoint()
{
    // Check if the sample time has changed since the last callback
    if(m_sample_time == m_sample_time_old){
        // If the sample time has not changed, publish a default setpoint
        //std::fill(std::begin(m_Ctrl_input), std::end(m_Ctrl_input), 0);
        publish_manual_control_setpoint(2,0,0,0,0,false);
    }
    else{
        // If the sample time has changed, publish the current control inputs
        publish_manual_control_setpoint(2, -m_Ctrl_input[3], m_Ctrl_input[4], -m_Ctrl_input[0], m_Ctrl_input[1], m_sticks_moving);
    }
    // Update the old sample time with the current sample time
    m_sample_time_old = m_sample_time;
    //auto m_ManualControlSetpoint = px4_msgs::msg::ManualControlSetpoint();
    //m_ManualControlSetpoint.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
    //m_ManualControlSetpoint.timestamp_sample = m_ManualControlSetpoint.timestamp;
    //m_ManualControlSetpoint.data_source = 2;
    //m_ManualControlSetpoint.roll = -m_Ctrl_input[3];
    //m_ManualControlSetpoint.pitch = m_Ctrl_input[4];
    //m_ManualControlSetpoint.yaw = -m_Ctrl_input[0];
    //m_ManualControlSetpoint.throttle = (m_Ctrl_input[1]+1)/2;
    //m_ManualControlSetpoint.sticks_moving = m_sticks_moving;
    //this->manual_control_setpoint_pub_->publish(m_ManualControlSetpoint);
}

void joy_ctrl::timer_callback_offboard_control()
{
    publish_offboard_control_mode();
    if(m_offboard_setpoint_counter < 11){
        if (++m_offboard_setpoint_counter == 10) {  // 每次循环自增计数器，判断是否等于10
                this->arm();
            }
    }
    //if (offboard_setpoint_counter_ == 10) {
        // Change to Offboard mode after 10 setpoints
        //this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

        // Arm the vehicle
        //this->arm();
    //}

    // OffboardControlMode needs to be paired with TrajectorySetpoint
    //publish_offboard_control_mode();
    //publish_trajectory_setpoint();

    // stop the counter after reaching 11
    //if (offboard_setpoint_counter_ < 11) {
    //    offboard_setpoint_counter_++;
    //}
}

void joy_ctrl::ActuatorMotors_callback(const px4_msgs::msg::ActuatorMotors::SharedPtr msg)
{
    m_MainWindows->Update_actuator(msg->control);
}

void joy_ctrl::publish_manual_control_setpoint(uint8_t data_source, float roll, float pitch, float yaw, float throttle, bool sticks_moving)
{
    px4_msgs::msg::ManualControlSetpoint msg{};
    msg.data_source = data_source;
    msg.roll = roll;
    msg.pitch = pitch;
    msg.yaw = yaw;
    msg.throttle = throttle;
    msg.sticks_moving = sticks_moving;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.timestamp_sample = this->get_clock()->now().nanoseconds() / 1000;
    this->manual_control_setpoint_pub_->publish(msg);

}

void joy_ctrl::publish_offboard_control_mode() // 发布板外控制的具体实现
{
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = true;
    msg.direct_actuator = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_pub_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void joy_ctrl::publish_vehicle_command(uint16_t command, float param1, float param2)//发布参数的具体实现
{
    px4_msgs::msg::VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_pub_->publish(msg);
}

void joy_ctrl::arm()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void joy_ctrl::disarm()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

joy_ctrl::joy_ctrl(std::string name) : Node(name)
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    Joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy",//订阅的消息名称
        10,//qs和队列长度
        std::bind(&joy_ctrl::joy_ctrl_callback,this,_1)//传入回调函数，使用bind函数将成员函数转换为回调函数（回调函数名称，回调函数所属的对象，出入参数占位符）
    );
    manual_control_setpoint_pub_ = this->create_publisher<px4_msgs::msg::ManualControlSetpoint>(
        "/fmu/in/manual_control_setpoint",
        10
    );
    offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode",
        10
    );
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command",
        10
    );
    

    ActuatorMotors_sub_ = this->create_subscription<px4_msgs::msg::ActuatorMotors>(
        "/fmu/out/actuator_motors",
        qos,
        std::bind(&joy_ctrl::ActuatorMotors_callback,this,_1)
    );
    timer_manual_control_setpoint = this->create_wall_timer(50ms, std::bind(&joy_ctrl::timer_callback_manual_control_setpoint,this));
    m_MainWindows = std::make_shared<MainWindow>(nullptr, this);
    m_MainWindows->init_first();
    m_MainWindows->show();
}

void joy_ctrl::start_offboard_control()
{
    if (timer_offboard_control_mode) {
        timer_offboard_control_mode->reset();
    }
    timer_offboard_control_mode = this->create_wall_timer(400ms,std::bind(&joy_ctrl::timer_callback_offboard_control,this));

}

void joy_ctrl::stop_offboard_control()
{
    if (timer_offboard_control_mode) {
        this->disarm();
        timer_offboard_control_mode->cancel();
        m_offboard_setpoint_counter = 0;
    }
}
