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

void joy_ctrl::timer_callback_manual_control_setpoint()
{
    if(m_sample_time == m_sample_time_old){
        std::fill(std::begin(m_Ctrl_input), std::end(m_Ctrl_input), 0);
    }
    auto m_ManualControlSetpoint = px4_msgs::msg::ManualControlSetpoint();
    m_ManualControlSetpoint.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
    m_ManualControlSetpoint.timestamp_sample = m_ManualControlSetpoint.timestamp;
    m_ManualControlSetpoint.data_source = 2;
    m_ManualControlSetpoint.roll = -m_Ctrl_input[3];
    m_ManualControlSetpoint.pitch = m_Ctrl_input[4];
    m_ManualControlSetpoint.yaw = -m_Ctrl_input[0];
    m_ManualControlSetpoint.throttle = (m_Ctrl_input[1]+1)/2;
    m_ManualControlSetpoint.sticks_moving = m_sticks_moving;
    this->manual_control_setpoint_pub_->publish(m_ManualControlSetpoint);
    m_sample_time_old = m_sample_time;
}

void joy_ctrl::timer_callback_offboard_control()
{
    if (offboard_setpoint_counter_ == 10) {
        // Change to Offboard mode after 10 setpoints
        this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

        // Arm the vehicle
        this->arm();
    }

    // OffboardControlMode needs to be paired with TrajectorySetpoint
    publish_offboard_control_mode();
    //publish_trajectory_setpoint();

    // stop the counter after reaching 11
    if (offboard_setpoint_counter_ < 11) {
        offboard_setpoint_counter_++;
    }
}

void joy_ctrl::ActuatorMotors_callback(const px4_msgs::msg::ActuatorMotors::SharedPtr msg)
{
    m_MainWindows->Update_actuator(msg->control);
}

void joy_ctrl::publish_offboard_control_mode()
{
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void joy_ctrl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
}

joy_ctrl::joy_ctrl(std::string name):Node(name)
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
