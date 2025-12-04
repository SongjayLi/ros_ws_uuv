#include "joy_ctrl.h"

// =================== 构造函数 ===================

joy_ctrl::joy_ctrl(std::string node_name)
: Node(node_name)
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // 1) 先初始化参数与 ADRC
    init_parameters_and_adrc();

    // 2) 创建订阅 / 发布 / 定时器
    Joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&joy_ctrl::Joy_callback, this, _1));

    BatteryStatus_sub_ = this->create_subscription<px4_msgs::msg::BatteryStatus>(
        "fmu/out/battery_status", qos,
        std::bind(&joy_ctrl::BatteryStatus_callback, this, _1));

    VehicleAttitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        "fmu/out/vehicle_attitude", qos,
        std::bind(&joy_ctrl::VehicleAttitude_callback, this, _1));

    SensorBaro_sub_ = this->create_subscription<px4_msgs::msg::SensorBaro>(
        "fmu/out/sensor_baro", qos,
        std::bind(&joy_ctrl::SensorBaro_callback, this, _1));

    ActuatorMotors_pub_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>(
        "fmu/in/actuator_motors", 10);

    Timer_Control_ = this->create_wall_timer(
        std::chrono::duration<double>(_ctrl_dt),
        std::bind(&joy_ctrl::Timer_Control_callback, this));
}

// =================== 参数加载 & ADRC 初始化 ===================

void joy_ctrl::init_parameters_and_adrc()
{
    // ---------- 控制周期 ----------
    this->declare_parameter<double>("control_dt", 0.02);
    double dt_param = 0.02;
    this->get_parameter("control_dt", dt_param);
    _ctrl_dt = static_cast<float>(dt_param);

    // ---------- 是否启用 ADRC ----------
    this->declare_parameter<bool>("use_adrc_roll", true);
    this->declare_parameter<bool>("use_adrc_yaw", true);
    this->get_parameter("use_adrc_roll", _use_adrc_roll);
    this->get_parameter("use_adrc_yaw", _use_adrc_yaw);

    // ---------- 横滚 ADRC 参数 ----------
    this->declare_parameter<double>("adrc_roll_b0",      3.0);
    this->declare_parameter<double>("adrc_roll_td_r",    20.0);
    this->declare_parameter<double>("adrc_roll_td_zeta", 1.0);
    this->declare_parameter<double>("adrc_roll_omega_o", 40.0);
    this->declare_parameter<double>("adrc_roll_omega_c", 10.0);
    this->declare_parameter<double>("adrc_roll_zeta_c",  0.9);
    this->declare_parameter<double>("adrc_roll_u_min",  -3.0);
    this->declare_parameter<double>("adrc_roll_u_max",   3.0);
    this->declare_parameter<double>("adrc_roll_dead_zone", 0.0);

    AdrcParams roll_p;
    roll_p.dt = _ctrl_dt;
    this->get_parameter("adrc_roll_b0",      dt_param); roll_p.b0        = static_cast<float>(dt_param);
    this->get_parameter("adrc_roll_td_r",    dt_param); roll_p.td_r      = static_cast<float>(dt_param);
    this->get_parameter("adrc_roll_td_zeta", dt_param); roll_p.td_zeta   = static_cast<float>(dt_param);
    this->get_parameter("adrc_roll_omega_o", dt_param); roll_p.omega_o   = static_cast<float>(dt_param);
    this->get_parameter("adrc_roll_omega_c", dt_param); roll_p.omega_c   = static_cast<float>(dt_param);
    this->get_parameter("adrc_roll_zeta_c",  dt_param); roll_p.zeta_c    = static_cast<float>(dt_param);
    this->get_parameter("adrc_roll_u_min",   dt_param); roll_p.u_min     = static_cast<float>(dt_param);
    this->get_parameter("adrc_roll_u_max",   dt_param); roll_p.u_max     = static_cast<float>(dt_param);
    this->get_parameter("adrc_roll_dead_zone", dt_param); roll_p.dead_zone = static_cast<float>(dt_param);

    _adrc_roll.configure(roll_p);

    // ---------- 航向 ADRC 参数 ----------
    this->declare_parameter<double>("adrc_yaw_b0",      2.0);
    this->declare_parameter<double>("adrc_yaw_td_r",    20.0);
    this->declare_parameter<double>("adrc_yaw_td_zeta", 1.0);
    this->declare_parameter<double>("adrc_yaw_omega_o", 35.0);
    this->declare_parameter<double>("adrc_yaw_omega_c", 8.0);
    this->declare_parameter<double>("adrc_yaw_zeta_c",  0.9);
    this->declare_parameter<double>("adrc_yaw_u_min",  -4.0);
    this->declare_parameter<double>("adrc_yaw_u_max",   4.0);
    this->declare_parameter<double>("adrc_yaw_dead_zone", 0.0);

    AdrcParams yaw_p;
    yaw_p.dt = _ctrl_dt;
    this->get_parameter("adrc_yaw_b0",      dt_param); yaw_p.b0        = static_cast<float>(dt_param);
    this->get_parameter("adrc_yaw_td_r",    dt_param); yaw_p.td_r      = static_cast<float>(dt_param);
    this->get_parameter("adrc_yaw_td_zeta", dt_param); yaw_p.td_zeta   = static_cast<float>(dt_param);
    this->get_parameter("adrc_yaw_omega_o", dt_param); yaw_p.omega_o   = static_cast<float>(dt_param);
    this->get_parameter("adrc_yaw_omega_c", dt_param); yaw_p.omega_c   = static_cast<float>(dt_param);
    this->get_parameter("adrc_yaw_zeta_c",  dt_param); yaw_p.zeta_c    = static_cast<float>(dt_param);
    this->get_parameter("adrc_yaw_u_min",   dt_param); yaw_p.u_min     = static_cast<float>(dt_param);
    this->get_parameter("adrc_yaw_u_max",   dt_param); yaw_p.u_max     = static_cast<float>(dt_param);
    this->get_parameter("adrc_yaw_dead_zone", dt_param); yaw_p.dead_zone = static_cast<float>(dt_param);

    _adrc_yaw.configure(yaw_p);

    RCLCPP_INFO(this->get_logger(),
                "ADRC params loaded: dt=%.4f, use_roll=%d, use_yaw=%d",
                _ctrl_dt, _use_adrc_roll, _use_adrc_yaw);
}

// =================== 话题回调 ===================

void joy_ctrl::Joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    _ctrl_input[0] = msg->axes[0]; // 左摇杆左右
    _ctrl_input[1] = msg->axes[1]; // 左摇杆前后
    _ctrl_input[2] = msg->axes[2]; // 右摇杆左右
    _ctrl_input[3] = msg->axes[3]; // 右摇杆前后
    _ctrl_input[4] = msg->axes[4];
    _ctrl_input[5] = msg->axes[5];
    _ctrl_input[6] = msg->axes[6];
    _ctrl_input[7] = msg->axes[7];

    _depth_ctrl = msg->buttons[0]; // X 键：深度闭环
    _yaw_ctrl   = msg->buttons[1]; // A 键：航向闭环
    _roll_ctrl  = msg->buttons[2]; // B 键：横滚闭环

    _input_update = true;
}

void joy_ctrl::BatteryStatus_callback(const px4_msgs::msg::BatteryStatus::SharedPtr msg)
{
    _battery_voltage = msg->voltage_filtered_v;
}

void joy_ctrl::VehicleAttitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
    Quatf q_att(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    Eulerf euler_att(q_att);

    _roll = euler_att.phi();
    _yaw  = euler_att.psi();

    // 可以先注释掉大量打印
    // std::cout << "roll: " << _roll << " yaw: " << _yaw << std::endl;
}

void joy_ctrl::SensorBaro_callback(const px4_msgs::msg::SensorBaro::SharedPtr msg)
{
    _depth = (msg->pressure - 103000) / 10000.0f;
}

// =================== 控制主循环 ===================

void joy_ctrl::Timer_Control_callback()
{
    float depth_out{0.0f};
    float roll_out{0.0f};
    float yaw_out{0.0f};

    // ---- 深度：沿用原来的增量 PID ----
    if (_depth_ctrl) {
        depth_out = m_IncPID_depth.IncPIDCalc(_depth, _depth_set,
                                               _depth_p, _depth_i, _depth_d);
    } else {
        depth_out = -_ctrl_input[1] * _z_max;
    }

    // ---- 横滚：可选 ADRC / PID ----
    if (_roll_ctrl) {
        if (_use_adrc_roll) {
            roll_out = _adrc_roll.step(_roll_set, _roll);
        } else {
            roll_out = m_IncPID_roll.IncPIDCalc(_roll, _roll_set,
                                                 _roll_p, _roll_i, _roll_d);
        }
    } else {
        roll_out = 0.0f;
        _adrc_roll.reset();   // 退出闭环时重置 ESO，避免残余
    }

    // ---- 航向：可选 ADRC / PID ----
    if (_yaw_ctrl) {
        if (_use_adrc_yaw) {
            yaw_out = _adrc_yaw.step(_yaw_set, _yaw);
        } else {
            yaw_out = m_IncPID_yaw.IncPIDCalc(_yaw, _yaw_set,
                                              _yaw_p, _yaw_i, _yaw_d);
        }
    } else {
        yaw_out = -_ctrl_input[0] * _yaw_max;
        _adrc_yaw.reset();
    }

    // ---- 推力分配与 PWM 计算 ----
    if (_input_update || _depth_ctrl || _yaw_ctrl || _roll_ctrl) {

        _input_update = false;

        _T_max = 0.0067f * _battery_voltage * _battery_voltage
                 + 0.4115f * _battery_voltage - 0.4524f;
        _T_min = 0.0022f * _battery_voltage * _battery_voltage
                 - 0.4984f * _battery_voltage + 1.3815f;

        Eigen::Matrix<float, 6, 1> Vforce_input;
        Vforce_input <<
            _ctrl_input[3] * _x_max,      // surge
            -_ctrl_input[2] * _y_max,     // sway
            depth_out,                    // heave
            roll_out,                     // roll moment
            _ctrl_input[3] * _x_max * 0.0849f, // pitch moment (简化)
            yaw_out;                      // yaw moment

        Eigen::Matrix<float, 6, 1> force_input =
            _power_dis.virtual2real_force(Vforce_input);

        float max_force = force_input.maxCoeff();
        float min_force = force_input.minCoeff();

        if (max_force > _T_max || min_force < _T_min) {
            float scale = std::min(_T_max / max_force, _T_min / min_force);
            force_input *= scale;
        }

        float pwm[6];
        auto pwm_vec = _power_dis.force2pwm(force_input,
                                            _battery_voltage,
                                            _T_min, _T_max);
        _power_dis.Vector2array(pwm_vec, pwm);

        // 可视化调试时打印
        // std::cout << "pwm: "
        //           << pwm_vec.transpose() << std::endl;

        Pub_ActuatorMotors(pwm[0], pwm[1], pwm[2],
                           pwm[3], pwm[4], pwm[5]);
    }
}

// =================== 发布 PWM ===================

void joy_ctrl::Pub_ActuatorMotors(const float &x, const float &y, const float &z,
                                  const float &roll, const float &pitch, const float &yaw)
{
    px4_msgs::msg::ActuatorMotors msg;
    msg.timestamp        = this->get_clock()->now().nanoseconds() / 1000;
    msg.timestamp_sample = msg.timestamp;

    msg.control[0] = x;
    msg.control[1] = y;
    msg.control[2] = z;
    msg.control[3] = roll;
    msg.control[4] = pitch;
    msg.control[5] = yaw;

    for (int i = 6; i < 12; ++i) {
        msg.control[i] = std::nanf("");
    }

    msg.reversible_flags = 255;
    ActuatorMotors_pub_->publish(msg);
}
