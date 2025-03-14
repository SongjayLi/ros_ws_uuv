#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_angular_velocity.hpp"
#include "px4_msgs/msg/actuator_motors.hpp"
#include "px4_msgs/msg/battery_status.hpp"
//#include "redisclient/redisasyncclient.h"
#include <hiredis/hiredis.h>
//1.导入消息类型
#include <string>
#include <vector>

using std::placeholders::_1;//传递参数的占位符，有一个参数时使用_1，有两个参数时使用_1,_2
using std::placeholders::_2;

class RedisWrite : public rclcpp::Node//继承了节点类
{
private:
//2.创建回调函数
    void sensor_combine_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg) const {
        //使用类型下的共享指针进行参数传递
//4.编写回调函数
        std::string command = "ZADD sensor_combined " +
                              std::to_string(msg->timestamp) + " "+ 
                              std::to_string(msg->gyro_rad[0]) + ","+ 
                              std::to_string(msg->gyro_rad[1]) + ","+ 
                              std::to_string(msg->gyro_rad[2]) + ","+ 
                              std::to_string(msg->accelerometer_m_s2[0]) + ","+ 
                              std::to_string(msg->accelerometer_m_s2[1]) + ","+
                              std::to_string(msg->accelerometer_m_s2[2]) + ","+std::to_string(msg->timestamp);
        //std::string command = "HSET sensor_combined " +
        //                      std::to_string(msg->timestamp) + " "+ 
        //                      std::to_string(msg->gyro_rad[0]) + ","+ 
        //                      std::to_string(msg->gyro_rad[1]) + ","+ 
        //                      std::to_string(msg->gyro_rad[2]) + ","+ 
        //                      std::to_string(msg->accelerometer_m_s2[0]) + ","+ 
        //                      std::to_string(msg->accelerometer_m_s2[1]) + ","+
        //                      std::to_string(msg->accelerometer_m_s2[2]);
        //std::cout<<command;
        redisReply *reply = static_cast<redisReply*>(redisCommand(context_, command.c_str()));
        if (reply == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute Redis command: %s\\n", context_->errstr);
            redisFree(context_);
            //return;
        } else if (reply->type == REDIS_REPLY_ERROR) {
            RCLCPP_ERROR(this->get_logger(), "Redis command execution error: %s\\n", reply->str);
            freeReplyObject(reply);
            redisFree(context_);
            //return;
        }
        freeReplyObject(reply);
        //std::cout << "12345678";
        //RCLCPP_INFO(this->get_logger(), "RedisWrite节点启动");
     }

    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) const {
        std::string command = "ZADD vehicle_local_position(pos_n) " + std::to_string(msg->timestamp) + " " + std::to_string(msg->x)+ ","+ std::to_string(msg->y)+ ","+ std::to_string(msg->z)+ ","+std::to_string(msg->timestamp);
        redisReply *reply = static_cast<redisReply*>(redisCommand(context_, command.c_str()));
        freeReplyObject(reply);
        command = "ZADD vehicle_local_position(vel_n) " + std::to_string(msg->timestamp) + " " + std::to_string(msg->vx)+ ","+ std::to_string(msg->vy)+ ","+ std::to_string(msg->vz)+ ","+std::to_string(msg->timestamp);
        reply = static_cast<redisReply*>(redisCommand(context_, command.c_str()));
        freeReplyObject(reply);
    }

    void vehicle_angular_velocity_callback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg) const {
        std::string command = "ZADD vehicle_angular_velocity " + std::to_string(msg->timestamp) + " " + std::to_string(msg->xyz[0])+ ","+ std::to_string(msg->xyz[1])+ ","+ std::to_string(msg->xyz[2])+ ","+std::to_string(msg->timestamp);
        redisReply *reply = static_cast<redisReply*>(redisCommand(context_, command.c_str()));
        freeReplyObject(reply);
    }

    void actuator_motors_callback(const px4_msgs::msg::ActuatorMotors::SharedPtr msg) const {
        std::string command = "ZADD actuator_motors " + std::to_string(msg->timestamp) + " " + std::to_string(msg->control[0])+ ","+ std::to_string(msg->control[1])+ ","+ std::to_string(msg->control[2])+ ","+ std::to_string(msg->control[3])+ ","+ std::to_string(msg->control[4])+ ","+ std::to_string(msg->control[5])+ ","+ std::to_string(msg->timestamp);
        redisReply *reply = static_cast<redisReply*>(redisCommand(context_, command.c_str()));
        freeReplyObject(reply);
    }

    void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg){
        redisReply *reply;
        std::string command;
        command = "HSET Vehicle_attitude w " + std::to_string(msg->q[0]);
        reply = static_cast<redisReply*>(redisCommand(context_, command.c_str()));
        freeReplyObject(reply);
        command = "HSET Vehicle_attitude x " + std::to_string(msg->q[1]);
        reply = static_cast<redisReply*>(redisCommand(context_, command.c_str()));
        freeReplyObject(reply);
        command = "HSET Vehicle_attitude y " + std::to_string(msg->q[2]);
        reply = static_cast<redisReply*>(redisCommand(context_, command.c_str()));
        freeReplyObject(reply);
        command = "HSET Vehicle_attitude z " + std::to_string(msg->q[3]);
        reply = static_cast<redisReply*>(redisCommand(context_, command.c_str()));
        freeReplyObject(reply);
        command = "ZADD vehicle_attitude " + std::to_string(msg->timestamp) + " " + std::to_string(msg->q[0]) + "," + std::to_string(msg->q[1]) + "," + std::to_string(msg->q[2]) + "," + std::to_string(msg->q[3])+ ","+std::to_string(msg->timestamp);
        reply = static_cast<redisReply*>(redisCommand(context_, command.c_str()));
        freeReplyObject(reply);
    }

    void battery_status_callback(const px4_msgs::msg::BatteryStatus::SharedPtr msg){
        std::string command = "ZADD battery_status_V " + std::to_string(msg->timestamp) + " " + std::to_string(msg->voltage_filtered_v)+ ","+std::to_string(msg->timestamp);
        redisReply *reply = static_cast<redisReply*>(redisCommand(context_, command.c_str()));
        freeReplyObject(reply);
    }

     //3.声明订阅者和发布者
    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr SensorCombined_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr VehicleAttitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr VehicleLocalPosition_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr VehicleAngularVelocity_sub_;
    rclcpp::Subscription<px4_msgs::msg::ActuatorMotors>::SharedPtr ActuatorMotors_sub_;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr BatteryStatus_sub_;
    //模板类，传入订阅参数的参数类型，使用共享指针
    redisContext* context_;
    //void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) const
    //{
    
    //}
public:
    RedisWrite():Node("redis_write")
    {
//dds质量管理设定
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

//3.5.创建订阅者和发布者
        SensorCombined_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
            "/fmu/out/sensor_combined",//订阅的消息名称
            qos,//qs和队列长度
            std::bind(&RedisWrite::sensor_combine_callback,this,_1)//传入回调函数，使用bind函数将成员函数转换为回调函数（回调函数名称，回调函数所属的对象，出入参数占位符）
        );
        VehicleAttitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude",
            qos,
            std::bind(&RedisWrite::vehicle_attitude_callback,this,_1)
        );
        VehicleLocalPosition_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position",
            qos,
            std::bind(&RedisWrite::vehicle_local_position_callback,this,_1)
        );
        VehicleAngularVelocity_sub_ = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
            "/fmu/out/vehicle_angular_velocity",
            qos,
            std::bind(&RedisWrite::vehicle_angular_velocity_callback,this,_1)
        );
        ActuatorMotors_sub_ = this->create_subscription<px4_msgs::msg::ActuatorMotors>(
            "/fmu/out/actuator_motors",
            qos,
            std::bind(&RedisWrite::actuator_motors_callback,this,_1)
        );
        BatteryStatus_sub_ = this->create_subscription<px4_msgs::msg::BatteryStatus>(
            "/fmu/out/battery_status",
            qos,
            std::bind(&RedisWrite::battery_status_callback,this,_1)
        );
        
        context_ = redisConnect("127.0.0.1", 6379);
        if (context_ != nullptr && context_->err) {
            RCLCPP_ERROR(this->get_logger(), "Error connecting to Redis: %s\\n", context_->errstr);
            redisFree(context_);
            return;
        }
        ////创建一个订阅者
        //subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
        //    "/fmu/out/sensor_combined", qos,
        //    std::bind(&RedisWrite::vehicle_attitude_callback, this, _1));
//
        //context_ = redisConnect("127.0.0.1", 6379);
        //if (context_ != nullptr && context_->err) {
        //    RCLCPP_ERROR(this->get_logger(), "Error connecting to Redis: %s\\n", context_->errstr);
         //   redisFree(context_);
        //   return;
        //}
    }
    

    ~RedisWrite() override
    {
        if (context_ != nullptr) {
            redisFree(context_);
        }
    }

//private:
//    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr subscription_;
//    redisContext *context_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);//初始化节点
    auto node = std::make_shared<RedisWrite>();
    //通过make_share创建一个指定类型的对象并返回指向该对象的智能指针
    //对象为rclcpp::Node类型
    //在堆上分配内存来存储对象，并返回一个指向该对象的共享指针，该指针会自动跟踪对象的引用计数。
    // 传递的字符串是节点名称，节点的名称是唯一的，它用于区分不同的节点
    RCLCPP_INFO(node->get_logger(), "RedisWrite节点启动");//打印输出
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

//RCLCPP_COMPONENTS_REGISTER_NODE(RedisWrite)