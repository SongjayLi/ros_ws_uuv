#include "mainwindow.h"
#include "ros_joy.h"
#include <QApplication>
#include <thread>
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
 
/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */
 
/*class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }
 
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};*/


int main(int argc, char *argv[])
{
    try {
        rclcpp::init(argc, argv);
        QApplication a(argc, argv);
        auto node = std::make_shared<joy_ctrl>("joy_ctrl");
        //MainWindow w(new joy_ctrl("play_joy"));
        //joy_ctrl j("play_joy");
        //connect(j,j.Update_Show,w,w.Update_num);
        //w.show();
        std::thread ros2_thread([&]() {
            if (rclcpp::ok()) {
                rclcpp::spin(node);
            }
        });//在创建线程时，通过一个 lambda 表达式[&]()作为线程函数的参数。这个 lambda 表达式定义了线程要执行的任务，即调用rclcpp::spin函数来处理 ROS2 消息。这里rclcpp::spin函数作用于一个std::make_shared<MinimalPublisher>()创建的共享指针所指向的对象，假设MinimalPublisher是一个 ROS2 节点类，它负责发布消息到 ROS2 网络中。
        //rclcpp::spin(std::make_shared<MinimalPublisher>());
        
        int result = a.exec();
        rclcpp::shutdown();
        if (ros2_thread.joinable()) {
            ros2_thread.join();
        }
        
        return result;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    } catch (...) {
        std::cerr << "Unknown exception occurred." << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    //return a.exec();
    //return result;
}
