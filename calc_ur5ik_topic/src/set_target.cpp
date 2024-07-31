#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <math.h>

using namespace std::chrono_literals;

double pi = 3.141592;

class set_target : public rclcpp::Node
{
public:
    set_target()
    : Node("set_target")
    {
        m_publisher = this -> create_publisher<std_msgs::msg::Float64MultiArray>(
            "topic_ik",10);
        m_timer = this -> create_wall_timer(
            3s, std::bind(&set_target::timer_callback, this));
    }
    private:
    double m_x = 0.0;
    double m_y = 0.6;
    double m_z = 0.3;
    double m_roll = pi;
    double m_pitch = 0.0;
    double m_yaw = 0.0;

    void timer_callback()
    {
        auto message = std_msgs::msg::Float64MultiArray();
        //message.data = "0.0 0.6 0.3 3.141592 0.0 0.0" ;
        message.data = {m_x, m_y, m_z, m_roll, m_pitch, m_yaw} ;

        RCLCPP_INFO(this->get_logger(), "x = %f", message.data[0]);
        RCLCPP_INFO(this->get_logger(), "y = %f", message.data[1]);
        RCLCPP_INFO(this->get_logger(), "z = %f", message.data[2]);
        RCLCPP_INFO(this->get_logger(), "roll = %f", message.data[3]);
        RCLCPP_INFO(this->get_logger(), "pitch = %f", message.data[4]);
        RCLCPP_INFO(this->get_logger(), "yaw = %f", message.data[5]);

        RCLCPP_INFO(this->get_logger(), "%s", "-----------------");
        //RCLCPP_INFO(this->get_logger(), "Set RT : %s", message.data.c_str());
        m_publisher -> publish(message);
    }

    rclcpp::TimerBase::SharedPtr m_timer;
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_publisher;
    size_t m_count;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<set_target>());
    rclcpp::shutdown();

    return 0;
}
