#include <stdio.h>
#include <iostream>
#include <math.h>
#include <sstream>

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"


using std::placeholders::_1;
double pi = 3.141592;

class calc_sub : public rclcpp::Node
{
public:
    calc_sub()
    : Node("calc_sub")
    {
        m_subscriber = this -> create_subscription<std_msgs::msg::Float64MultiArray>(
            "topic_ik", 10, std::bind(&calc_sub::calc_sub_callback, this, _1));
    }
#if 0
    calc_sub(float x, float y, float z, float roll, float pitch, float yaw)
    : calc_sub()
    {
        set_trans(x, y, z, roll, pitch, yaw);
        cal_theta();
        print_theta();
    }
#endif
private:

    double m_theta[6];

    //DHparameter (m)
    double m_d1 = 0.104159;
    double m_a2 = -0.42500;
    double m_a3 = -0.39225;
    double m_d4 = 0.10915;
    double m_d5 = 0.09465;
    double m_d6 = 0.2235;

    double T[4][4] = { 0 };

    double m_sepe_get_topic[6] = { 0 };

#if 1
    void calc_sub_callback(const std_msgs::msg::Float64MultiArray & get_topic)
    {
        for (int i = 0; i < 6; i++)
        {
            m_sepe_get_topic[i] = get_topic.data[i];
        }

        RCLCPP_INFO(this->get_logger(), "x = %f", m_sepe_get_topic[0]);
        RCLCPP_INFO(this->get_logger(), "y = %f", m_sepe_get_topic[1]);
        RCLCPP_INFO(this->get_logger(), "z = %f", m_sepe_get_topic[2]);
        RCLCPP_INFO(this->get_logger(), "roll = %f", m_sepe_get_topic[3]);
        RCLCPP_INFO(this->get_logger(), "pitch = %f", m_sepe_get_topic[4]);
        RCLCPP_INFO(this->get_logger(), "yaw = %f", m_sepe_get_topic[5]);
        RCLCPP_INFO(this->get_logger(), " %s", "-----------------");
        //RCLCPP_INFO(this->get_logger(), "T got: %s", get_topic.data.c_str());

        #if 0
        std::string str = get_topic.data.c_str();
        std::string seperator = " ";
        int cur_pos = 0;
        int position;
        int num = 0;

        while ((position = str.find(seperator, cur_pos)) != std::string::npos)
        {
            int len = position - cur_pos;
            std::string result = str.substr(cur_pos, len);
            std::stringstream ssDouble(result);
            ssDouble >> m_sepe_get_topic[num];
            std::cout << m_sepe_get_topic[num] << std::endl;
            cur_pos = position + 1;
            num = num + 1;
        }

        std::string result = str.substr(cur_pos);
        std::stringstream ssDouble(result);
        ssDouble >> m_sepe_get_topic[num];
        std::cout << m_sepe_get_topic[num] << std::endl;
        #endif
        set_trans(m_sepe_get_topic[0],m_sepe_get_topic[1],m_sepe_get_topic[2],m_sepe_get_topic[3],m_sepe_get_topic[4],m_sepe_get_topic[5]);
        cal_theta();
        print_theta();

    }
#endif

    void set_trans(double x, double y, double z, double roll, double pitch, double yaw)
    {
        double XYZ[3][1] = { {x},{y},{z} };

        double Rx[3][3] = { {1, 0, 0},
                            {0, cos(roll), - sin(roll)},
                            {0, sin(roll), cos(roll)} };

        double Ry[3][3] = { {cos(pitch), 0, sin(pitch)},
                            {0, 1, 0},
                            {-sin(pitch), 0, cos(pitch)} };

        double Rz[3][3] = { {cos(yaw), - sin(yaw), 0},
                            {sin(yaw), cos(yaw), 0},
                            {0, 0, 1} };

        double R[3][3] = { 0 };

        for (int row = 0; row < 3; row++)
        {
            for (int column = 0; column < 3; column++)
            {
                for (int k = 0; k < 3; k++)
                {
                    for (int l = 0; l < 3; l++)
                    {
                        R[row][column] += Rz[row][k] * Ry[k][l] * Rx[l][column];
                    }

                }
            }
        }

        for (int row = 0; row < 4; row++)
        {
            for (int column = 0; column < 4; column++)
            {
                if (row <= 2 && column <= 2)
                {
                    T[row][column] = R[row][column];
                }
                else if (row <= 2 && column == 3)
                {
                    T[row][column] = XYZ[row][0];
                }
                else if (row == 3 && column <= 2)
                {
                    T[row][column] = 0;
                }
                else
                {
                    T[row][column] = 1;
                }
            }
        }
#if 0
        for (int row = 0; row < 4; row++)
        {
            for (int column = 0; column < 4; column++)
            {
                std::cout << T[row][column] << "\t";
            }
            std::cout << "\n";
        }
#endif

    }

    void cal_theta()
    {
        double p05[4][1] = { 0 };
        double p65[4][1] = { {0},{0},{-m_d6},{1} };

        for (int row = 0; row < 4; row++)
        {
            for (int column = 0; column < 1; column++)
            {
                for (int k = 0; k < 4; k++)
                {
                    p05[row][column] += T[row][k] * p65[k][column];
                }
            }

        }

        m_theta[0] = atan2(p05[1][0], p05[0][0]) - acos(m_d4 / sqrt(pow(p05[0][0], 2) + pow(p05[1][0], 2))) + pi / 2;
        m_theta[4] = acos((T[0][3] * sin(m_theta[0]) - T[1][3] * cos(m_theta[0]) - m_d4) / m_d6);
        m_theta[5] = atan2(((-T[0][1] * sin(m_theta[0]) + T[1][1] * cos(m_theta[0])) / sin(m_theta[4])),
            ((T[0][0] * sin(m_theta[0]) - T[1][0] * cos(m_theta[0])) / sin(m_theta[4])));

        double theta234 = atan2(-T[2][2] / sin(m_theta[4]),
            (T[0][2] * cos(m_theta[0]) + T[1][2] * sin(m_theta[0])) / sin(m_theta[4]));
        double T24_b14 = T[0][3] * cos(m_theta[0]) + T[1][3] * sin(m_theta[0])
            - m_d6 * T[0][2] * cos(m_theta[0]) - m_d6 * T[1][2] * sin(m_theta[0])
            + m_d5 * T[1][0] * sin(m_theta[0]) * sin(m_theta[5]) + m_d5 * T[0][1] * cos(m_theta[0]) * cos(m_theta[5])
            + m_d5 * T[0][0] * cos(m_theta[0]) * sin(m_theta[5]) + m_d5 * T[1][1] * sin(m_theta[0]) * cos(m_theta[5]);
        double T24_b24 = T[2][3] - m_d1 - m_d6 * T[2][2] + m_d5 * T[2][1] * cos(m_theta[5]) + m_d5 * T[2][0] * sin(m_theta[5]);

        //theta3 = 2 * atan(sqrt(((a2 + a3) ^ 2 - (b14. ^ 2 + b24. ^ 2)) / ((b14. ^ 2 + b24. ^ 2) - (a2 - a3) ^ 2)))
        m_theta[2] = -acos((pow(T24_b14,2) + pow(T24_b24,2) - pow(m_a2,2) - pow(m_a3,2)) / (2 * m_a2 * m_a3));

        double alpha = atan2(T24_b24, T24_b14);
        double beta = atan2(m_a3 * sin(m_theta[2]), m_a2 + m_a3 * cos(m_theta[2]));
        m_theta[1] = alpha - beta;

        m_theta[3] = theta234 - m_theta[1] - m_theta[2];
    }

    void print_theta()
    {
        //std::cout << m_sepe_get_topic[0] <<"; " << m_sepe_get_topic[1] << "; " << m_sepe_get_topic[2] << "; " << m_sepe_get_topic[3] << "; " << m_sepe_get_topic[4] << "; " << m_sepe_get_topic[5] << "\n";
        //std::cout << m_theta[0] <<"; " << m_theta[1] << "; " << m_theta[2] << "; " << m_theta[3] << "; " << m_theta[4] << "; " << m_theta[5] << "\n";

        RCLCPP_INFO(this->get_logger(), "theta1 = %f", m_theta[0]);
        RCLCPP_INFO(this->get_logger(), "theta2 = %f", m_theta[1]);
        RCLCPP_INFO(this->get_logger(), "theta3 = %f", m_theta[2]);
        RCLCPP_INFO(this->get_logger(), "theta4 = %f", m_theta[3]);
        RCLCPP_INFO(this->get_logger(), "theta5 = %f", m_theta[4]);
        RCLCPP_INFO(this->get_logger(), "theta6 = %f", m_theta[5]);

        RCLCPP_INFO(this->get_logger(), " %s", "-----------------");
    }


   //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subscriber;
   rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_subscriber;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
#if 0
    std::string str = "0.0 0.6 0.3 3.141592 0.0 0.0";
    std::string seperator = " ";
    int cur_pos = 0;
    int position;
    double val[6] = { 0 };
    int num = 0;

    while ((position = str.find(seperator, cur_pos)) != std::string::npos)
    {
        int len = position - cur_pos;
        std::string result = str.substr(cur_pos, len);
        std::stringstream ssDouble(result);
        ssDouble >> val[num];
        std::cout << val[num] << std::endl;
        cur_pos = position + 1;
        num = num + 1;
    }

    std::string result = str.substr(cur_pos);
    std::stringstream ssDouble(result);
    ssDouble >> val[num];
    std::cout << val[num] << std::endl;

    for (int i = 0; i < 6; i++)
    {
        std::cout << val[i] << "\n";
    }

    calc_sub sol(0.0, 0.6, 0.3, pi, 0, 0);
    calc_sub sol2(val[0], val[1], val[2], val[3], val[4], val[5]);
#endif
    rclcpp::spin(std::make_shared<calc_sub>());

    rclcpp::shutdown();
    return 0;
}
