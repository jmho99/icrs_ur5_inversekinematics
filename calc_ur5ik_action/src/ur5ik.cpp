#include <memory>
#include <thread>

#include "calc_interfaces/action/calc_theta.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

//namespace calc_ur5ik_action_cpp

class ur5ik : public rclcpp::Node
{
public:
    using Interface = calc_interfaces::action::CalcTheta;
    using GoalHandleInterface = rclcpp_action::ServerGoalHandle<Interface>;

    explicit ur5ik(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("ur5ik", options)
    {
        using namespace std::placeholders;

        this->m_action_server = rclcpp_action::create_server<Interface>(
            this,
            "Goal_pose",
            std::bind(&ur5ik::handle_goal, this, _1, _2),
            std::bind(&ur5ik::handle_cancel, this, _1),
            std::bind(&ur5ik::handle_accepted, this, _1)
        );
    }
private:
    rclcpp_action::Server<Interface>::SharedPtr m_action_server;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Interface::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal %f", goal->act_target[0]);
        RCLCPP_INFO(this->get_logger(), "Received goal %f", goal->act_target[1]);
        RCLCPP_INFO(this->get_logger(), "Received goal %f", goal->act_target[2]);
        RCLCPP_INFO(this->get_logger(), "Received goal %f", goal->act_target[3]);
        RCLCPP_INFO(this->get_logger(), "Received goal %f", goal->act_target[4]);
        RCLCPP_INFO(this->get_logger(), "Received goal %f", goal->act_target[5]);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleInterface> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleInterface> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&ur5ik::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleInterface> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Interface::Feedback>();
        auto result = std::make_shared<Interface::Result>();

        auto x = goal->act_target[0]; auto y = goal->act_target[1]; auto z = goal->act_target[2];
        auto roll = goal->act_target[3]; auto pitch = goal->act_target[4]; auto yaw = goal->act_target[5];
        set_trans(x,y,z,roll,pitch,yaw);
        cal_theta();

        result->act_theta[0] = m_theta[0];
        result->act_theta[1] = m_theta[1];
        result->act_theta[2] = m_theta[2];
        result->act_theta[3] = m_theta[3];
        result->act_theta[4] = m_theta[4];
        result->act_theta[5] = m_theta[5];

        RCLCPP_INFO(this->get_logger(), "Result %f", result->act_theta[0]);
        RCLCPP_INFO(this->get_logger(), "Result %f", result->act_theta[1]);
        RCLCPP_INFO(this->get_logger(), "Result %f", result->act_theta[2]);
        RCLCPP_INFO(this->get_logger(), "Result %f", result->act_theta[3]);
        RCLCPP_INFO(this->get_logger(), "Result %f", result->act_theta[4]);
        RCLCPP_INFO(this->get_logger(), "Result %f", result->act_theta[5]);

        // Publish feedback
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish feedback");

        // Check if goal is done and publish result
        if(rclcpp::ok())
        {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }


    double m_theta[6] = { 0 };
    double pi = 3.141592;
    //DHparameter (m)
    double m_d1 = 0.104159;
    double m_a2 = -0.42500;
    double m_a3 = -0.39225;
    double m_d4 = 0.10915;
    double m_d5 = 0.09465;
    double m_d6 = 0.2235;

    double T[4][4] = { 0 };

    double m_sepe_get_topic[6] = { 0 };

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
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ur5ik>());
	rclcpp::shutdown();
}
