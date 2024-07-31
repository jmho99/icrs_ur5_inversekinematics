#include <memory>
#include <functional>
#include <future>

#include "calc_interfaces/action/calc_theta.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

double pi = 3.141592;

class set_target : public rclcpp::Node
{
public:
    using Interface = calc_interfaces::action::CalcTheta;
    using GoalHandleInterface = rclcpp_action::ClientGoalHandle<Interface>;

    explicit set_target(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("set_target", options)
    {
        this->m_action_client = rclcpp_action::create_client<Interface>(
            this,
            "action_ik");

            this->m_timer = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&set_target::send_goal, this));
    }

    void send_goal()
    {
        using namespace std::placeholders;

        this->m_timer->cancel();

        if(!this->m_action_client->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after watiting");
            rclcpp::shutdown();
        }

        auto request = Interface::Goal();
        request.act_target[0] = 0.0;
        request.act_target[1] = 0.6;
        request.act_target[2] = 0.3;
        request.act_target[3] = pi;
        request.act_target[4] = 0.0;
        request.act_target[5] = 0.0;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<Interface>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&set_target::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&set_target::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&set_target::result_callback, this, _1);

        this->m_action_client->async_send_goal(request, send_goal_options);
    }

private:
    rclcpp_action::Client<Interface>::SharedPtr m_action_client;
    rclcpp::TimerBase::SharedPtr m_timer;

    void goal_response_callback(const GoalHandleInterface::SharedPtr & goal_handle)
    {
        if(!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by serve, waitiong for result");
        }
    }

    void feedback_callback(
        GoalHandleInterface::SharedPtr,
        const std::shared_ptr<const Interface::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(),"Feedback");
    }

    void result_callback(const GoalHandleInterface::WrappedResult & result)
    {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
        auto theta1 = result.result->act_theta[0];
        auto theta2 = result.result->act_theta[1];
        auto theta3 = result.result->act_theta[2];
        auto theta4 = result.result->act_theta[3];
        auto theta5 = result.result->act_theta[4];
        auto theta6 = result.result->act_theta[5];

        RCLCPP_INFO(this->get_logger(), "theta = { %f, %f, %f, %f, %f, %f}",
            theta1, theta2, theta3, theta4, theta5, theta6);

        rclcpp::shutdown();
    }
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<set_target>());
	rclcpp::shutdown();
}