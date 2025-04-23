#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "interfaces_hmm_sim/msg/status.hpp"
#include "interfaces_hmm_sim/msg/agent_fail.hpp"

using namespace std::chrono_literals;

class StatusAgentFailPublisher : public rclcpp::Node
{
public:
    StatusAgentFailPublisher()
        : Node("status_agent_fail_publisher"), publishing_status_(true)
    {
        // Extract namespace and derive agent name and ID
        std::string namespace_ = this->get_namespace();
        agent_name_ = namespace_.substr(1);  // Remove leading '/'
        agent_id_ = std::stoi(agent_name_.substr(agent_name_.find_last_not_of("0123456789") + 1));

        // Initialize publishers
        status_publisher_ = this->create_publisher<interfaces_hmm_sim::msg::Status>("status", 10);
        agent_fail_publisher_ = this->create_publisher<interfaces_hmm_sim::msg::AgentFail>("agent_failure", 10);

        // Random duration between 10 and 20 seconds
        duration_to_publish_ = 10 + (std::rand() % 11);

        // Start the timer
        timer_ = this->create_wall_timer(1s, std::bind(&StatusAgentFailPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Publishing Status messages for %d seconds...", duration_to_publish_);
    }

private:
    void timer_callback()
    {
        if (publishing_status_)
        {
            // Publish Status messages
            auto status_msg = interfaces_hmm_sim::msg::Status();
            status_msg.agent = agent_name_;
            status_msg.start = false;
            status_msg.arrived = true;
            status_msg.replan_received = false;

            status_publisher_->publish(status_msg);
            RCLCPP_INFO(this->get_logger(), "Published Status message");

            // Check if the duration has elapsed
            elapsed_time_++;
            if (elapsed_time_ >= duration_to_publish_)
            {
                publishing_status_ = false;
                RCLCPP_INFO(this->get_logger(), "Switching to publishing AgentFail messages...");
            }
        }
        else
        {
            // Publish AgentFail message
            auto agent_fail_msg = interfaces_hmm_sim::msg::AgentFail();
            agent_fail_msg.agent_id = agent_id_;
            agent_fail_msg.pose_index = 60;
            agent_fail_msg.position = {10.46875,11.934156378600822};

            agent_fail_publisher_->publish(agent_fail_msg);
            RCLCPP_INFO(this->get_logger(), "Published AgentFail message");

            // Stop the timer after publishing AgentFail
            timer_->cancel();
        }
    }

    rclcpp::Publisher<interfaces_hmm_sim::msg::Status>::SharedPtr status_publisher_;
    rclcpp::Publisher<interfaces_hmm_sim::msg::AgentFail>::SharedPtr agent_fail_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string agent_name_;
    int agent_id_;
    bool publishing_status_;
    int duration_to_publish_;
    int elapsed_time_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatusAgentFailPublisher>());
    rclcpp::shutdown();
    return 0;
}

