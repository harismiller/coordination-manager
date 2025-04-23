#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "ltl_automaton_msgs/msg/relay_response.hpp"
#include "ltl_automaton_msgs/msg/ltl_plan.hpp"
#include "interfaces_hmm_sim/msg/path_plan.hpp"

class PlannerParserNode : public rclcpp::Node
{
public:
    PlannerParserNode()
        : Node("planner_parser_node")
    {
        // Subscribe to the three topics within the namespace
        replanning_response_sub_ = this->create_subscription<ltl_automaton_msgs::msg::RelayResponse>(
            "replanning_response", 10,
            std::bind(&PlannerParserNode::replanningResponseCallback, this, std::placeholders::_1));

        prefix_plan_sub_ = this->create_subscription<ltl_automaton_msgs::msg::LTLPlan>(
            "prefix_plan", 10,
            std::bind(&PlannerParserNode::prefixPlanCallback, this, std::placeholders::_1));

        suffix_plan_sub_ = this->create_subscription<ltl_automaton_msgs::msg::LTLPlan>(
            "suffix_plan", 10,
            std::bind(&PlannerParserNode::suffixPlanCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "PlannerParserNode started in namespace: %s", this->get_namespace());
    }

private:
    // Callback for replanning_response topic
    void replanningResponseCallback(const ltl_automaton_msgs::msg::RelayResponse::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received replanning response: success=%s",
                    msg->success ? "true" : "false");

        if (msg->success)
        {
            RCLCPP_INFO(this->get_logger(), "New plan prefix:");
            for (const auto &action : msg->new_plan_prefix.action_sequence)
            {
                RCLCPP_INFO(this->get_logger(), "  Action: %s", action.c_str());
            }

            RCLCPP_INFO(this->get_logger(), "New plan suffix:");
            for (const auto &action : msg->new_plan_suffix.action_sequence)
            {
                RCLCPP_INFO(this->get_logger(), "  Action: %s", action.c_str());
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Replanning failed.");
        }
    }

    // Callback for prefix_plan topic
    void prefixPlanCallback(const ltl_automaton_msgs::msg::LTLPlan::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received prefix plan:");
        for (const auto &action : msg->action_sequence)
        {
            RCLCPP_INFO(this->get_logger(), "  Action: %s", action.c_str());
        }
    }

    // Callback for suffix_plan topic
    void suffixPlanCallback(const ltl_automaton_msgs::msg::LTLPlan::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received suffix plan:");
        for (const auto &action : msg->action_sequence)
        {
            RCLCPP_INFO(this->get_logger(), "  Action: %s", action.c_str());
        }
    }

    // Subscribers
    rclcpp::Subscription<ltl_automaton_msgs::msg::RelayResponse>::SharedPtr replanning_response_sub_;
    rclcpp::Subscription<ltl_automaton_msgs::msg::LTLPlan>::SharedPtr prefix_plan_sub_;
    rclcpp::Subscription<ltl_automaton_msgs::msg::LTLPlan>::SharedPtr suffix_plan_sub_;
};

// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PlannerParserNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}