#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include "ltl_automaton_msgs/msg/relay_response.hpp"
#include "ltl_automaton_msgs/msg/ltl_plan.hpp"
#include "interfaces_hmm_sim/msg/path_plan.hpp"

#include <sqlite3.h>

class PlannerParserNode : public rclcpp::Node
{
public:
    PlannerParserNode()
        : Node("planner_parser_node")
    {
        // Declare and get the 'db_directory' parameter
        this->declare_parameter<std::string>("db_directory", "./db");
        db_directory_ = this->get_parameter("db_directory").as_string();

        RCLCPP_INFO(this->get_logger(), "Using database directory: %s", db_directory_.c_str());

        // Initialize the SQLite database
        initializeDatabase();

        // Print the lookahead value from the database
        printLookaheadValue();

        // Subscribe to the three topics within the namespace
        replanning_response_sub_ = this->create_subscription<ltl_automaton_msgs::msg::RelayResponse>(
            "replanning_response", 10,
            std::bind(&PlannerParserNode::replanningResponseCallback, this, std::placeholders::_1));

        prefix_plan_sub_ = this->create_subscription<ltl_automaton_msgs::msg::LTLPlan>(
            "prefix_plan", 10,
            std::bind(&PlannerParserNode::prefixPlanCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "PlannerParserNode started in namespace: %s", this->get_namespace());
    }

    ~PlannerParserNode()
    {
        if (db_)
        {
            sqlite3_close(db_);
            RCLCPP_INFO(this->get_logger(), "Database connection closed.");
        }
    }

private:
    std::string db_directory_; // Store the database directory path
    sqlite3 *db_ = nullptr;    // SQLite database handle

    void initializeDatabase()
    {
        std::string db_path = db_directory_ + "/system_compiler.db";
        int rc = sqlite3_open(db_path.c_str(), &db_);
        if (rc)
        {
            RCLCPP_ERROR(this->get_logger(), "Can't open database: %s", sqlite3_errmsg(db_));
            db_ = nullptr;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Database opened successfully: %s", db_path.c_str());
        }
    }

    void printLookaheadValue()
    {
        if (!db_)
        {
            RCLCPP_ERROR(this->get_logger(), "Database is not initialized.");
            return;
        }

        const char *sql = "SELECT value FROM compiled_data WHERE key = 'lookahead';";
        sqlite3_stmt *stmt;

        int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
        if (rc != SQLITE_OK)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to prepare statement: %s", sqlite3_errmsg(db_));
            return;
        }

        rc = sqlite3_step(stmt);
        if (rc == SQLITE_ROW)
        {
            int lookahead = sqlite3_column_int(stmt, 0);
            RCLCPP_INFO(this->get_logger(), "Lookahead value from database: %d", lookahead);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No lookahead value found in the database.");
        }

        sqlite3_finalize(stmt);
    }

    // Callback for replanning_response topic
    void replanningResponseCallback(const ltl_automaton_msgs::msg::RelayResponse::SharedPtr msg)
    {
        if (msg->success)
        {
            RCLCPP_INFO(this->get_logger(), "Replanning response received with success. Processing new plan prefix...");
            const auto &new_plan_prefix = msg->new_plan_prefix;

            // Log the action sequence from the new plan prefix
            RCLCPP_INFO(this->get_logger(), "New Plan Prefix Actions:");
            for (const auto &action : new_plan_prefix.action_sequence)
            {
                RCLCPP_INFO(this->get_logger(), "  - %s", action.c_str());
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Replanning response received but marked as unsuccessful.");
        }
    }

    // Callback for prefix_plan topic
    void prefixPlanCallback(const ltl_automaton_msgs::msg::LTLPlan::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Prefix plan received. Processing...");

        // Log the action sequence from the prefix plan
        RCLCPP_INFO(this->get_logger(), "Prefix Plan Actions:");
        for (const auto &action : msg->action_sequence)
        {
            RCLCPP_INFO(this->get_logger(), "  - %s", action.c_str());
        }

        // Log the current task
        RCLCPP_INFO(this->get_logger(), "Current Task: %d", msg->cur_task);
    }

    // Subscribers
    rclcpp::Subscription<ltl_automaton_msgs::msg::RelayResponse>::SharedPtr replanning_response_sub_;
    rclcpp::Subscription<ltl_automaton_msgs::msg::LTLPlan>::SharedPtr prefix_plan_sub_;
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