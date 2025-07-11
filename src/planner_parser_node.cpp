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

        path_plan_pub_ = this->create_publisher<interfaces_hmm_sim::msg::PathPlan>(
            "parsed_path_plan", 10);

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

    std::vector<std::pair<double, double>> waypoints;
    std::vector<unsigned char> actions;
    std::vector<int> index;

    int cur_task;

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

    void parseActionSequence(const std::vector<std::string> &action_sequence)
    {
        waypoints.clear();
        actions.clear();
        index.clear();

        sqlite3_stmt *stmt = nullptr;
        const char *sql = "SELECT x, y FROM halton_points WHERE id = ?;";

        // Prepare the SQL statement once
        if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to prepare statement: %s", sqlite3_errmsg(db_));
            return;
        }

        for (const auto &action : action_sequence)
        {
            if (action.rfind("from_", 0) == 0 && action.find("_to_") != std::string::npos)
            {
                // Parse "from_n_to_m"
                size_t from_pos = action.find("from_") + 5;
                size_t to_pos = action.find("_to_");
                int n = std::stoi(action.substr(from_pos, to_pos - from_pos));
                int m = std::stoi(action.substr(to_pos + 4));

                if (index.empty() || index.back() != n)
                {
                    // First action or new "from_n_to_m" format
                    index.push_back(n);
                    index.push_back(m);

                    // Query the database for n
                    sqlite3_bind_int(stmt, 1, n);
                    if (sqlite3_step(stmt) == SQLITE_ROW)
                    {
                        double x = sqlite3_column_double(stmt, 0);
                        double y = sqlite3_column_double(stmt, 1);
                        waypoints.emplace_back(x, y);
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "No waypoint found for id: %d", n);
                    }
                    sqlite3_reset(stmt);

                    // Query the database for m
                    sqlite3_bind_int(stmt, 1, m);
                    if (sqlite3_step(stmt) == SQLITE_ROW)
                    {
                        double x = sqlite3_column_double(stmt, 0);
                        double y = sqlite3_column_double(stmt, 1);
                        waypoints.emplace_back(x, y);
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "No waypoint found for id: %d", m);
                    }
                    sqlite3_reset(stmt);

                    actions.push_back('g');
                    actions.push_back('g');
                }
                else
                {
                    // Subsequent "from_n_to_m" format
                    index.push_back(m);

                    // Query the database for m
                    sqlite3_bind_int(stmt, 1, m);
                    if (sqlite3_step(stmt) == SQLITE_ROW)
                    {
                        double x = sqlite3_column_double(stmt, 0);
                        double y = sqlite3_column_double(stmt, 1);
                        waypoints.emplace_back(x, y);
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "No waypoint found for id: %d", m);
                    }
                    sqlite3_reset(stmt);

                    actions.push_back('g');
                }
            }
            else
            {
                // Handle other actions
                if (!index.empty() && !waypoints.empty())
                {
                    index.push_back(index.back());
                    waypoints.push_back(waypoints.back());
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "No previous index or waypoint to duplicate for action: %s", action.c_str());
                    continue;
                }
                actions.push_back(action[0]); // First letter of the action
            }
        }

        // Finalize the prepared statement
        sqlite3_finalize(stmt);
    }

    void logParsedData()
    {
        RCLCPP_INFO(this->get_logger(), "Parsed Waypoints:");
        for (const auto &wp : waypoints)
        {
            RCLCPP_INFO(this->get_logger(), "  - (%f, %f)", wp.first, wp.second);
        }

        RCLCPP_INFO(this->get_logger(), "Parsed Actions:");
        for (const auto &action : actions)
        {
            RCLCPP_INFO(this->get_logger(), "  - %c", action);
        }

        RCLCPP_INFO(this->get_logger(), "Parsed Indices:");
        for (const auto &idx : index)
        {
            RCLCPP_INFO(this->get_logger(), "  - %d", idx);
        }
    }

    void publishPathPlan()
    {
        interfaces_hmm_sim::msg::PathPlan path_plan_msg;
        for (const auto &wp : waypoints)
        {
            geometry_msgs::msg::Point point;
            point.x = wp.first;
            point.y = wp.second;
            point.z = 0.0; // Assuming z is always 0 for 2D points
            path_plan_msg.waypoints.push_back(point);
        }
        path_plan_msg.actions = actions;
        path_plan_msg.index = index;

        path_plan_msg.cur_task = cur_task;

        path_plan_pub_->publish(path_plan_msg);
        RCLCPP_INFO(this->get_logger(), "Published Path Plan.");
    }

    // Callback for replanning_response topic
    void replanningResponseCallback(const ltl_automaton_msgs::msg::RelayResponse::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Replanning response received with success. Processing new plan prefix...");
        parseActionSequence(msg->new_plan_prefix.action_sequence);

        cur_task = msg->new_plan_prefix.cur_task;

        // Log the parsed data
        // logParsedData();

        // Publish the path plan
        publishPathPlan();
    }

    // Callback for prefix_plan topic
    void prefixPlanCallback(const ltl_automaton_msgs::msg::LTLPlan::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Prefix plan received. Processing...");
        parseActionSequence(msg->action_sequence);

        cur_task = msg->cur_task;

        // Log the parsed data
        // logParsedData();
        
        // Publish the path plan
        publishPathPlan();
    }

    // Subscribers
    rclcpp::Subscription<ltl_automaton_msgs::msg::RelayResponse>::SharedPtr replanning_response_sub_;
    rclcpp::Subscription<ltl_automaton_msgs::msg::LTLPlan>::SharedPtr prefix_plan_sub_;
    rclcpp::Publisher<interfaces_hmm_sim::msg::PathPlan>::SharedPtr path_plan_pub_;
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