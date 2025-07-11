#include <rclcpp/rclcpp.hpp>
#include "coordination-manager/system_compiler.h"
#include "coordination-manager/agent_state.h"
#include <sqlite3.h>
#include <string>
#include <iostream>
#include <filesystem>
#include <unordered_set>
#include <vector>

class SystemCompilerNode : public rclcpp::Node
{
public:
    SystemCompilerNode()
        : Node("system_compiler_node"),
          systemCompiler()
    {
        std::string dbDir = systemCompiler.getDatabaseDirectory();
        db_path_ = dbDir + "/system_compiler.db";

        // Initialize the database
        initializeDatabase();

        systemCompiler.start("halton_points.csv");
        systemCompiler.setLookahead(3);
        
        // try {
        //     // Check if the index is within bounds
        //     auto point = systemCompiler.getPointByIndex(0);
        //     RCLCPP_INFO(this->get_logger(), "Point at index 0: (%f, %f)", point.first, point.second);
        // } catch (const std::out_of_range& e) {
        //     RCLCPP_ERROR(this->get_logger(), "Error accessing Halton point: %s", e.what());
        // }
    

        this->lookahead_ = systemCompiler.getLookahead();

        // Populate the Halton points table
        populateHaltonPointsTable();

        int num_agents = this->declare_parameter<int>("num_agents", 1);

        path_plan_sub_ = this->create_subscription<ltl_automaton_msgs::msg::PathPlan>(
            "parsed_path_plan", 10,
            std::bind(&SystemCompilerNode::pathPlanCallback, this, std::placeholders::_1));

        // Timer to simulate data compilation and database updates
        // timer_ = this->create_wall_timer(
        //     std::chrono::seconds(1),
        //     std::bind(&SystemCompilerNode::updateDatabase, this));

        for (int i = 1; i <= num_agents; ++i) {
            std::string topic_name = "/robot" + std::to_string(i) + "/parsed_path_plan";
            auto subscription = this->create_subscription<interfaces_hmm_sim::msg::PathPlan>(
                topic_name, 10,
                [this, i](const interfaces_hmm_sim::msg::PathPlan::SharedPtr msg) {
                    this->pathPlanCallback(i, msg);
                });
            path_plan_subscriptions_.push_back(subscription);
        }

        RCLCPP_INFO(this->get_logger(), "SystemCompilerNode initialized with %d agents.", num_agents);
    }

    ~SystemCompilerNode()
    {
        sqlite3_close(db_);
    }

private:
    std::string db_path_;
    int lookahead_;
    sqlite3 *db_;
    rclcpp::TimerBase::SharedPtr timer_;
    SystemCompiler systemCompiler;
    std::unordered_set<int> known_agents_;

    void initializeDatabase()
    {
        // Open the SQLite database (creates the file if it doesn't exist)
        // int rc = sqlite3_open("system_compiler.db", &db_);
        int rc = sqlite3_open(db_path_.c_str(), &db_);
        if (rc) 
        {
            RCLCPP_ERROR(this->get_logger(), "Can't open database: %s", sqlite3_errmsg(db_));
            return;
        }

        const char *pragma_sql = "PRAGMA journal_mode=WAL;";
        char *errMsg = nullptr;
        rc = sqlite3_exec(db_, pragma_sql, nullptr, nullptr, &errMsg);
        if (rc != SQLITE_OK)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set WAL mode: %s", errMsg);
            sqlite3_free(errMsg);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "WAL mode enabled for SQLite.");
        }

        // Create a table for storing compiled data
        //   DROP TABLE IF EXISTS compiled_data;
        const char *sql = R"(
            CREATE TABLE IF NOT EXISTS compiled_data (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                key TEXT NOT NULL UNIQUE,
                value INTEGER NOT NULL
            );

            CREATE TABLE IF NOT EXISTS halton_points (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                x REAL NOT NULL,
                y REAL NOT NULL
            );
        )";

        rc = sqlite3_exec(db_, sql, nullptr, nullptr, &errMsg);
        if (rc != SQLITE_OK)
        {
            RCLCPP_ERROR(this->get_logger(), "SQL error: %s", errMsg);
            sqlite3_free(errMsg);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Database initialized successfully.");
        }
    }

    void updateDatabase()
    {
        // Create the SQL query dynamically
        std::string sql = "INSERT INTO compiled_data (key, value) "
                          "VALUES ('lookahead', " + std::to_string(lookahead_) + ") "
                          "ON CONFLICT(key) DO UPDATE SET value = " + std::to_string(lookahead_) + ";";

        // Execute the SQL query
        char *errMsg = nullptr;
        int rc = sqlite3_exec(db_, sql.c_str(), nullptr, nullptr, &errMsg);
        if (rc != SQLITE_OK)
        {
            RCLCPP_ERROR(this->get_logger(), "SQL error: %s", errMsg);
            sqlite3_free(errMsg);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Database updated successfully with lookahead: %d", lookahead_);
        }
    }

    void clearDatabase()
    {
        const char *sql = "DELETE FROM compiled_data;";
        // const char *sql = "DROP TABLE IF EXISTS compiled_data;";
        char *errMsg = nullptr;
        int rc = sqlite3_exec(db_, sql, nullptr, nullptr, &errMsg);
        if (rc != SQLITE_OK)
        {
            RCLCPP_ERROR(this->get_logger(), "SQL error: %s", errMsg);
            sqlite3_free(errMsg);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Database cleared successfully.");
        }
    }

    void populateHaltonPointsTable()
    {
        // Clear the table before inserting new data
        const char *clearTableSQL = "DELETE FROM halton_points;";
        char *errMsg = nullptr;
        int rc = sqlite3_exec(db_, clearTableSQL, nullptr, nullptr, &errMsg);
        if (rc != SQLITE_OK)
        {
            RCLCPP_ERROR(this->get_logger(), "SQL error while clearing Halton points table: %s", errMsg);
            sqlite3_free(errMsg);
            return;
        }

        // Reset the auto-increment counter for the halton_points table
        const char *resetIdSQL = "DELETE FROM sqlite_sequence WHERE name = 'halton_points';";
        rc = sqlite3_exec(db_, resetIdSQL, nullptr, nullptr, &errMsg);
        if (rc != SQLITE_OK)
        {
            RCLCPP_ERROR(this->get_logger(), "SQL error while resetting Halton points table ID counter: %s", errMsg);
            sqlite3_free(errMsg);
            return;
        }

        // Insert each Halton point into the table
        for (const auto &point : systemCompiler.getHaltonPoints())
        {
            std::string sql = "INSERT INTO halton_points (x, y) VALUES (" +
                              std::to_string(point.first) + ", " +
                              std::to_string(point.second) + ");";

            rc = sqlite3_exec(db_, sql.c_str(), nullptr, nullptr, &errMsg);
            if (rc != SQLITE_OK)
            {
                RCLCPP_ERROR(this->get_logger(), "SQL error while inserting Halton point: %s", errMsg);
                sqlite3_free(errMsg);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Halton points table populated successfully.");
    }

    void pathPlanCallback(int agent_id, const interfaces_hmm_sim::msg::PathPlan::SharedPtr msg)
    {
        AgentState state;

        // Convert waypoints from geometry_msgs::msg::Point to std::pair<double, double>
        for (const auto &point : msg->waypoints)
        {
            state.waypoints.emplace_back(point.x, point.y);
        }

        // Convert actions from unsigned char to char
        state.actions = std::vector<char>(msg->actions.begin(), msg->actions.end());

        // Copy index and cur_task
        state.index = msg->index;
        state.curTask = msg->cur_task;

        // Populate additional fields (example logic, adjust as needed)
        if (!state.waypoints.empty())
        {
            state.currentPosition = state.waypoints.front(); // Assume the first waypoint is the current position
        }
        state.planIndex = 0;       // Example: Initialize planIndex to 0
        state.agentStatus = true;  // Example: Assume agent is active
        state.taskStatus = true;   // Example: Assume task is not yet completed

        // Check if the agent is already known
        if (known_agents_.find(agent_id) == known_agents_.end())
        {
            // First time receiving a message for this agent
            systemCompiler.addAgent(agent_id, state);
            known_agents_.insert(agent_id);

            RCLCPP_INFO(this->get_logger(), "Added agent %d.", agent_id);
        }
        else
        {
            // Update the existing agent
            systemCompiler.updateAgent(agent_id, state);

            RCLCPP_INFO(this->get_logger(), "Updated agent %d.", agent_id);
        }
    }

    // rclcpp::Subscription<ltl_automaton_msgs::msg::PathPlan>::SharedPtr path_plan_sub_;
    std::vector<rclcpp::Subscription<interfaces_hmm_sim::msg::PathPlan>::SharedPtr> path_plan_subscriptions_; // Store subscriptions
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SystemCompilerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}