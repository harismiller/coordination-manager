#ifndef AGENT_DATA_H
#define AGENT_DATA_H

#include <unordered_map>
#include <string>
#include <list>
#include <utility> // for std::pair

struct AgentState {
    std::pair<double, double> currentPosition; // Current position (x, y)
    std::string status;                        // Status (e.g., "idle", "moving", "completed")
    std::list<std::pair<double, double>> pathPlan; // List of (x, y) for the planned path
    std::list<char> flags;                     // List of flags (e.g., 'A', 'B', etc.)
    int planIndex;                             // Current index in the path plan
};

class AgentData {
private:
    std::unordered_map<int, AgentState> agents; // Key: Agent ID, Value: AgentState

public:
    // Add or update an agent's state
    void updateAgent(int agentID, const AgentState& state);

    // Get an agent's state
    const AgentState* getAgent(int agentID) const;

    // Remove an agent
    void removeAgent(int agentID);

    // Check if an agent exists
    bool hasAgent(int agentID) const;

    // Get all agent IDs
    std::list<int> getAllAgentIDs() const;
};

#endif // AGENT_DATA_H