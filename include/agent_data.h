#ifndef AGENT_DATA_H
#define AGENT_DATA_H

#include "agent_state.h" // Include the AgentState definition
#include <unordered_map>
#include <list>

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