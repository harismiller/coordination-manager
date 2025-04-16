#ifndef AGENT_DATA_H
#define AGENT_DATA_H

#include "agent_state.h"
#include <unordered_map>
#include <list>

class AgentData {
private:
    std::unordered_map<int, AgentState> agents; // Key: Agent ID, Value: AgentState

public:
    // Add or update an agent's state
    void updateAgent(int agentID, const AgentState& state);
    
    const AgentState* getAgent(int agentID) const;
    void removeAgent(int agentID);
    bool hasAgent(int agentID) const;
    std::list<int> getAllAgentIDs() const;
};

#endif // AGENT_DATA_H