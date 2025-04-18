#include "coordination-manager/agent_data.h"

void AgentData::updateAgent(int agentID, const AgentState& state) {
    agents[agentID] = state;
}

const AgentState* AgentData::getAgent(int agentID) const {
    auto it = agents.find(agentID);
    if (it != agents.end()) {
        return &it->second; // Return a pointer to the AgentState
    }
    return nullptr; // Return nullptr if the agent doesn't exist
}

void AgentData::removeAgent(int agentID) {
    agents.erase(agentID);
}

bool AgentData::hasAgent(int agentID) const {
    return agents.find(agentID) != agents.end();
}

std::list<int> AgentData::getAllAgentIDs() const {
    std::list<int> ids;
    for (const auto& entry : agents) {
        ids.push_back(entry.first);
    }
    return ids;
}