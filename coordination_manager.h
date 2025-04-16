#ifndef COORDINATION_MANAGER_H
#define COORDINATION_MANAGER_H

#include "system_compiler.h"
#include "agent_data.h"
#include <string>

class CoordinationManager {
private:
    std::string coordinationManagerDir;
    std::string envDir;

    SystemCompiler systemCompiler;
    AgentData agentData;

public:
    CoordinationManager();
    std::string getRepositoryDirectory() const;
    std::string getEnvironmentDirectory() const;
    void start(const std::string& haltonFile);

    void setLookahead(int lookahead);
    void getLookahead(int& lookahead) const;

    void addAgent(int agentID, const AgentState& state);
    void updateAgent(int agentID, const AgentState& state);
    void removeAgent(int agentID);
    const AgentState* getAgent(int agentID) const;
    bool hasAgent(int agentID) const;
    std::list<int> getAllAgentIDs() const;
};

#endif // COORDINATION_MANAGER_H