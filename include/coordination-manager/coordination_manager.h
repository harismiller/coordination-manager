#ifndef COORDINATION_MANAGER_H
#define COORDINATION_MANAGER_H

#include "coordination-manager/system_compiler.h"
#include "coordination-manager/agent_data.h"
#include <string>

class CoordinationManager {
private:
    std::string coordinationManagerDir;
    std::string envDir;

    SystemCompiler systemCompiler;
    AgentData agentData;

public:
    CoordinationManager();
    ~CoordinationManager();
    
    std::string getRepositoryDirectory() const;
    std::string getEnvironmentDirectory() const;
    void start(const std::string& haltonFile);

    // System Compiler management functions
    void setLookahead(int lookahead);
    int getLookahead() const;
    void setStandbyLimit(int generalLimit);
    void setStandbyLimit(const std::unordered_map<std::pair<int, int>, int, pair_hash>& individualLimits);
    int getStandbyLimit() const;
    std::unordered_map<std::pair<int, int>, int, pair_hash> getStandbyLimits() const;
    bool checkGeneralLimitActive() const;

    // Agent management functions
    void addAgent(int agentID, const AgentState& state);
    void updateAgent(int agentID, const AgentState& state);
    void removeAgent(int agentID);
    const AgentState* getAgent(int agentID) const;
    bool hasAgent(int agentID) const;
    std::list<int> getAllAgentIDs() const;
};

#endif // COORDINATION_MANAGER_H