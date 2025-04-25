#ifndef SYSTEM_COMPILER_H
#define SYSTEM_COMPILER_H

#include "coordination-manager/environment_compiler.h"
#include "coordination-manager/agent_data.h"
#include <string>

class SystemCompiler {
private:
    std::string systemCompilerDir;
    std::string envDir;
    std::string dbDir;

    EnvironmentCompiler environmentCompiler;
    AgentData agentData;

public:
    SystemCompiler();
    ~SystemCompiler();
    
    std::string getRepositoryDirectory() const;
    std::string getEnvironmentDirectory() const;
    std::string getDatabaseDirectory() const;
    void start(const std::string& haltonFile);

    // Environment Compiler management functions
    void setLookahead(int lookahead);
    int getLookahead() const;
    void setStandbyLimit(int generalLimit);
    void setStandbyLimit(const std::unordered_map<std::pair<int, int>, int, pair_hash>& individualLimits);
    int getStandbyLimit() const;
    std::unordered_map<std::pair<int, int>, int, pair_hash> getStandbyLimits() const;
    bool checkGeneralLimitActive() const;
    std::pair<double, double> getPointByIndex(size_t index) const;
    std::vector<std::pair<double, double>> getHaltonPoints() const;

    // Agent management functions
    void addAgent(int agentID, const AgentState& state);
    void updateAgent(int agentID, const AgentState& state);
    void removeAgent(int agentID);
    const AgentState* getAgent(int agentID) const;
    bool hasAgent(int agentID) const;
    std::list<int> getAllAgentIDs() const;
};

#endif // SYSTEM_COMPILER_H