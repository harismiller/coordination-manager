#include "coordination-manager/coordination_manager.h"
#include "coordination-manager/system_compiler.h"
#include "coordination-manager/agent_state.h"
#include "coordination-manager/agent_data.h"
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

CoordinationManager::CoordinationManager() 
    : systemCompiler(""), agentData() 
{
    fs::path sourceDir = fs::path(__FILE__).parent_path().parent_path();
    coordinationManagerDir = sourceDir.string();
    envDir = (sourceDir / "env").string();
}

CoordinationManager::~CoordinationManager() {}

std::string CoordinationManager::getRepositoryDirectory() const {
    return coordinationManagerDir;
}

std::string CoordinationManager::getEnvironmentDirectory() const {
    return envDir;
}

void CoordinationManager::start(const std::string& haltonFile) {
    std::cout << "Repository directory: " << coordinationManagerDir << std::endl;
    std::cout << "Environment directory: " << envDir << std::endl;

    std::cout << "Starting Coordination Manager..." << std::endl;

    fs::path haltonFilePath = fs::path(envDir) / haltonFile;

    SystemCompiler compiler(haltonFilePath.string());
    compiler.compileSystem();
}


// System Compiler management functions
void CoordinationManager::setLookahead(int lookahead) {
    systemCompiler.setLookahead(lookahead);
    std::cout << "Lookahead set to: " << systemCompiler.getLookahead() << std::endl;
}

int CoordinationManager::getLookahead() const {
    return systemCompiler.getLookahead();
}

void CoordinationManager::setStandbyLimit(int generalLimit) {
    systemCompiler.setStandbyLimit(generalLimit);
}

void CoordinationManager::setStandbyLimit(const std::unordered_map<std::pair<int, int>, int, pair_hash>& individualLimits) {
    systemCompiler.setStandbyLimit(individualLimits);
}

int CoordinationManager::getStandbyLimit() const {
    return systemCompiler.getStandbyLimit();
}

std::unordered_map<std::pair<int, int>, int, pair_hash> CoordinationManager::getStandbyLimits() const {
    return systemCompiler.getStandbyLimits();
}

bool CoordinationManager::checkGeneralLimitActive() const {
    return systemCompiler.checkGeneralLimitActive();
}

// Agent management functions
void CoordinationManager::addAgent(int agentID, const AgentState& state) {
    agentData.updateAgent(agentID, state);
}

void CoordinationManager::updateAgent(int agentID, const AgentState& state) {
    agentData.updateAgent(agentID, state);
}

void CoordinationManager::removeAgent(int agentID) {
    agentData.removeAgent(agentID);
}

const AgentState* CoordinationManager::getAgent(int agentID) const {
    return agentData.getAgent(agentID);
}

bool CoordinationManager::hasAgent(int agentID) const {
    return agentData.hasAgent(agentID);
}

std::list<int> CoordinationManager::getAllAgentIDs() const {
    return agentData.getAllAgentIDs();
}