#include "coordination-manager/system_compiler.h"
#include "coordination-manager/environment_compiler.h"
#include "coordination-manager/agent_state.h"
#include "coordination-manager/agent_data.h"
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

SystemCompiler::SystemCompiler() 
    : environmentCompiler(), agentData() 
{
    fs::path sourceDir = fs::path(__FILE__).parent_path().parent_path();
    systemCompilerDir = sourceDir.string();
    dbDir = (sourceDir / "db").string();
    envDir = (sourceDir / "env").string();
}

SystemCompiler::~SystemCompiler() {}

std::string SystemCompiler::getRepositoryDirectory() const {
    return systemCompilerDir;
}

std::string SystemCompiler::getEnvironmentDirectory() const {
    return envDir;
}

std::string SystemCompiler::getDatabaseDirectory() const {
    return dbDir;
}

void SystemCompiler::start(const std::string& haltonFile) {
    std::cout << "Repository directory: " << systemCompilerDir << std::endl;
    std::cout << "Environment directory: " << envDir << std::endl;

    std::cout << "Starting System Compiler..." << std::endl;

    fs::path haltonFilePath = fs::path(envDir) / haltonFile;

    environmentCompiler.compileEnvironment(haltonFilePath.string());
}


// Environment Compiler management functions
void SystemCompiler::setLookahead(int lookahead) {
    environmentCompiler.setLookahead(lookahead);
    std::cout << "Lookahead set to: " << environmentCompiler.getLookahead() << std::endl;
}

int SystemCompiler::getLookahead() const {
    return environmentCompiler.getLookahead();
}

void SystemCompiler::setStandbyLimit(int generalLimit) {
    environmentCompiler.setStandbyLimit(generalLimit);
}

void SystemCompiler::setStandbyLimit(const std::unordered_map<std::pair<int, int>, int, pair_hash>& individualLimits) {
    environmentCompiler.setStandbyLimit(individualLimits);
}

int SystemCompiler::getStandbyLimit() const {
    return environmentCompiler.getStandbyLimit();
}

std::unordered_map<std::pair<int, int>, int, pair_hash> SystemCompiler::getStandbyLimits() const {
    return environmentCompiler.getStandbyLimits();
}

bool SystemCompiler::checkGeneralLimitActive() const {
    return environmentCompiler.checkGeneralLimitActive();
}

std::pair<double,double> SystemCompiler::getPointByIndex(size_t index) const {
    return environmentCompiler.getPointByIndex(index);
}

std::vector<std::pair<double,double>> SystemCompiler::getHaltonPoints() const {
    return environmentCompiler.getHaltonPoints();
}

// Agent management functions
void SystemCompiler::addAgent(int agentID, const AgentState& state) {
    agentData.updateAgent(agentID, state);
}

void SystemCompiler::updateAgent(int agentID, const AgentState& state) {
    agentData.updateAgent(agentID, state);
}

void SystemCompiler::removeAgent(int agentID) {
    agentData.removeAgent(agentID);
}

const AgentState* SystemCompiler::getAgent(int agentID) const {
    return agentData.getAgent(agentID);
}

bool SystemCompiler::hasAgent(int agentID) const {
    return agentData.hasAgent(agentID);
}

std::list<int> SystemCompiler::getAllAgentIDs() const {
    return agentData.getAllAgentIDs();
}

