#include "coordination_manager.h"
#include "system_compiler.h"
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

CoordinationManager::CoordinationManager() {
    fs::path sourceDir = fs::path(__FILE__).parent_path();
    coordinationManagerDir = sourceDir.string();
    envDir = (sourceDir / "env").string();
}

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