#include "coordination_manager.h"
#include "system_compiler.h"
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

CoordinationManager::CoordinationManager() {
    coordinationManagerDir = fs::current_path().string();
    envDir = coordinationManagerDir + "/env";
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

    std::string haltonFilePath = envDir + "/" + haltonFile;

    SystemCompiler compiler(haltonFilePath);
    compiler.compileSystem();
}