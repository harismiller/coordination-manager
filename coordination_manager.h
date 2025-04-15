#ifndef COORDINATION_MANAGER_H
#define COORDINATION_MANAGER_H

#include <string>

class CoordinationManager {
private:
    std::string coordinationManagerDir;
    std::string envDir;

public:
    CoordinationManager();
    std::string getRepositoryDirectory() const;
    std::string getEnvironmentDirectory() const;
    void start(const std::string& haltonFile);
};

#endif // COORDINATION_MANAGER_H