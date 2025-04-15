#ifndef SYSTEM_COMPILER_H
#define SYSTEM_COMPILER_H

#include <string>
#include <vector>

class SystemCompiler {
private:
    std::string haltonFilePath;
    std::vector<std::vector<double>> haltonPoints;

    void parseHaltonPoints();

public:
    SystemCompiler(const std::string& filePath);
    void compileSystem();
};

#endif // SYSTEM_COMPILER_H