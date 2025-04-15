#include "system_compiler.h"
#include <iostream>
#include <fstream>
#include <sstream>

SystemCompiler::SystemCompiler(const std::string& filePath) : haltonFilePath(filePath) {}

void SystemCompiler::parseHaltonPoints() {
    std::ifstream file(haltonFilePath);
    if (!file.is_open()) {
        std::cerr << "Error loading Halton points: Unable to open file " << haltonFilePath << std::endl;
        return;
    }

    haltonPoints.clear();
    std::string line;
    while (std::getline(file, line)) {
        std::vector<double> row;
        std::stringstream ss(line);
        std::string value;
        while (std::getline(ss, value, ',')) {
            try {
                row.push_back(std::stod(value));
            } catch (const std::exception& e) {
                std::cerr << "Error parsing value: " << value << " (" << e.what() << ")" << std::endl;
            }
        }
        haltonPoints.push_back(row);
    }

    std::cout << "Halton points loaded from " << haltonFilePath << std::endl;
}

void SystemCompiler::compileSystem() {
    if (haltonPoints.empty()) {
        parseHaltonPoints();
    }

    // Further processing can be added here
    std::cout << "Compiling system..." << std::endl;
}