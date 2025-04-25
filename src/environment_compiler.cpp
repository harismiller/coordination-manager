#include "coordination-manager/environment_compiler.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>

EnvironmentCompiler::EnvironmentCompiler()
    : totalPoints(0), gridDimensions({0, 0}), lookahead(0), isGeneralLimitActive(true) {
    standby.generalLimit = 0; // Initialize the union with generalLimit = 0
}

EnvironmentCompiler::~EnvironmentCompiler() {
    if (!isGeneralLimitActive) {
        standby.individualLimits.~unordered_map();
    }
}

void EnvironmentCompiler::parseHaltonPoints() {
    std::ifstream file(haltonFilePath);
    if (!file.is_open()) {
        std::cerr << "Error loading Halton points: Unable to open file " << haltonFilePath << std::endl;
        return;
    }

    gridTallies.clear();
    labeledPoints.clear();
    haltonPoints.clear(); // Clear the vector before populating
    totalPoints = 0;
    gridDimensions = {0, 0};

    std::string line;
    bool isHeader = true;

    while (std::getline(file, line)) {
        if (isHeader) {
            isHeader = false; // Skip the header line
            continue;
        }

        std::istringstream lineStream(line);
        std::string xStr, yStr, label;
        if (!std::getline(lineStream, xStr, ',') ||
            !std::getline(lineStream, yStr, ',')) {
            continue; // Skip malformed lines
        }

        // Convert x and y to doubles
        double x = std::stod(xStr);
        double y = std::stod(yStr);

        // std::cout << "Parsed point: (" << x << ", " << y << ")\n";

        // Store the point in the vector
        haltonPoints.emplace_back(x, y);
        // std::cout << "Stored point: (" << haltonPoints.back().first << ", " << haltonPoints.back().second << ")\n";

        // Update grid tallies and dimensions
        int xPos = static_cast<int>(x);
        int yPos = static_cast<int>(y);
        gridTallies[{xPos, yPos}] += 1;
        totalPoints++;

        gridDimensions.first = std::max(gridDimensions.first, xPos + 1);
        gridDimensions.second = std::max(gridDimensions.second, yPos + 1);

        // If a label exists, store it in labeledPoints
        if (std::getline(lineStream, label, ',') && !label.empty()) {
            labeledPoints[{xPos, yPos}] = label;
            // std::cout << "Found label: " << label << " at position (" << xPos << ", " << yPos << ")\n";
        }
    }

    file.close();

    // // Print Halton points for debugging
    // std::cout << "Halton points loaded:\n";
    // for (const auto& point : haltonPoints) {
    //     std::cout << "  (" << point.first << ", " << point.second << ")\n";
    // }

    // // Output the tallies
    // std::cout << "Grid Tallies:\n";
    // for (const auto& entry : gridTallies) {
    //     std::cout << "Position (" << entry.first.first << ", " << entry.first.second
    //               << ") has tally: " << entry.second << "\n";
    // }

    std::cout << "Total points: " << totalPoints << "\n";
    std::cout << "Grid dimensions: (" << gridDimensions.first << ", " << gridDimensions.second << ")\n";
    std::cout << "Halton points loaded from " << haltonFilePath << std::endl;
}

void EnvironmentCompiler::compileEnvironment(const std::string& filePath) {
    haltonFilePath = filePath;
    // Ensure Halton points are parsed before compiling
    if (gridTallies.empty()) {
        parseHaltonPoints();
    }

    // // Process grid tallies
    // std::cout << "Compiling environment using grid tallies...\n";
    // for (const auto& entry : gridTallies) {
    //     std::cout << "Position (" << entry.first.first << ", " << entry.first.second
    //               << ") has tally: " << entry.second << "\n";
    // }

    // // Process labeled points
    // std::cout << "Processing labeled points...\n";
    // for (const auto& entry : labeledPoints) {
    //     std::cout << "Position (" << entry.first.first << ", " << entry.first.second
    //               << ") has label: " << entry.second << "\n";
    // }

    // Print Halton points for debugging
    // std::cout << "Halton points loaded:\n";
    // for (const auto& point : haltonPoints) {
    //     std::cout << "  (" << point.first << ", " << point.second << ")\n";
    // }

    std::cout << "Environment compilation complete.\n";
}

void EnvironmentCompiler::setLookahead(int lookahead) {
    this->lookahead = lookahead;
}

std::string EnvironmentCompiler::getHaltonFilePath() const {
    return haltonFilePath;
}

std::unordered_map<std::pair<int, int>, int, pair_hash> EnvironmentCompiler::getGridTallies() const {
    return gridTallies;
}

std::unordered_map<std::pair<int, int>, std::string, pair_hash> EnvironmentCompiler::getLabeledPoints() const {
    return labeledPoints;
}

int EnvironmentCompiler::getTotalPoints() const {
    return totalPoints;
}

std::pair<int, int> EnvironmentCompiler::getGridDimensions() const {
    return gridDimensions;
}

int EnvironmentCompiler::getLookahead() const {
    return lookahead;
}

void EnvironmentCompiler::setStandbyLimit(int generalLimit) {
    if (!isGeneralLimitActive) {
        // Destroy the current individualLimits before switching
        standby.individualLimits.~unordered_map();
    }
    standby.generalLimit = generalLimit;
    isGeneralLimitActive = true;
}

void EnvironmentCompiler::setStandbyLimit(const std::unordered_map<std::pair<int, int>, int, pair_hash>& individualLimits) {
    if (isGeneralLimitActive) {
        // Switch from generalLimit to individualLimits
        new (&standby.individualLimits) std::unordered_map<std::pair<int, int>, int, pair_hash>(individualLimits);
    } else {
        // Replace the existing individualLimits
        standby.individualLimits = individualLimits;
    }
    isGeneralLimitActive = false;
}

int EnvironmentCompiler::getStandbyLimit() const {
    if (!isGeneralLimitActive) {
        throw std::logic_error("standbyLimits is currently set to individualLimits, not generalLimit.");
    }
    return standby.generalLimit;
}

std::unordered_map<std::pair<int, int>, int, pair_hash> EnvironmentCompiler::getStandbyLimits() const {
    if (isGeneralLimitActive) {
        throw std::logic_error("standbyLimits is currently set to generalLimit, not individualLimits.");
    }
    return standby.individualLimits;
}

bool EnvironmentCompiler::checkGeneralLimitActive() const {
    return isGeneralLimitActive;
}

std::pair<double, double> EnvironmentCompiler::getPointByIndex(size_t index) const {
    if (index >= haltonPoints.size()) {
        throw std::out_of_range("Index out of range for Halton points");
    }
    return haltonPoints[index];
}

std::vector<std::pair<double, double>> EnvironmentCompiler::getHaltonPoints() const {
    std::cout << "Size of Halton points: " << haltonPoints.size() << std::endl;
    return haltonPoints;
}

