#include "system_compiler.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <unordered_map>

SystemCompiler::SystemCompiler(const std::string& filePath) : haltonFilePath(filePath) {}

void SystemCompiler::parseHaltonPoints() {
    std::ifstream file(haltonFilePath);
    if (!file.is_open()) {
        std::cerr << "Error loading Halton points: Unable to open file " << haltonFilePath << std::endl;
        return;
    }

    gridTallies.clear(); // Clear any existing data
    labeledPoints.clear(); // Clear labeled points data
    totalPoints = 0; // Reset total points
    gridDimensions = {0, 0}; // Reset grid dimensions

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

        // Convert x and y to integers (grid positions)
        double x = std::stod(xStr);
        double y = std::stod(yStr);

        int xPos = static_cast<int>(x);
        int yPos = static_cast<int>(y);

        // Increment tally at the grid position
        gridTallies[{xPos, yPos}] += 1;

        // Update total points
        totalPoints++;

        // Update grid dimensions
        gridDimensions.first = std::max(gridDimensions.first, xPos + 1);
        gridDimensions.second = std::max(gridDimensions.second, yPos + 1);

        // If a label exists, store it in labeledPoints
        if (std::getline(lineStream, label, ',') && !label.empty()) {
            labeledPoints[{xPos, yPos}] = label;
            // std::cout << "Found label: " << label << " at position (" << xPos << ", " << yPos << ")\n";
        }
    }

    file.close();

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

void SystemCompiler::compileSystem() {
    // Ensure Halton points are parsed before compiling
    if (gridTallies.empty()) {
        parseHaltonPoints();
    }

    // // Process grid tallies
    // std::cout << "Compiling system using grid tallies...\n";
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

    // Add any additional logic for system compilation here
    std::cout << "System compilation complete.\n";
}

void SystemCompiler::setLookahead(int lookahead) {
    this->lookahead = lookahead;
}

std::string SystemCompiler::getHaltonFilePath() const {
    return haltonFilePath;
}

std::unordered_map<std::pair<int, int>, int, pair_hash> SystemCompiler::getGridTallies() const {
    return gridTallies;
}

std::unordered_map<std::pair<int, int>, std::string, pair_hash> SystemCompiler::getLabeledPoints() const {
    return labeledPoints;
}

int SystemCompiler::getTotalPoints() const {
    return totalPoints;
}

std::pair<int, int> SystemCompiler::getGridDimensions() const {
    return gridDimensions;
}

int SystemCompiler::getLookahead() const {
    return lookahead;
}