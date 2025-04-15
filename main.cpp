#include "coordination_manager.h"
#include <iostream>
#include <string>

int main() {
    // Prompt the user for the Halton file path
    std::string haltonFile;
    std::cout << "Enter the path to the Halton file (default: halton_points.csv): ";
    std::getline(std::cin, haltonFile);

    // Use the default file if no input is provided
    if (haltonFile.empty()) {
        haltonFile = "halton_points.csv";
    }

    // Create an instance of CoordinationManager and start it
    CoordinationManager manager;
    manager.start(haltonFile);

    return 0;
}