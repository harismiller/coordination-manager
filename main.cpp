#include "coordination_manager.h"
#include "agent_state.h"
#include <iostream>
#include <string>
#include <unordered_map>

int main() {
    // Prompt the user for the Halton file path
    std::string haltonFile;
    std::cout << "Enter the name of the Halton file (default: halton_points.csv): ";
    std::getline(std::cin, haltonFile);

    // Use the default file if no input is provided
    if (haltonFile.empty()) {
        haltonFile = "halton_points.csv";
    }

    // Create an instance of CoordinationManager and start it
    CoordinationManager manager;
    manager.start(haltonFile);

    // Test setting and getting generalLimit
    std::cout << "\nTesting generalLimit...\n";
    manager.setStandbyLimit(10); // Set generalLimit to 10
    try {
        int generalLimit = manager.getStandbyLimit();
        std::cout << "General Standby Limit: " << generalLimit << std::endl;
    } catch (const std::logic_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    // Test setting and getting individualLimits
    std::cout << "\nTesting individualLimits...\n";
    std::unordered_map<std::pair<int, int>, int, pair_hash> individualLimits = {
        {{0, 0}, 5},
        {{1, 1}, 10},
        {{2, 2}, 15}
    };
    manager.setStandbyLimit(individualLimits); // Set individualLimits
    try {
        auto retrievedLimits = manager.getStandbyLimits();
        std::cout << "Individual Standby Limits:\n";
        for (const auto& [key, value] : retrievedLimits) {
            std::cout << "  Position (" << key.first << ", " << key.second << ") -> Limit: " << value << "\n";
        }
    } catch (const std::logic_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    // Test calling the wrong getter
    std::cout << "\nTesting error handling for wrong getter...\n";
    try {
        int generalLimit = manager.getStandbyLimit(); // This should throw an error
        std::cout << "General Standby Limit: " << generalLimit << std::endl;
    } catch (const std::logic_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cout << "isGeneralLimitActive: " << (manager.checkGeneralLimitActive() ? "true" : "false") << std::endl;
    }

    // Test lookahead functionality
    std::cout << "\nTesting lookahead...\n";
    manager.setLookahead(5); // Set lookahead to 5
    int lookahead = manager.getLookahead(lookahead);
    std::cout << "Lookahead value: " << lookahead << std::endl;

    manager.setLookahead(10); // Update lookahead to 10
    lookahead = manager.getLookahead(lookahead);
    std::cout << "Updated Lookahead value: " << lookahead << std::endl;

    // Test agent management
    std::cout << "\nTesting agent management...\n";
    AgentState agent1 = {
        {0.0, 0.0}, // Current position
        "moving",   // Status
        {{1.0, 1.0}, {2.0, 2.0}, {3.0, 3.0}}, // Path plan
        {'g', 'g', 'r'}, // Flags
        0 // Plan index
    };
    manager.addAgent(1, agent1);
    std::cout << "Agent 1 added with ID 1." << std::endl;

    if (manager.hasAgent(1)) {
        std::cout << "Agent 1 exists." << std::endl;
        const AgentState* state = manager.getAgent(1);
        if (state) {
            std::cout << "Agent 1 current position: (" << state->currentPosition.first << ", " << state->currentPosition.second << ")" << std::endl;
            std::cout << "Agent 1 status: " << state->status << std::endl;
            std::cout << "Agent 1 path plan: ";
            for (const auto& point : state->pathPlan) {
                std::cout << "(" << point.first << ", " << point.second << ") ";
            }
            std::cout << std::endl;
            std::cout << "Agent 1 flags: ";
            for (const auto& flag : state->flags) {
                std::cout << flag << " ";
            }
            std::cout << std::endl;
        } else {
            std::cout << "Failed to retrieve Agent 1 state." << std::endl;
        }
    } else {
        std::cout << "Agent 1 does not exist." << std::endl;
    }

    return 0;
}