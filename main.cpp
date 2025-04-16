#include "coordination_manager.h"
#include "agent_data.h"
#include <iostream>
#include <string>



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

    manager.setLookahead(3); // Set lookahead to 5
    int lookahead;
    manager.getLookahead(lookahead); // Get the lookahead value
    std::cout << "Current lookahead: " << lookahead << std::endl;

    AgentState agent1 = {
        {0.0, 0.0}, // Current position
        "moving", // Status
        {{1.0, 1.0}, {2.0, 2.0}, {3.0, 3.0}}, // Path plan
        {'g', 'g','r'}, // Flags
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