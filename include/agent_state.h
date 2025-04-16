#ifndef AGENT_STATE_H
#define AGENT_STATE_H

#include <string>
#include <list>
#include <utility> // for std::pair

struct AgentState {
    std::pair<double, double> currentPosition; // Current position (x, y)
    std::string status;                        // Status (e.g., "idle", "moving", "completed")
    std::list<std::pair<double, double>> pathPlan; // List of (x, y) for the planned path
    std::list<char> flags;                     // List of flags (e.g., 'A', 'B', etc.)
    int planIndex;                             // Current index in the path plan
};

#endif // AGENT_STATE_H