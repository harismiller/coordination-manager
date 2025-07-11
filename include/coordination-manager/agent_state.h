#ifndef AGENT_STATE_H
#define AGENT_STATE_H

#include <string>
#include <vector>
#include <utility>
#include <stdbool.h>

struct AgentState {
    std::pair<double, double> currentPosition;
    std::vector<std::pair<double, double>> waypoints;
    std::vector<char> actions;
    std::vector<int> index;
    int planIndex;
    bool agentStatus;
    int curTask;
    bool taskStatus;
};

#endif // AGENT_STATE_H