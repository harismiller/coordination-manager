#ifndef AGENT_STATE_H
#define AGENT_STATE_H

#include <string>
#include <list>
#include <utility>

struct AgentState {
    std::pair<double, double> currentPosition;
    std::string status;
    std::list<std::pair<double, double>> pathPlan;
    std::list<char> flags;
    int planIndex;
};

#endif // AGENT_STATE_H