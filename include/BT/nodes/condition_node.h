#ifndef CONDITION_NODE_H
#define CONDITION_NODE_H

#include "BT/bt_control.h"
#include <functional>
#include <iostream>

class ConditionNode : public BTNode {
public:
    explicit ConditionNode(const std::string& name, std::function<NodeStatus()> callback = nullptr);
    NodeStatus tick() override;

    // Method to retrieve the last status of the node
    NodeStatus getLastStatus() const;

private:
    std::function<NodeStatus()> callback_;
    NodeStatus last_status_; // Store the last status of the node
};

#endif // CONDITION_NODE_H