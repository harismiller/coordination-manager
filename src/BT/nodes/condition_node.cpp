#include "BT/nodes/condition_node.h"
#include <iostream>

ConditionNode::ConditionNode(const std::string& name, std::function<NodeStatus()> callback)
    : BTNode(name), callback_(callback), last_status_(NodeStatus::FAILURE) {}

NodeStatus ConditionNode::tick() {
    if (callback_) {
        last_status_ = callback_();
        // Ensure the condition only returns SUCCESS or FAILURE
        if (last_status_ != NodeStatus::SUCCESS && last_status_ != NodeStatus::FAILURE) {
            std::cerr << "ConditionNode '" << name_ << "' returned invalid status. Defaulting to FAILURE." << std::endl;
            last_status_ = NodeStatus::FAILURE;
        }
    } else {
        std::cout << "Evaluating Condition: " << name_ << " - Default behavior" << std::endl;
        last_status_ = NodeStatus::SUCCESS; // Default behavior if no callback is provided
    }
    return last_status_;
}

NodeStatus ConditionNode::getLastStatus() const {
    return last_status_;
}