#include "BT/nodes/action_node.h"
#include <iostream>

ActionNode::ActionNode(const std::string& name, std::function<NodeStatus()> callback)
    : BTNode(name), callback_(callback), last_status_(NodeStatus::RUNNING) {}

NodeStatus ActionNode::tick() {
    if (callback_) {
        last_status_ = callback_();
    } else {
        std::cout << "Executing Action: " << name_ << " - Default behavior" << std::endl;
        last_status_ = NodeStatus::SUCCESS;
    }
    return last_status_;
}

NodeStatus ActionNode::getLastStatus() const {
    return last_status_;
}