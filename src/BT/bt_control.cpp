#include "BT/bt_control.h"
#include <iostream>

// Base functionality for CompositeNode
NodeStatus CompositeNode::tick() {
    std::cout << "Ticking Composite Node: " << name_ << std::endl;

    for (auto& child : children_) {
        NodeStatus status = child->tick();
        if (status == NodeStatus::RUNNING || status == NodeStatus::FAILURE) {
            return status; // Return immediately if a child is running or fails
        }
    }

    return NodeStatus::SUCCESS; // All children succeeded
}

// Utility function to print the status of a node
std::string nodeStatusToString(NodeStatus status) {
    switch (status) {
        case NodeStatus::SUCCESS:
            return "SUCCESS";
        case NodeStatus::FAILURE:
            return "FAILURE";
        case NodeStatus::RUNNING:
            return "RUNNING";
        default:
            return "UNKNOWN";
    }
}

std::ostream& operator<<(std::ostream& os, const NodeStatus& status) {
    os << nodeStatusToString(status);
    return os;
}