#include "BT/nodes/sequence_node.h"

SequenceNode::SequenceNode(const std::string& name) : CompositeNode(name) {}

NodeStatus SequenceNode::tick() {
    for (auto& child : children_) {
        NodeStatus status = child->tick();
        if (status != NodeStatus::SUCCESS) {
            return status; // Return immediately if a child fails or is running
        }
    }
    return NodeStatus::SUCCESS; // All children succeeded
}