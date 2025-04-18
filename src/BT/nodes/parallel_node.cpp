#include "BT/nodes/parallel_node.h"
#include <iostream>

ParallelNode::ParallelNode(const std::string& name, int successThreshold, int failureThreshold)
    : CompositeNode(name), successThreshold_(successThreshold), failureThreshold_(failureThreshold) {}

NodeStatus ParallelNode::tick() {
    int successCount = 0;
    int failureCount = 0;

    for (auto& child : children_) {
        NodeStatus status = child->tick();

        if (status == NodeStatus::SUCCESS) {
            successCount++;
        } else if (status == NodeStatus::FAILURE) {
            failureCount++;
        }

        // Check if success or failure thresholds are met
        if (successCount >= successThreshold_) {
            std::cout << "ParallelNode '" << name_ << "' succeeded." << std::endl;
            return NodeStatus::SUCCESS;
        }
        if (failureCount >= failureThreshold_) {
            std::cout << "ParallelNode '" << name_ << "' failed." << std::endl;
            return NodeStatus::FAILURE;
        }
    }

    // If neither threshold is met, the node is still running
    return NodeStatus::RUNNING;
}