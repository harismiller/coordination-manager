#ifndef PARALLEL_NODE_H
#define PARALLEL_NODE_H

#include "BT/bt_control.h"
#include <vector>
#include <memory>

class ParallelNode : public CompositeNode {
public:
    explicit ParallelNode(const std::string& name, int successThreshold, int failureThreshold);

    NodeStatus tick() override;

private:
    int successThreshold_; // Number of children that must succeed for the node to succeed
    int failureThreshold_; // Number of children that must fail for the node to fail
};

#endif // PARALLEL_NODE_H