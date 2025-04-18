#ifndef SEQUENCE_NODE_H
#define SEQUENCE_NODE_H

#include "BT/bt_control.h"
#include <string>

class SequenceNode : public CompositeNode {
public:
    explicit SequenceNode(const std::string& name);
    NodeStatus tick() override;
};

#endif // SEQUENCE_NODE_H