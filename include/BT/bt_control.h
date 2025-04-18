#ifndef BT_CONTROL_H
#define BT_CONTROL_H

#include <string>
#include <vector>
#include <memory>

enum class NodeStatus { SUCCESS, FAILURE, RUNNING };

std::ostream& operator<<(std::ostream& os, const NodeStatus& status);

class BTNode {
public:
    explicit BTNode(const std::string& name) : name_(name) {}
    virtual ~BTNode() = default;

    virtual NodeStatus tick() = 0;

protected:
    std::string name_;
};

class CompositeNode : public BTNode {
public:
    explicit CompositeNode(const std::string& name) : BTNode(name) {}
    void addChild(std::shared_ptr<BTNode> child) { children_.push_back(child); }

    // Override the tick() method
    NodeStatus tick() override;

protected:
    std::vector<std::shared_ptr<BTNode>> children_;
};

#endif // BT_CONTROL_H