#include "BT/nodes/action_node.h"
#include "BT/nodes/condition_node.h"
#include "BT/nodes/sequence_node.h"
#include <memory>
#include <iostream>
#include <functional> // For std::bind

// Callback function to print a message
NodeStatus printMessageCallback() {
    std::cout << "Executing Print Message Action: Hello from Callback Pointer!" << std::endl;
    return NodeStatus::SUCCESS;
}

// Callback function to increment a counter
NodeStatus incrementCounterCallback(int* counter) {
    (*counter)++;
    std::cout << "Executing Increment Counter Action: Counter = " << *counter << std::endl;
    return NodeStatus::SUCCESS;
}

// Callback function for condition
NodeStatus conditionCallback(int* counter) {
    std::cout << "Evaluating custom condition..." << std::endl;
    if (*counter >= 5) {
        std::cout << "Counter exceeded limit, returning FAILURE." << std::endl;
        return NodeStatus::FAILURE; // Simulate failure if counter exceeds 5
    } else {
        std::cout << "Counter is within limit, returning SUCCESS." << std::endl;
        return NodeStatus::SUCCESS; // Simulate success otherwise
    }
}

// Callback function for a condition in the subtree
NodeStatus subtreeConditionCallback() {
    std::cout << "Evaluating Subtree Condition" << std::endl;
    return NodeStatus::SUCCESS;
}

// Callback function for an action in the subtree
NodeStatus subtreeActionCallback() {
    std::cout << "Executing Subtree Action" << std::endl;
    return NodeStatus::SUCCESS;
}

int main() {
    auto root = std::make_shared<SequenceNode>("Root");

    int counter = 0;

    // Use std::bind to create callback references
    auto conditionCallbackRef = std::bind(conditionCallback, &counter);
    auto incrementCallbackRef = std::bind(incrementCounterCallback, &counter);

    // Create nodes with callback references
    auto condition = std::make_shared<ConditionNode>("Custom Condition", conditionCallbackRef);
    auto printAction = std::make_shared<ActionNode>("Print Message", &printMessageCallback);
    auto incrementAction = std::make_shared<ActionNode>("Increment Counter", incrementCallbackRef);

    // Create the root of tree2 (subtree)
    auto tree2 = std::make_shared<SequenceNode>("Tree2");

    // Add nodes to tree2
    auto subtreeCondition = std::make_shared<ConditionNode>("Subtree Condition", &subtreeConditionCallback);
    auto subtreeAction = std::make_shared<ActionNode>("Subtree Action", &subtreeActionCallback);
    tree2->addChild(subtreeCondition);
    tree2->addChild(subtreeAction);

    root->addChild(condition);
    root->addChild(printAction);
    root->addChild(tree2);    // Add the subtree (tree2)
    root->addChild(incrementAction);

    while (true) {
        std::cout << "~~~~ Running Behavior Tree ~~~~" << std::endl;
        NodeStatus status = root->tick();

        // Access the status of individual nodes
        std::cout << "Condition Node Status: " << condition->getLastStatus() << std::endl;
        std::cout << "Print Action Node Status: " << printAction->getLastStatus() << std::endl;
        std::cout << "Increment Action Node Status: " << incrementAction->getLastStatus() << std::endl;

        if (status == NodeStatus::SUCCESS) {
            std::cout << "Behavior Tree completed successfully!" << std::endl;
            // break;
        } else if (status == NodeStatus::FAILURE) {
            std::cout << "Behavior Tree failed!" << std::endl;
            break;
        }
    }

    return 0;
}