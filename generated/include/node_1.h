#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/control_node.h>
#include <behaviortree_cpp/decorator_node.h>

namespace MyProject {

namespace MyProject {

class Sequence : public BT::ControlNode {
public:
    Sequence(const std::string& name, const BT::NodeConfig& config);
    virtual ~Sequence() = default;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    // Node-specific private members and methods
};

} // namespace MyProject

} // namespace MyProject
