#ifndef BATTERY_BEHAVIOR_H
#define BATTERY_BEHAVIOR_H

// Pulling in the core behavior tree library
#include "behaviortree_cpp_v3/behavior_tree.h"

// ROS2 stuff — mainly for logging and using the node handle
#include "rclcpp/rclcpp.hpp"

// We’ll need these for the WaitSeconds node
#include <chrono>
#include <thread>

//
// A simple context to keep track of battery level across all nodes.
// It’s static, so everything accesses the same battery value.
//
class BatteryContext {
public:
  static int battery_level;
};

//
// Node: DecrementBattery
// Lowers the battery level by 25% each time it runs.
//
class DecrementBattery : public BT::SyncActionNode {
public:
  // Standard constructor: we pass in the node name, config, and ROS2 node for logging
  DecrementBattery(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_ptr_(node) {}

  // This node doesn’t need any ports (inputs or outputs)
  static BT::PortsList providedPorts() { return {}; }

  // What this node actually does when it runs (defined in the .cpp file)
  BT::NodeStatus tick() override;

private:
  // Used for printing logs to the ROS2 console
  rclcpp::Node::SharedPtr node_ptr_;
};

//
// Node: CheckBattery
// Checks if the battery is above 0%. Returns SUCCESS if it is.
//
class CheckBattery : public BT::ConditionNode {
public:
  // Same setup as before — name, config, and ROS2 node
  CheckBattery(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
    : BT::ConditionNode(name, config), node_ptr_(node) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_ptr_;
};

//
// Node: IsBatteryLow
// Checks if the battery is completely empty (== 0%) and returns SUCCESS if so.
//
class IsBatteryLow : public BT::ConditionNode {
public:
  IsBatteryLow(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
    : BT::ConditionNode(name, config), node_ptr_(node) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_ptr_;
};

//
// Node: RechargeBattery
// Resets the battery level back to 100%. Simple and effective.
//
class RechargeBattery : public BT::SyncActionNode {
public:
  RechargeBattery(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_ptr_(node) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_ptr_;
};

//
// Node: WaitSeconds
// This one pauses execution for a given number of seconds.
// We read that value from the behavior tree input port.
//
class WaitSeconds : public BT::SyncActionNode {
public:
  WaitSeconds(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_ptr_(node) {}

  // We expect one input called "seconds", which is how long we’ll sleep
  static BT::PortsList providedPorts() {
    return { BT::InputPort<int>("seconds") };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_ptr_;
};

#endif
