#include "battery_behavior.h"  // Include the header where behavior tree node classes are declared

// Initialize the static member variable that holds the battery level
int BatteryContext::battery_level = 100;

// Node: DecrementBattery
// Decreases battery level by 25%. If battery level goes below 0, clamp it to 0.
BT::NodeStatus DecrementBattery::tick() {
  BatteryContext::battery_level -= 25;
  if (BatteryContext::battery_level < 0) BatteryContext::battery_level = 0;
  RCLCPP_INFO(node_ptr_->get_logger(), "Decrement Battery-> Battery level: %d%%", BatteryContext::battery_level);
  return BT::NodeStatus::SUCCESS;
}

// Node: CheckBattery
// Returns SUCCESS if battery is above 0%, otherwise FAILURE.
BT::NodeStatus CheckBattery::tick() {
  RCLCPP_INFO(node_ptr_->get_logger(), "Check Battery");
  return (BatteryContext::battery_level > 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// Node: IsBatteryLow
// Returns SUCCESS if battery is at 0%, otherwise FAILURE.
BT::NodeStatus IsBatteryLow::tick() {
  if (BatteryContext::battery_level == 0)
    RCLCPP_INFO(node_ptr_->get_logger(), "Battery low! Need to charge.");
  return (BatteryContext::battery_level == 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// Node: RechargeBattery
// Recharges battery back to 100%.
BT::NodeStatus RechargeBattery::tick() {
  BatteryContext::battery_level = 100;
  RCLCPP_INFO(node_ptr_->get_logger(), "Battery recharged to 100%%");
  return BT::NodeStatus::SUCCESS;
}

// Node: WaitSeconds
// Retrieves an input value for "seconds", then waits for that duration.
// Returns FAILURE if the input is not found.
BT::NodeStatus WaitSeconds::tick() {
  int seconds;
  if (!getInput("seconds", seconds)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Failed to get input seconds");
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(node_ptr_->get_logger(), "Waiting for %d seconds to recharge...", seconds);
  std::this_thread::sleep_for(std::chrono::seconds(seconds));
  return BT::NodeStatus::SUCCESS;
}
