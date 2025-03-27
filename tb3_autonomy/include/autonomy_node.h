#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "navigation_behaviors.h" //Includes GoToPose Node
#include "battery_behavior.h"
#include "ament_index_cpp/get_package_share_directory.hpp"//// Get package paths
#include <tf2/LinearMath/Quaternion.h> //Quaternion math for orientation

class AutonomyNode : public rclcpp::Node
{
public:
  explicit AutonomyNode(const std::string &node_name);
  void setup(); // Initializes the behavior tree.
  void create_behavior_tree(); // Loads the BT from tree.xml.
  void update_behavior_tree(); // Quaternion math for orientation

private:
  rclcpp::TimerBase::SharedPtr timer_; // Timer to tick BT every 500ms
  //Executes the update_behavior_tree() function every 500ms.
  BT::Tree tree_;
  //Stores the Behavior Tree instance.
};