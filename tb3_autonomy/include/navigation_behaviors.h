#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//GoToPose inherits from StatefulActionNode (a behavior tree node that holds states).
class GoToPose : public BT::StatefulActionNode
{
public:
  //Declares the GoToPose class, which is a Behavior Tree (BT) node that tells the robot to navigate to a specific location.
  GoToPose(const std::string &name, //The BT node name.
           const BT::NodeConfiguration &config,//Configuration (e.g., input ports).
           rclcpp::Node::SharedPtr node_ptr); //A shared pointer to a ROS 2 node.
  //Defines NavigateToPose action → Used to send the robot to a target position.
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  rclcpp::Node::SharedPtr node_ptr_;
  //Defines the ROS 2 action client to send navigation goals to the /navigate_to_pose server.
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_ptr_;
  //Uses done_flag_ to track whether the robot has reached the goal.
  bool done_flag_;

  //Includes method overrides for BT execution.
  BT::NodeStatus onStart() override; //Reads the target location from YAML and sends a navigation goal.
  BT::NodeStatus onRunning() override; // Checks if the robot has reached its goal.
  void onHalted() override{}; // Defines what happens if the node is stopped (empty here).

  static BT::PortsList providedPorts();
  //Handles the response from Nav2 after sending a navigation goal.
  // Action Client callback
  void nav_to_pose_callback(const GoalHandleNav::WrappedResult &result);
};