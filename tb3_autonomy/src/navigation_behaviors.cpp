#include "navigation_behaviors.h"
#include "yaml-cpp/yaml.h"
#include <string>
//class Constructor (GoToPose::GoToPose)
GoToPose::GoToPose(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
  action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
  done_flag_ = false;
}
//This is a Behavior Tree node that executes the GoToPose behavior.
//It initializes an action client that sends navigation goals to the /navigate_to_pose action server.
//The done_flag_ is set to false, meaning navigation has not yet finished.

//Defining Input Ports (providedPorts())
BT::PortsList GoToPose::providedPorts()
{
  return {BT::InputPort<std::string>("loc")};
}
//It defines a Behavior Tree (BT) input port named "loc", which represents a location key (e.g., "location1", "location2").

BT::NodeStatus GoToPose::onStart()
{
  // Get location key from port and read YAML file
  BT::Optional<std::string> loc = getInput<std::string>("loc");
  const std::string location_file = node_ptr_->get_parameter("location_file").as_string();

  YAML::Node locations = YAML::LoadFile(location_file);

  std::vector<float> pose = locations[loc.value()].as<std::vector<float>>();

  // setup action client
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  //Until this point, nav_to_pose_callback doesn't execute yet. It's just registered as the function to call later when the goal finishes.
  send_goal_options.result_callback = std::bind(&GoToPose::nav_to_pose_callback, this, std::placeholders::_1);

  // make pose
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.pose.position.x = pose[0];
  goal_msg.pose.pose.position.y = pose[1];
  //Orientation: Uses a quaternion (pose[2] is the yaw angle).
  tf2::Quaternion q;
  q.setRPY(0, 0, pose[2]);
  q.normalize(); 
  goal_msg.pose.pose.orientation = tf2::toMsg(q);

  //Sends the goal to Navigation 2 using an action client.
  done_flag_ = false;
  //Sends a navigation goal asynchronously (doesn't block execution).
  action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(node_ptr_->get_logger(), "Go To: %s", loc.value().c_str());
  return BT::NodeStatus::RUNNING;
  //Returns RUNNING, meaning the robot is still navigating.
}

BT::NodeStatus GoToPose::onRunning()
{
  //Checks if navigation is complete using done_flag_.
  //If done_flag_ == true, it returns SUCCESS, meaning the goal was reached.
  //Otherwise, it keeps running.
  if (done_flag_)
  {
    //RCLCPP_INFO(node_ptr_->get_logger(), "Goal reached\n");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::RUNNING;
  }
}

void GoToPose::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
  //This callback is executed when the navigation goal is completed.
  //If a result is received, done_flag_ is set to true, meaning the robot has arrived.
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    done_flag_ = true;
    RCLCPP_INFO(node_ptr_->get_logger(), "Location successfully reached!");
  }
  else
  {
    done_flag_ = false;
    RCLCPP_WARN(node_ptr_->get_logger(), "Navigation failed or was cancelled.");
  }
}