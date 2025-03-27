#include "autonomy_node.h"

using namespace std::chrono_literals;

const std::string bt_xml_dir =
    ament_index_cpp::get_package_share_directory("tb3_autonomy") + "/bt_xml";


//This creates a ROS 2 node named "autonomy_node".
//It declares a ROS parameter (location_file), which stores a YAML file containing navigation waypoints.
AutonomyNode::AutonomyNode(const std::string &nodeName) : Node(nodeName)
{
  this->declare_parameter("location_file","none");

  RCLCPP_INFO(get_logger(), "Init done");
}

void AutonomyNode::setup()
{
  RCLCPP_INFO(get_logger(), "Setting up");
  //Calls create_behavior_tree() to load the behavior tree from tree.xml.
  create_behavior_tree();
  RCLCPP_INFO(get_logger(), "BT created");
  //Creates a ROS timer that updates the behavior tree every 500ms.
  const auto timer_period = 500ms;
  timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&AutonomyNode::update_behavior_tree, this));
  //Runs the ROS node (rclcpp::spin()) until the process is stopped.
  rclcpp::spin(shared_from_this());
  rclcpp::shutdown();
}

//Create and Register Behavior Tree Nodes
void AutonomyNode::create_behavior_tree()
{ //Creates a BehaviorTreeFactory to manage BT nodes.
  BT::BehaviorTreeFactory factory;

  // register bt node
  /*here we are using factory.registerBuilder with a lambda because our nodes requires additional parameters, such as a ROS node pointer.*/
  /*factory.registerNodeType<NodeClass>("NodeName") is used when the node class constructor only has two arguments (name, config).
  SO here we have 3 so we should use factory.registerBuilder with a lambda */
  BT::NodeBuilder builder =
      [=](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<GoToPose>(name, config, shared_from_this());
  };
  //Registers GoToPose so that it can be used in tree.xml.
  factory.registerBuilder<GoToPose>("GoToPose", builder);
  /*GoToPose requires a rclcpp::Node::SharedPtr (the ROS 2 node) in its constructor.
  You need to pass shared_from_this() (from AutonomyNode) to give the BT node access to ROS features (e.g., action client, parameters).
  Because this constructor needs more than just name and config, you can’t use registerNodeType<T>() — you have to manually define a builder function.*/

  // Register your Battery behaviors with node_ptr
  factory.registerBuilder<DecrementBattery>(
    "DecrementBattery",
    [node_ptr = shared_from_this()](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<DecrementBattery>(name, config, node_ptr);
    });

  factory.registerBuilder<CheckBattery>(
    "CheckBattery",
    [node_ptr = shared_from_this()](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<CheckBattery>(name, config, node_ptr);
    });

  factory.registerBuilder<IsBatteryLow>(
    "IsBatteryLow",
    [node_ptr = shared_from_this()](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<IsBatteryLow>(name, config, node_ptr);
    });

  factory.registerBuilder<RechargeBattery>(
    "RechargeBattery",
    [node_ptr = shared_from_this()](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<RechargeBattery>(name, config, node_ptr);
    });

  factory.registerBuilder<WaitSeconds>(
    "WaitSeconds",
    [node_ptr = shared_from_this()](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<WaitSeconds>(name, config, node_ptr);
    });

    RCLCPP_INFO(get_logger(), bt_xml_dir.c_str());
    //Loads the BT structure from tree.xml.
    tree_ = factory.createTreeFromFile(bt_xml_dir + "/tree.xml");
    RCLCPP_INFO(get_logger(), "3");
}
//Tick the Behavior Tree
void AutonomyNode::update_behavior_tree()
{
  //Every 500ms, the tree executes (tickRoot()).
  BT::NodeStatus tree_status = tree_.tickRoot();
  //If still running → keep going.
  if (tree_status == BT::NodeStatus::RUNNING)
  {
    return;
  }
  //If successful → log "Finished Navigation".
  else if (tree_status == BT::NodeStatus::SUCCESS)
  {
    //RCLCPP_INFO(this->get_logger(), "Finished Navigation");
  }
  //If failed → cancel the timer.
  else if (tree_status == BT::NodeStatus::FAILURE)
  {
    //RCLCPP_INFO(this->get_logger(), "Need to charge");
    //timer_->cancel();
  }
}
//Run the Node (main())
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutonomyNode>("autonomy_node");
  node->setup();

  return 0;
}

