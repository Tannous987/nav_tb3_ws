// Core ROS2 functionality
#include "rclcpp/rclcpp.hpp"

// Message type used to set the robot's initial pose estimate in the map
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

// Used to convert yaw (theta) into a quaternion orientation
#include "tf2/LinearMath/Quaternion.h"

//
// This node publishes a single /initialpose message to set the robot's pose for AMCL.
//
class InitAmclPosePublisher : public rclcpp::Node
{
public:
    // Constructor: sets up the node and prepares to publish
    InitAmclPosePublisher() : Node("init_amcl_pose_publisher")
    {
        // Declare launch-time parameters with default values
        this->declare_parameter<double>("x", 0.0);       // X position
        this->declare_parameter<double>("y", 0.0);       // Y position
        this->declare_parameter<double>("theta", 0.0);   // Orientation (yaw) in radians
        this->declare_parameter<double>("cov", 0.25);    // Covariance (default = 0.5^2)

        // Set up a publisher to /initialpose with queue size 10
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

        // Create a timer that calls publish_init_pose() after 1 second
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&InitAmclPosePublisher::publish_init_pose, this));
    }

private:
    //
    // Publishes the initial pose to /initialpose
    //
    void publish_init_pose()
    {
        // Only publish if someone is actually listening to /initialpose
        if (publisher_->get_subscription_count() == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for subscribers to /initialpose");
            return;
        }

        // Get parameters from the parameter server
        double x = this->get_parameter("x").as_double();
        double y = this->get_parameter("y").as_double();
        double theta = this->get_parameter("theta").as_double();
        double cov = this->get_parameter("cov").as_double();

        // Fill in the pose message
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.frame_id = "map";           // Pose is in the "map" frame
        msg.header.stamp = this->now();        // Timestamp the message

        // Set position (x, y, z)
        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        msg.pose.pose.position.z = 0.0;         // Flat on the map

        // Convert yaw angle (theta) to quaternion for orientation
        tf2::Quaternion quat;
        quat.setRPY(0, 0, theta);  // roll = 0, pitch = 0, yaw = theta

        msg.pose.pose.orientation.x = quat.x();
        msg.pose.pose.orientation.y = quat.y();
        msg.pose.pose.orientation.z = quat.z();
        msg.pose.pose.orientation.w = quat.w();

        // Set covariance matrix (6x6), only x, y, and theta variances are non-zero
        msg.pose.covariance = {
            cov, 0, 0, 0, 0, 0,
            0, cov, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, cov};

        // Publish the pose to AMCL
        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Published initial AMCL pose.");

        // Cancel the timer after publishing once — this node only sends one message
        timer_->cancel();
    }

    // Publisher for the initial pose
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;

    // Timer that schedules the one-time publishing event
    rclcpp::TimerBase::SharedPtr timer_;
};

//
// Main function — sets up and runs the node
//
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // Initialize ROS2
    rclcpp::spin(std::make_shared<InitAmclPosePublisher>()); // Start the node
    rclcpp::shutdown(); // Cleanup when done
    return 0;
}
