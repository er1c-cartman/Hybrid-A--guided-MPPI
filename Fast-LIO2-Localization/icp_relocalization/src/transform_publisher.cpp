#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class TransformPublisherNode : public rclcpp::Node
{
public:
    TransformPublisherNode()
        : Node("transform_publisher_node"), latest_transform_received(false)
    {
        this->declare_parameter<std::string>("odom_frame_id", "odom");
        this->declare_parameter<std::string>("map_frame_id", "map");

        this->get_parameter("odom_frame_id", odom_frame_id);
        this->get_parameter("map_frame_id", map_frame_id);

        subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "icp_result", 10, std::bind(&TransformPublisherNode::callback, this, std::placeholders::_1));
        
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Publish at a fixed rate, e.g., 10 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TransformPublisherNode::publishTransform, this));
    }

private:
    void callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // Update the latest transform with data from icp_result
        latest_transform.header.stamp = this->now();
        latest_transform.header.frame_id = map_frame_id;
        latest_transform.child_frame_id = odom_frame_id;
        latest_transform.transform.translation.x = msg->pose.pose.position.x;
        latest_transform.transform.translation.y = msg->pose.pose.position.y;
        latest_transform.transform.translation.z = msg->pose.pose.position.z;
        latest_transform.transform.rotation = msg->pose.pose.orientation;

        latest_transform_received = true;
    }

    void publishTransform()
    {
        if (latest_transform_received)
        {
            // Set the timestamp to the current time for each broadcast
            latest_transform.header.stamp = this->now();
            broadcaster_->sendTransform(latest_transform);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::TransformStamped latest_transform;
    std::string odom_frame_id, map_frame_id;
    bool latest_transform_received;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
