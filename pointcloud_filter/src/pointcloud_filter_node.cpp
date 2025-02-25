#include <rclcpp/rclcpp.hpp>  // Include the rclcpp header for Node class
#include <sensor_msgs/msg/point_cloud2.hpp>  // For handling PointCloud2 messages
#include <pcl_conversions/pcl_conversions.h>  // For converting between ROS and PCL
#include <pcl/point_cloud.h>  // PCL library for handling point clouds
#include <pcl/point_types.h>  // PCL types for point clouds
#include <vector>
#include <string>
#include <memory>
#include <cmath>

struct AngleFilter {
  double min_angle;
  double max_angle;
};

struct AreaFilter {
  double min_x;
  double max_x;
  double min_y;
  double max_y;
};

class PointCloudFilter : public rclcpp::Node
{
public:
  PointCloudFilter(const rclcpp::NodeOptions & options)
  : Node("pointcloud_filter_node", options), total_points_received_(0), filtered_by_area_(0)
  {
    // Declare parameters
    this->declare_parameter<std::string>("input_topic", "input_pointcloud");
    this->declare_parameter<std::string>("output_topic", "filtered_pointcloud");
    this->declare_parameter<double>("distance_threshold", 0.1);
    this->declare_parameter<double>("max_distance", 20.0); // Max distance parameter

    // Ground filter parameters
    this->declare_parameter<double>("ground_ignore_radius", 10.0); // Distance beyond which ground points are ignored
    this->declare_parameter<double>("ground_height_threshold", -0.5); // Height below which points are considered ground

    // Load robot body boundaries for filtering
    this->declare_parameter<double>("robot_min_x", -0.5);
    this->declare_parameter<double>("robot_max_x", 0.5);
    this->declare_parameter<double>("robot_min_y", -0.5);
    this->declare_parameter<double>("robot_max_y", 0.5);

    // Load angle filters
    this->declare_parameter<std::vector<double>>("angle_filters", {});

    // Parameter assignment
    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    distance_threshold_ = this->get_parameter("distance_threshold").as_double();
    max_distance_ = this->get_parameter("max_distance").as_double();
    ground_ignore_radius_ = this->get_parameter("ground_ignore_radius").as_double();
    ground_height_threshold_ = this->get_parameter("ground_height_threshold").as_double();

    // Robot body filtering
    filter_area_.min_x = this->get_parameter("robot_min_x").as_double();
    filter_area_.max_x = this->get_parameter("robot_max_x").as_double();
    filter_area_.min_y = this->get_parameter("robot_min_y").as_double();
    filter_area_.max_y = this->get_parameter("robot_max_y").as_double();

    // Angle filters
    std::vector<double> angle_filters_vec = this->get_parameter("angle_filters").as_double_array();
    for (size_t i = 0; i < angle_filters_vec.size(); i += 2) {
      if (i + 1 < angle_filters_vec.size()) {
        angle_filters_.emplace_back(AngleFilter{angle_filters_vec[i], angle_filters_vec[i + 1]});
      }
    }

    // Subscription and Publisher
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, 10, std::bind(&PointCloudFilter::pointcloud_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

    // Timer for printing point counts
    timer_ = this->create_wall_timer(
      std::chrono::seconds(5), std::bind(&PointCloudFilter::print_point_count, this));
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    total_points_received_ += cloud->points.size();

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    filtered_cloud->points.reserve(cloud->points.size());

    int filtered_in_this_callback = 0;

    for (const auto& point : cloud->points) {
      double distance = std::sqrt(point.x * point.x + point.y * point.y);

      // Ground points filtering beyond the ignore radius
      if (distance > ground_ignore_radius_ && point.z < ground_height_threshold_) {
        continue; // Skip ground points beyond the radius
      }

      if (distance < distance_threshold_ || distance > max_distance_) {
        continue; // Distance filter
      }

      double xy_angle = std::atan2(point.y, point.x);
      bool in_filter = false;
      for (const auto& filter : angle_filters_) {
        if (xy_angle >= filter.min_angle && xy_angle <= filter.max_angle) {
          in_filter = true;
          break;
        }
      }
      if (in_filter) {
        continue; // Angle filter
      }

      if (point.x >= filter_area_.min_x && point.x <= filter_area_.max_x &&
          point.y >= filter_area_.min_y && point.y <= filter_area_.max_y) {
        filtered_in_this_callback++;
        continue; // Robot body area filter
      }

      // Add valid point to filtered cloud
      filtered_cloud->points.push_back(point);
    }

    filtered_by_area_ += filtered_in_this_callback;

    sensor_msgs::msg::PointCloud2 output;
    pcl::toPCLPointCloud2(*filtered_cloud, pcl_pc2);
    pcl_conversions::fromPCL(pcl_pc2, output);
    output.header.frame_id = msg->header.frame_id;
    output.header.stamp = msg->header.stamp;
    publisher_->publish(output);
  }

  void print_point_count()
  {
    // Print total points received and filtered points
    // RCLCPP_INFO(this->get_logger(), "Total points received: %d", total_points_received_);
    // RCLCPP_INFO(this->get_logger(), "Total points filtered by robot body area: %d", filtered_by_area_);

    // Reset the counters
    total_points_received_ = 0;
    filtered_by_area_ = 0;
  }

  std::string input_topic_;
  std::string output_topic_;
  double distance_threshold_;
  double max_distance_;
  double ground_ignore_radius_;   // Radius beyond which ground points are ignored
  double ground_height_threshold_; // Height threshold for ground points

  AreaFilter filter_area_;  // Specific area filter for the robot body
  std::vector<AngleFilter> angle_filters_;  // XY angle filter
  int total_points_received_;   // Track total points received from the LiDAR
  int filtered_by_area_;        // Track points filtered by the robot body area

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer_;  // Timer for printing point counts every 5 seconds
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;

  auto node = std::make_shared<PointCloudFilter>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
