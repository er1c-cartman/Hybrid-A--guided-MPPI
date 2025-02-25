#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp> // Include the Livox custom message header

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

class CustomPointCloudFilter : public rclcpp::Node
{
public:
  CustomPointCloudFilter(const rclcpp::NodeOptions & options)
  : Node("custom_pointcloud_filter_node", options), total_points_received_(0), filtered_by_area_(0)
  {
    this->declare_parameter<std::string>("input_topic", "input_pointcloud");
    this->declare_parameter<std::string>("output_topic", "filtered_pointcloud");
    this->declare_parameter<double>("distance_threshold", 0.1);
    this->declare_parameter<double>("max_distance", 20.0); // Max distance parameter

    // New: Declaring the filter area (robot body) to be filtered
    this->declare_parameter<double>("robot_min_x", -0.5); // Min X boundary for robot body
    this->declare_parameter<double>("robot_max_x", 0.5);  // Max X boundary for robot body
    this->declare_parameter<double>("robot_min_y", -0.5); // Min Y boundary for robot body
    this->declare_parameter<double>("robot_max_y", 0.5);  // Max Y boundary for robot body

    // Load angle filters (XY plane)
    this->declare_parameter<std::vector<double>>("angle_filters", {});

    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    distance_threshold_ = this->get_parameter("distance_threshold").as_double();
    max_distance_ = this->get_parameter("max_distance").as_double();

    // Load robot body boundaries for filtering
    filter_area_.min_x = this->get_parameter("robot_min_x").as_double();
    filter_area_.max_x = this->get_parameter("robot_max_x").as_double();
    filter_area_.min_y = this->get_parameter("robot_min_y").as_double();
    filter_area_.max_y = this->get_parameter("robot_max_y").as_double();

    // Load angle filters
    std::vector<double> angle_filters_vec = this->get_parameter("angle_filters").as_double_array();
    for (size_t i = 0; i < angle_filters_vec.size(); i += 2) {
      if (i + 1 < angle_filters_vec.size()) {
        angle_filters_.emplace_back(AngleFilter{angle_filters_vec[i], angle_filters_vec[i + 1]});
      }
    }

    subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_topic_, 10, std::bind(&CustomPointCloudFilter::pointcloud_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(output_topic_, 10);

    // Create a timer to print the point counts every 5 seconds
    timer_ = this->create_wall_timer(
      std::chrono::seconds(5), std::bind(&CustomPointCloudFilter::print_point_count, this));
  }

private:
  void pointcloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    auto filtered_points = std::make_shared<std::vector<livox_ros_driver2::msg::CustomPoint>>();
    filtered_points->reserve(msg->points.size());

    total_points_received_ += msg->points.size();

    int filtered_in_this_callback = 0;

    for (const auto& point : msg->points) {
      double distance = std::sqrt(point.x * point.x + point.y * point.y);
      if (distance < distance_threshold_ || distance > max_distance_) {
        continue; // Filter points outside of distance thresholds
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
        continue; // Skip points within the XY angle filter
      }

      // Filter points within the robot body area (blue box)
      if (point.x >= filter_area_.min_x && point.x <= filter_area_.max_x &&
          point.y >= filter_area_.min_y && point.y <= filter_area_.max_y) {
        filtered_in_this_callback++;  // Count points filtered by this logic
        continue; // Filter out points inside the robot body area
      }

      filtered_points->push_back(point); // Keep the points that pass all filters
    }

    filtered_by_area_ += filtered_in_this_callback;  // Update total filtered count

    auto output = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
    output->header = msg->header;
    output->timebase = msg->timebase;
    output->point_num = filtered_points->size();
    output->lidar_id = msg->lidar_id;
    output->rsvd = msg->rsvd;
    output->points = *filtered_points;

    publisher_->publish(*output);
  }

  void print_point_count()
  {
    // RCLCPP_INFO(this->get_logger(), "Total points received: %d", total_points_received_);
    // RCLCPP_INFO(this->get_logger(), "Total points filtered by robot body area: %d", filtered_by_area_);

    total_points_received_ = 0;
    filtered_by_area_ = 0;
  }

  std::string input_topic_;
  std::string output_topic_;
  double distance_threshold_;
  double max_distance_;

  AreaFilter filter_area_;  // Specific area filter for the robot body (blue box)
  std::vector<AngleFilter> angle_filters_;  // XY angle filter
  int total_points_received_;   // Track total points received from the LiDAR
  int filtered_by_area_;        // Track points filtered by the robot body area

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer_;  // Timer for printing point counts every 5 seconds
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;

  auto node = std::make_shared<CustomPointCloudFilter>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
