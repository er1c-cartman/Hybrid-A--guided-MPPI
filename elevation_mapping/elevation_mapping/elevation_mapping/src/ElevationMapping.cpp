#define BOOST_BIND_NO_PLACEHOLDERS
/*
 * ElevationMapping.cpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <cmath>
#include <memory>
#include <string>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <kindr/Core>
#include <kindr_ros/kindr_ros.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/ElevationMapping.hpp"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp"

namespace elevation_mapping {

ElevationMapping::ElevationMapping(rclcpp::Node::SharedPtr nodeHandle)
    : nodeHandle_(nodeHandle),
      inputSources_(nodeHandle_),
      robotPoseCacheSize_(10),
      // transformListener_(transformBuffer_),
      map_(nodeHandle),
      robotMotionMapUpdater_(nodeHandle),
      ignoreRobotMotionUpdates_(false),
      updatesEnabled_(true),                    
      maxNoUpdateDuration_(rclcpp::Duration::from_seconds(0.0)),
      timeTolerance_(rclcpp::Duration::from_seconds(100.0)),
      fusedMapPublishTimerDuration_(rclcpp::Duration::from_seconds(0.0)),
      isContinuouslyFusing_(true),
      visibilityCleanupTimerDuration_(rclcpp::Duration::from_seconds(0.0)),
      receivedFirstMatchingPointcloudAndPose_(false),  
      initializeElevationMap_(false),
      initializationMethod_(0),
      lengthInXInitSubmap_(1.2),
      lengthInYInitSubmap_(1.8),
      marginInitSubmap_(0.3),  
      initSubmapHeightOffset_(0.0) {
#ifndef NDEBUG
  // Print a warning if built in debug.
  fusionServiceGroup_ = nodeHandle_->->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  RCLCPP_WARN(nodeHandle_->get_logger(),"CMake Build Type is 'Debug'. Change to 'Release' for better performance.");
#endif

  RCLCPP_INFO(nodeHandle_->get_logger(),"Elevation mapping node started.");

  readParameters();
  setupSubscribers();
  setupServices();
  setupTimers();
  // mapUpdateTimer_ = nodeHandle_->create_wall_timer(
  //       std::chrono::milliseconds(static_cast<int>(maxNoUpdateDuration_.seconds() * 1000)),
  //       std::bind(&ElevationMapping::mapUpdateTimerCallback, this));
  tfBuffer_ = std::make_shared<tf2_ros::Buffer>(nodeHandle_->get_clock());
  transformListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
  initialize();

  RCLCPP_INFO(nodeHandle_->get_logger(),"Successfully launched node.");
}

void ElevationMapping::setupSubscribers() {  // Handle deprecated point_cloud_topic and input_sources configuration.
  
  const bool configuredInputSources = inputSources_.configureFromRos("input_sources");
  const bool hasDeprecatedPointcloudTopic = nodeHandle_->get_parameter("point_cloud_topic",pointCloudTopic_);
  if (hasDeprecatedPointcloudTopic) {
    RCLCPP_WARN(nodeHandle_->get_logger(),"Parameter 'point_cloud_topic' is deprecated, please use 'input_sources' instead.");
  }
  if (!configuredInputSources && hasDeprecatedPointcloudTopic) {
    // pointCloudSubscriber_ = nodeHandle_->create_subscription<sensor_msgs::msg::PointCloud2>(
    // pointCloudTopic_, 1,
    // [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    //     this->pointCloudCallback(msg, true, sensorProcessor_);
    // });
  }
  if (configuredInputSources) {
    inputSources_.registerCallbacks(*this, std::make_pair("pointcloud", &ElevationMapping::pointCloudCallback));
  
  }

  if (!robotPoseTopic_.empty()) {
    // RCLCPP_WARN(nodeHandle_->get_logger(),robotPoseTopic_);
    robotPoseSubscriber_.subscribe(nodeHandle_.get(), robotPoseTopic_, rmw_qos_profile_default);

    // 캐시 설정
    robotPoseCache_.connectInput(robotPoseSubscriber_);
    robotPoseCache_.setCacheSize(robotPoseCacheSize_);
  } else {
    ignoreRobotMotionUpdates_ = true;
  }
}

void ElevationMapping::setupServices() {
  // Multi-threading for fusion.
  fusionTriggerService_ = nodeHandle_->create_service<std_srvs::srv::Empty>(
      "trigger_fusion", std::bind(&ElevationMapping::fuseEntireMapServiceCallback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, fusionServiceGroup_);

  fusedSubmapService_ = nodeHandle_->create_service<grid_map_msgs::srv::GetGridMap>(
      "get_submap", std::bind(&ElevationMapping::getFusedSubmapServiceCallback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, fusionServiceGroup_);

  rawSubmapService_ = nodeHandle_->create_service<grid_map_msgs::srv::GetGridMap>(
      "get_raw_submap", std::bind(&ElevationMapping::getRawSubmapServiceCallback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, fusionServiceGroup_);

  clearMapService_ = nodeHandle_->create_service<std_srvs::srv::Empty>(
      "clear_map", std::bind(&ElevationMapping::clearMapServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

  enableUpdatesService_ = nodeHandle_->create_service<std_srvs::srv::Empty>(
      "enable_updates", std::bind(&ElevationMapping::enableUpdatesServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

  disableUpdatesService_ = nodeHandle_->create_service<std_srvs::srv::Empty>(
      "disable_updates", std::bind(&ElevationMapping::disableUpdatesServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

  maskedReplaceService_ = nodeHandle_->create_service<grid_map_msgs::srv::SetGridMap>(
      "masked_replace", std::bind(&ElevationMapping::maskedReplaceServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

  saveMapService_ = nodeHandle_->create_service<grid_map_msgs::srv::ProcessFile>(
      "save_map", std::bind(&ElevationMapping::saveMapServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

  loadMapService_ = nodeHandle_->create_service<grid_map_msgs::srv::ProcessFile>(
      "load_map", std::bind(&ElevationMapping::loadMapServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

  loadMapService_ = nodeHandle_->create_service<grid_map_msgs::srv::ProcessFile>(
        "load_map", std::bind(&ElevationMapping::loadMapServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
}



ElevationMapping::~ElevationMapping() {
  // Shutdown all services.

  rawSubmapService_.reset();
  fusionTriggerService_.reset();
  fusedSubmapService_.reset();
  fusedMapPublishTimer_.reset();
  visibilityCleanupTimer_.reset();
  nodeHandle_.reset();
}

bool ElevationMapping::readParameters(bool reload) {
  // Using getDataToWrite gets a write-lock on the parameters thereby blocking all reading threads for the whole scope of this
  // readParameters, which is desired.
  double minUpdateRate;
  double timeTolerance;
  double fusedMapPublishingRate;
  double visibilityCleanupRate;
  grid_map::Length length;
  grid_map::Position position;
  double resolution;
  nodeHandle_->declare_parameter("enable_circular_clear", true);
  nodeHandle_->declare_parameter("circular_clear_radius", 1.0);
  nodeHandle_->declare_parameter("circular_clear_frequency", 0.2);

  nodeHandle_->get_parameter("enable_circular_clear", enableCircularClear_);
  nodeHandle_->get_parameter("circular_clear_radius", circularClearRadius_);
  nodeHandle_->get_parameter("circular_clear_frequency", circularClearFrequency_);

  nodeHandle_->declare_parameter("robot_base_frame_id", std::string("/base_link"));
  nodeHandle_->declare_parameter("point_cloud_topic","");
  nodeHandle_->declare_parameter("robot_pose_with_covariance_topic", std::string("/base_link_pose"));
  nodeHandle_->declare_parameter("track_point_frame_id", "base_link");
  nodeHandle_->declare_parameter("track_point_x", 0.0);
  nodeHandle_->declare_parameter("track_point_y", 0.0);
  nodeHandle_->declare_parameter("track_point_z", -0.05);
  nodeHandle_->declare_parameter("robot_pose_cache_size", 50);
  nodeHandle_->declare_parameter("min_update_rate", 2.0);
  nodeHandle_->declare_parameter("time_tolerance", 0.0);
  nodeHandle_->declare_parameter("fused_map_publishing_rate", 1.0);
  nodeHandle_->declare_parameter("visibility_cleanup_rate", 1.0);
  nodeHandle_->declare_parameter("map_frame_id", "odom");
  nodeHandle_->declare_parameter("length_in_x", 12.0);
  nodeHandle_->declare_parameter("length_in_y", 12.0);
  nodeHandle_->declare_parameter("position_x", 0.0);
  nodeHandle_->declare_parameter("position_y", 0.0);
  nodeHandle_->declare_parameter("resolution", 0.1);
  nodeHandle_->declare_parameter("min_variance", pow(0.003, 2));
  nodeHandle_->declare_parameter("max_variance", pow(0.03, 2));
  nodeHandle_->declare_parameter("mahalanobis_distance_threshold", 2.5);
  nodeHandle_->declare_parameter("multi_height_noise", pow(0.003, 2));
  nodeHandle_->declare_parameter("min_horizontal_variance", pow(0.01 / 2.0, 2));
  nodeHandle_->declare_parameter("max_horizontal_variance", 0.5);
  nodeHandle_->declare_parameter("underlying_map_topic", std::string());
  nodeHandle_->declare_parameter("enable_visibility_cleanup", true);
  nodeHandle_->declare_parameter("enable_continuous_cleanup", false);
  nodeHandle_->declare_parameter("scanning_duration", 1.0);
  nodeHandle_->declare_parameter("increase_height_alpha", 0.0);
  nodeHandle_->declare_parameter("masked_replace_service_mask_layer_name", std::string("mask"));
  nodeHandle_->declare_parameter("initialize_elevation_map", false);
  nodeHandle_->declare_parameter("initialization_method", 0);
  nodeHandle_->declare_parameter("length_in_x_init_submap", 1.2);
  nodeHandle_->declare_parameter("length_in_y_init_submap", 1.8);
  nodeHandle_->declare_parameter("init_submap_height_offset", 0.0);
  nodeHandle_->declare_parameter("init_submap_variance", 0.01);
  nodeHandle_->declare_parameter("target_frame_init_submap", "base_link");
  nodeHandle_->declare_parameter("sensor_processor/type", std::string("laser"));

  nodeHandle_->get_parameter("point_cloud_topic", pointCloudTopic_);
  nodeHandle_->get_parameter("robot_pose_with_covariance_topic", robotPoseTopic_);
  nodeHandle_->get_parameter("track_point_frame_id", trackPointFrameId_);
  nodeHandle_->get_parameter("track_point_x", trackPoint_.x());
  nodeHandle_->get_parameter("track_point_y", trackPoint_.y());
  nodeHandle_->get_parameter("track_point_z", trackPoint_.z());
  nodeHandle_->get_parameter("robot_pose_cache_size", robotPoseCacheSize_);
  nodeHandle_->get_parameter("min_update_rate", minUpdateRate);
  nodeHandle_->get_parameter("time_tolerance", timeTolerance);
  nodeHandle_->get_parameter("fused_map_publishing_rate", fusedMapPublishingRate);
  nodeHandle_->get_parameter("visibility_cleanup_rate", visibilityCleanupRate);
  nodeHandle_->get_parameter("map_frame_id", mapFrameId_);
  nodeHandle_->get_parameter("length_in_x", length(0));
  nodeHandle_->get_parameter("length_in_y", length(1));
  nodeHandle_->get_parameter("position_x", position.x());
  nodeHandle_->get_parameter("position_y", position.y());
  nodeHandle_->get_parameter("resolution", resolution);
  nodeHandle_->get_parameter("min_variance", map_.minVariance_);
  nodeHandle_->get_parameter("max_variance", map_.maxVariance_);
  nodeHandle_->get_parameter("mahalanobis_distance_threshold", map_.mahalanobisDistanceThreshold_);
  nodeHandle_->get_parameter("multi_height_noise", map_.multiHeightNoise_);
  nodeHandle_->get_parameter("min_horizontal_variance", map_.minHorizontalVariance_);  // two-sigma
  nodeHandle_->get_parameter("max_horizontal_variance", map_.maxHorizontalVariance_);
  nodeHandle_->get_parameter("underlying_map_topic", map_.underlyingMapTopic_);
  nodeHandle_->get_parameter("enable_visibility_cleanup", map_.enableVisibilityCleanup_);
  nodeHandle_->get_parameter("enable_continuous_cleanup", map_.enableContinuousCleanup_);
  nodeHandle_->get_parameter("scanning_duration", map_.scanningDuration_);
  nodeHandle_->get_parameter("masked_replace_service_mask_layer_name", maskedReplaceServiceMaskLayerName_);
  nodeHandle_->declare_parameter("sensor_processor/min_radius",  0.018);
  nodeHandle_->declare_parameter("sensor_processor/beam_angle",  0.0006);
  nodeHandle_->declare_parameter("sensor_processor/beam_constant", 0.0015);
  nodeHandle_->get_parameter("initialize_elevation_map", initializeElevationMap_);
  nodeHandle_->get_parameter("initialization_method", initializationMethod_);
  nodeHandle_->get_parameter("length_in_x_init_submap", lengthInXInitSubmap_);
  nodeHandle_->get_parameter("length_in_y_init_submap", lengthInYInitSubmap_);
  nodeHandle_->get_parameter("init_submap_height_offset", initSubmapHeightOffset_);
  nodeHandle_->get_parameter("target_frame_init_submap", targetFrameInitSubmap_);


  RCLCPP_EXPORT(nodeHandle_->get_logger(),robotPoseCacheSize_ >= 0);

  if (minUpdateRate == 0.0) {
    maxNoUpdateDuration_= rclcpp::Duration::from_seconds(0.0);
    RCLCPP_WARN(nodeHandle_->get_logger(),"Rate for publishing the map is zero.");
  } else {
    maxNoUpdateDuration_= rclcpp::Duration::from_seconds(1.0 / minUpdateRate);
  }
   assert(maxNoUpdateDuration_.seconds() != 0.0);

  timeTolerance_= rclcpp::Duration::from_seconds(timeTolerance);

  if (fusedMapPublishingRate == 0.0) {
    fusedMapPublishTimerDuration_= rclcpp::Duration::from_seconds(0.0);
    RCLCPP_WARN(nodeHandle_->get_logger(),
        "Rate for publishing the fused map is zero. The fused elevation map will not be published unless the service `triggerFusion` is "
        "called.");
  } else if (std::isinf(fusedMapPublishingRate)) {
    isContinuouslyFusing_ = true;
    fusedMapPublishTimerDuration_= rclcpp::Duration::from_seconds(0.0);
  } else {
    fusedMapPublishTimerDuration_= rclcpp::Duration::from_seconds(1.0 / fusedMapPublishingRate);
  }

  if (visibilityCleanupRate == 0.0) {
    visibilityCleanupTimerDuration_= rclcpp::Duration::from_seconds(0.0);
    RCLCPP_WARN(nodeHandle_->get_logger(),"Rate for visibility cleanup is zero and therefore disabled.");
  } else {
    visibilityCleanupTimerDuration_= rclcpp::Duration::from_seconds(1.0 / visibilityCleanupRate);
    map_.visibilityCleanupDuration_ = 1.0 / visibilityCleanupRate;
  }

  // ElevationMap  TODO Move this to the elevation map class.


  if (!reload) {
    map_.setFrameId("odom");
    map_.setGeometry(length, resolution, position);
  }


  // SensorProcessor  Deprecated, use the sensorProcessor from within input sources instead!
  std::string sensorType;
  nodeHandle_->get_parameter("sensor_processor/type", sensorType);
  SensorProcessorBase::GeneralParameters generalSensorProcessorConfig{nodeHandle_->get_parameter("robot_base_frame_id").as_string(), mapFrameId_};
  if (sensorType == "structured_light") {
    sensorProcessor_ = std::make_shared<StructuredLightSensorProcessor>(nodeHandle_, generalSensorProcessorConfig);
  } else if (sensorType == "stereo") {
    sensorProcessor_ = std::make_shared<StereoSensorProcessor>(nodeHandle_, generalSensorProcessorConfig);
  } else if (sensorType == "laser") {
    sensorProcessor_ = std::make_shared<LaserSensorProcessor>(nodeHandle_, generalSensorProcessorConfig);
  } else if (sensorType == "perfect") {
    sensorProcessor_ = std::make_shared<PerfectSensorProcessor>(nodeHandle_, generalSensorProcessorConfig);
  } else {
    RCLCPP_ERROR(nodeHandle_->get_logger(),"The sensor type %s is not available.", sensorType.c_str());
  }
  if (!sensorProcessor_->readParameters()) {
    return false;
  }
  if (!robotMotionMapUpdater_.readParameters()) {
    return false;
  }

  return true;
}


// ... other includes and code ...

void ElevationMapping::circularClearCallback() {
  if (!enableCircularClear_) {
    return;
  }

  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer_->lookupTransform(map_.getFrameId(), "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Could not transform base_link to %s: %s", map_.getFrameId().c_str(), ex.what());
    return;
  }

  grid_map::Position center(transformStamped.transform.translation.x, transformStamped.transform.translation.y);

  std::unique_lock<std::recursive_mutex> lock(map_.getRawDataMutex());

  for (grid_map::CircleIterator iterator(map_.getRawGridMap(), center, circularClearRadius_); !iterator.isPastEnd(); ++iterator) {
    for (const auto& layer : map_.getRawGridMap().getLayers()) {
      if (map_.getRawGridMap().exists(layer)) {
        map_.getRawGridMap().at(layer, *iterator) = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }
  map_.postprocessAndPublishRawElevationMap();
}

void ElevationMapping::setupTimers() {
  if (!(fusedMapPublishTimerDuration_.nanoseconds() == 0)) {
    fusedMapPublishTimer_ = nodeHandle_->create_wall_timer(
      std::chrono::nanoseconds(fusedMapPublishTimerDuration_.nanoseconds()),
      std::bind(&ElevationMapping::publishFusedMapCallback, this),
      fusionServiceGroup_
    );
  }

  if (map_.enableVisibilityCleanup_ && !(visibilityCleanupTimerDuration_.nanoseconds() == 0) && !map_.enableContinuousCleanup_) {
    visibilityCleanupTimer_ = nodeHandle_->create_wall_timer(
      std::chrono::nanoseconds(visibilityCleanupTimerDuration_.nanoseconds()),
      std::bind(&ElevationMapping::visibilityCleanupCallback, this)
    );
  }

  mapUpdateTimer_ = nodeHandle_->create_wall_timer(
    std::chrono::nanoseconds(maxNoUpdateDuration_.nanoseconds()),
    std::bind(&ElevationMapping::mapUpdateTimerCallback, this)
  );

  if (enableCircularClear_) {
    auto circularClearDuration = std::chrono::milliseconds(static_cast<int>(1000 / circularClearFrequency_));
    circularClearTimer_ = nodeHandle_->create_wall_timer(
      circularClearDuration,
      std::bind(&ElevationMapping::circularClearCallback, this)
    );
  }
}

// ... other code ...





bool ElevationMapping::initialize() {
  RCLCPP_INFO(nodeHandle_->get_logger(),"Elevation mapping node initializing ... ");
  fusionServiceThread_ = std::thread(&ElevationMapping::runFusionServiceThread, this);
  rclcpp::sleep_for(std::chrono::seconds(1));  // Need this to get the TF caches fill up.
  //resetMapUpdateTimer();
  // fusedMapPublishTimer_->reset();
  visibilityCleanupThread_ = std::thread([this] { visibilityCleanupThread(); });
  // visibilityCleanupTimer_->reset();
  initializeElevationMap();
  return true;
}

void ElevationMapping::runFusionServiceThread() {
  // rclcpp::Rate rate(1);  // Set the rate to 1 Hz
  // while (rclcpp::ok()) {
    // // Call the first service
    // auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
    // if (callService<std_srvs::srv::Empty>("trigger_fusion", empty_request)) {
        // RCLCPP_INFO(nodeHandle_->get_logger(), "Fusion service succeeded");

        // // Call the second service after the first succeeds
        // auto submap_request = std::make_shared<grid_map_msgs::srv::GetGridMap::Request>();
        // // request->position.x = 
        // // request->position.y = 
        // // request->length.x = 
        // // request->length.y = 
        
        // if (callService<grid_map_msgs::srv::GetGridMap>("get_submap", submap_request)) {
            // RCLCPP_INFO(nodeHandle_->get_logger(), "Get fused submap service succeeded");

            // // Call the third service after the second succeeds
            // if (callService<grid_map_msgs::srv::GetGridMap>("get_raw_submap", submap_request)) {
                // RCLCPP_INFO(nodeHandle_->get_logger(), "Get raw submap service succeeded");
            // }
        // }
    // }

    // rate.sleep(); // Call services every second
// }
}

void ElevationMapping::visibilityCleanupThread() {
  // rclcpp::executors::MultiThreadedExecutor executor;
  //   executor.add_node(nodeHandle_);
  //   executor.spin_some();
  //   executor.remove_node(nodeHandle_);
}

void ElevationMapping::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg, bool publishPointCloud,
                                           const std::shared_ptr<SensorProcessorBase>& sensorProcessor_) {
  RCLCPP_DEBUG(nodeHandle_->get_logger(),"Processing data from: %s", pointCloudMsg->header.frame_id.c_str());
  if (!updatesEnabled_) {
    RCLCPP_WARN_THROTTLE(nodeHandle_->get_logger(),*nodeHandle_->get_clock(),10, "Updating of elevation map is disabled. (Warning message is throttled, 10s.)");
    if (publishPointCloud) {
      map_.setTimestamp(nodeHandle_->get_clock()->now());
      map_.postprocessAndPublishRawElevationMap();
    }
    return;
  }
  double cache_size = robotPoseCache_.getOldestTime().seconds();
  double lase = robotPoseCache_.getLatestTime().seconds();
  // Check if point cloud has corresponding robot pose at the beginning
  if (!receivedFirstMatchingPointcloudAndPose_) {
    const double oldestPoseTime = robotPoseCache_.getOldestTime().seconds();
    const double currentPointCloudTime = rclcpp::Time(pointCloudMsg->header.stamp, nodeHandle_->get_clock()->get_clock_type()).seconds();
    if (currentPointCloudTime < oldestPoseTime) {
      RCLCPP_WARN_THROTTLE(nodeHandle_->get_logger(),*nodeHandle_->get_clock(),5, "No corresponding point cloud and pose are found. Waiting for first match. (Warning message is throttled, 5s.)");
      return;
    } else {
      RCLCPP_DEBUG(nodeHandle_->get_logger(),"First corresponding point cloud and pose found, elevation mapping started. ");
      receivedFirstMatchingPointcloudAndPose_ = true;
    }
  }

  stopMapUpdateTimer();

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud.
  // TODO(max): Double check with http://wiki.ros.org/hydro/Migration
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*pointCloudMsg, pcl_pc);

  PointCloudType::Ptr pointCloud(new PointCloudType);
  pcl::fromPCLPointCloud2(pcl_pc, *pointCloud);
  lastPointCloudUpdateTime_ = rclcpp::Time(pointCloudMsg->header.stamp, nodeHandle_->get_clock()->get_clock_type());
  RCLCPP_DEBUG(nodeHandle_->get_logger(),"ElevationMap received a point cloud (%i points) for elevation mapping.", static_cast<int>(pointCloud->size()));

  // Get robot pose covariance matrix at timestamp of point cloud.
  Eigen::Matrix<double, 6, 6> robotPoseCovariance;
  robotPoseCovariance.setZero();
  // RCLCPP_INFO(nodeHandle_->get_logger(),"This1?");
  if (!ignoreRobotMotionUpdates_) {
      // RCLCPP_INFO(nodeHandle_->get_logger(),"This2?");

    std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped > poseMessage =
        robotPoseCache_.getElemBeforeTime(lastPointCloudUpdateTime_);
    if (!poseMessage) {
        // RCLCPP_INFO(nodeHandle_->get_logger(),"This3?");

      // Tell the user that either for the timestamp no pose is available or that the buffer is possibly empty
      if (robotPoseCache_.getOldestTime().seconds() > lastPointCloudUpdateTime_.seconds()) {
        RCLCPP_ERROR(nodeHandle_->get_logger(),"The oldest pose available is at %f, requested pose at %f", robotPoseCache_.getOldestTime().seconds(),
                  lastPointCloudUpdateTime_.seconds());
      } else {
        RCLCPP_ERROR(nodeHandle_->get_logger(),"Could not get pose information from robot for time %f. Buffer empty?", lastPointCloudUpdateTime_.seconds());
      }
      return;
    }
    robotPoseCovariance = Eigen::Map<const Eigen::MatrixXd>(poseMessage->pose.covariance.data(), 6, 6);
  }

  // Process point cloud.
  PointCloudType::Ptr pointCloudProcessed(new PointCloudType);
  Eigen::VectorXf measurementVariances;
   sensorProcessor_->process(pointCloud, robotPoseCovariance, pointCloudProcessed, measurementVariances,
                                 pointCloudMsg->header.frame_id);

  if (!sensorProcessor_->process(pointCloud, robotPoseCovariance, pointCloudProcessed, measurementVariances,
                                 std::string(pointCloudMsg->header.frame_id))) {         

    if (!sensorProcessor_->isTfAvailableInBuffer()) {

      rclcpp::Clock clock;
      RCLCPP_INFO_THROTTLE(nodeHandle_->get_logger(), clock, 10, "Waiting for tf transformation to be available. (Message is throttled, 10s.)");
      return;
    }
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Point cloud could not be processed."); //TODO: what causes this issue
    resetMapUpdateTimer();
    return;
  }

  std::unique_lock<std::recursive_mutex>scopedLock(map_.getRawDataMutex());

  // Update map location.
  updateMapLocation();

  // Update map from motion prediction.
  if (!updatePrediction(lastPointCloudUpdateTime_)) {
    RCLCPP_ERROR(nodeHandle_->get_logger(),"Updating process noise failed.");
    resetMapUpdateTimer();
    return;
  }

  // Clear the map if continuous clean-up was enabled.
  if (map_.enableContinuousCleanup_) {
    RCLCPP_DEBUG(nodeHandle_->get_logger(),"Clearing elevation map before adding new point cloud.");
    map_.clear();
  }

  // Add point cloud to elevation map.
  if (!map_.add(pointCloudProcessed, measurementVariances, lastPointCloudUpdateTime_,
                Eigen::Affine3d(sensorProcessor_->transformationSensorToMap_))) {
    RCLCPP_ERROR(nodeHandle_->get_logger(),"Adding point cloud to elevation map failed.");
    resetMapUpdateTimer();
    return;
  }

  if (publishPointCloud) {
    // Publish elevation map.
    map_.postprocessAndPublishRawElevationMap();
    if (isFusingEnabled()) {
      map_.fuseAll();
      map_.publishFusedElevationMap();
    }
  }

  resetMapUpdateTimer();
}

void ElevationMapping::mapUpdateTimerCallback() {
  if (!updatesEnabled_) {
    rclcpp::Clock clock;
    RCLCPP_WARN_THROTTLE(nodeHandle_->get_logger(), clock, 10, "Updating of elevation map is disabled. (Warning message is throttled, 10s.)");
    map_.setTimestamp(nodeHandle_->get_clock()->now());
    map_.postprocessAndPublishRawElevationMap();
    return;
  }

  rclcpp::Time time = nodeHandle_->get_clock()->now();
  // if ((lastPointCloudUpdateTime_ - time) <= maxNoUpdateDuration_) {  // there were updates from sensordata, no need to force an update.
  //   return;
  // }
  rclcpp::Clock clock;
  //RCLCPP_WARN_THROTTLE(nodeHandle_->get_logger(), clock, 5, "Elevation map is updated without data from the sensor. (Warning message is throttled, 5s.)");

  std::unique_lock<std::recursive_mutex> scopedLock(map_.getRawDataMutex());

  stopMapUpdateTimer();

  // Update map from motion prediction.
  if (!updatePrediction(time)) {
    RCLCPP_ERROR(nodeHandle_->get_logger(),"Updating process noise failed.");
    resetMapUpdateTimer();
    return;
  }

  // Publish elevation map.
  map_.postprocessAndPublishRawElevationMap();
  if (isFusingEnabled()) {
    map_.fuseAll();
    map_.publishFusedElevationMap();
  }

  resetMapUpdateTimer();
}

void ElevationMapping::publishFusedMapCallback() {
  if (!map_.hasFusedMapSubscribers()) {
    return;
  }
  RCLCPP_DEBUG(nodeHandle_->get_logger(),"Elevation map is fused and published from timer.");
  std::unique_lock<std::recursive_mutex>scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  map_.publishFusedElevationMap();
}

void ElevationMapping::visibilityCleanupCallback() {
  RCLCPP_DEBUG(nodeHandle_->get_logger(),"Elevation map is running visibility cleanup.");
  // Copy constructors for thread-safety.
  map_.visibilityCleanup(nodeHandle_->get_clock()->now());
}

bool ElevationMapping::fuseEntireMapServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  std::unique_lock<std::recursive_mutex> scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  map_.publishFusedElevationMap();
  return true;
}

bool ElevationMapping::isFusingEnabled() {
  return isContinuouslyFusing_ && map_.hasFusedMapSubscribers();
}

bool ElevationMapping::updatePrediction(const rclcpp::Time& time) {
  if (ignoreRobotMotionUpdates_) {
    return true;
  }

  RCLCPP_DEBUG(nodeHandle_->get_logger(),"Updating map with latest prediction from time %f.", robotPoseCache_.getLatestTime().seconds());

 // if (time + timeTolerance_ < map_.getTimeOfLastUpdate()) {
 //   RCLCPP_ERROR(nodeHandle_->get_logger(),"THis?");
 //   RCLCPP_ERROR(nodeHandle_->get_logger(),"Requested update with time stamp %f, but time of last update was %f.", time.seconds(), map_.getTimeOfLastUpdate().seconds());
 //   return false;
 // } else if (time < map_.getTimeOfLastUpdate()) {
 //   RCLCPP_DEBUG(nodeHandle_->get_logger(),"Requested update with time stamp %f, but time of last update was %f. Ignoring update.", time.seconds(),
 //             map_.getTimeOfLastUpdate().seconds());
 //   return true;
 // }

  // Get robot pose at requested time.
  std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped const> poseMessage = robotPoseCache_.getElemBeforeTime(time);
  if (!poseMessage) {
    // Tell the user that either for the timestamp no pose is available or that the buffer is possibly empty
    if (robotPoseCache_.getOldestTime().seconds() > lastPointCloudUpdateTime_.seconds()) {
      RCLCPP_ERROR(nodeHandle_->get_logger(),"The oldest pose available is at %f, requested pose at %f", robotPoseCache_.getOldestTime().seconds(),
                lastPointCloudUpdateTime_.seconds());
    } else {
      RCLCPP_ERROR(nodeHandle_->get_logger(),"Could not get pose information from robot for time %f. Buffer empty?", lastPointCloudUpdateTime_.seconds());
    }
    return false;
  }

  kindr::HomTransformQuatD robotPose;
  kindr_ros::convertFromRosGeometryMsg(poseMessage->pose.pose, robotPose);
  // Covariance is stored in row-major in ROS: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovariance.html
  Eigen::Matrix<double, 6, 6> robotPoseCovariance =
      Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(poseMessage->pose.covariance.data(), 6, 6);

  // Compute map variance update from motion prediction.
  robotMotionMapUpdater_.update(map_, robotPose, robotPoseCovariance, time);

  return true;
}

bool ElevationMapping::updateMapLocation() {
  RCLCPP_DEBUG(nodeHandle_->get_logger(),"Elevation map is checked for relocalization.");

  geometry_msgs::msg::PointStamped trackPoint;
  trackPoint.header.frame_id = trackPointFrameId_;
  trackPoint.header.stamp = rclcpp::Time(0, 0, nodeHandle_->get_clock()->get_clock_type());
  kindr_ros::convertToRosGeometryMsg(trackPoint_, trackPoint.point);
  geometry_msgs::msg::PointStamped trackPointTransformed;

  try {

    geometry_msgs::msg::TransformStamped transformStamped = tfBuffer_->lookupTransform(
        map_.getFrameId(), "base_link", tf2::TimePointZero);
    tf2::doTransform(trackPoint, trackPointTransformed, transformStamped);
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(nodeHandle_->get_logger(),"%s", ex.what());
    return false;
  }
  kindr::Position3D position3d;
  kindr_ros::convertFromRosGeometryMsg(trackPointTransformed.point, position3d);
  grid_map::Position position = position3d.vector().head(2);
  map_.move(position);
  return true;
}

bool ElevationMapping::getFusedSubmapServiceCallback(const std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> request, std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response) {
  grid_map::Position requestedSubmapPosition(request->position_x, request->position_y);
  grid_map::Length requestedSubmapLength(request->length_x, request->length_y);
  RCLCPP_INFO(nodeHandle_->get_logger(),"Elevation submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(), requestedSubmapPosition.y(),
            requestedSubmapLength(0), requestedSubmapLength(1));
  std::unique_lock<std::recursive_mutex>scopedLock(map_.getFusedDataMutex());
  RCLCPP_ERROR(nodeHandle_->get_logger(),"I Think here");
  map_.fuseArea(requestedSubmapPosition, requestedSubmapLength);

  bool isSuccess{false};
  grid_map::Index index;
  grid_map::GridMap subMap = map_.getFusedGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, isSuccess);
  scopedLock.unlock();
  std::unique_ptr<grid_map_msgs::msg::GridMap> message;
  
  if (request->layers.empty()) {
    message = grid_map::GridMapRosConverter::toMessage(subMap);
    response->map = *message;
  } else {
    std::vector<std::string> layers;
    for (const std::string& layer : request->layers) {
      layers.push_back(layer);

    }
    message = grid_map::GridMapRosConverter::toMessage(subMap, layers);
    if (!message) {
      RCLCPP_ERROR(nodeHandle_->get_logger(), "Failed to convert submap with requested layers to message.");
      return false;
    }

    response->map = *message;
  }

  RCLCPP_INFO(nodeHandle_->get_logger(),"Elevation submap responded with timestamp %f.", map_.getTimeOfLastFusion().seconds());
  return isSuccess;
}

bool ElevationMapping::getRawSubmapServiceCallback(const std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> request, std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response) {
  grid_map::Position requestedSubmapPosition(request->position_x, request->position_y);
  grid_map::Length requestedSubmapLength(request->length_x, request->length_y);
  RCLCPP_INFO(nodeHandle_->get_logger(),"Elevation raw submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(),
            requestedSubmapPosition.y(), requestedSubmapLength(0), requestedSubmapLength(1));
  std::unique_lock<std::recursive_mutex>scopedLock(map_.getRawDataMutex());

  bool isSuccess{false};
  grid_map::Index index;
  grid_map::GridMap subMap = map_.getRawGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, isSuccess);
  scopedLock.unlock();
  std::unique_ptr<grid_map_msgs::msg::GridMap> temp;
  if (request->layers.empty()) {
    temp = grid_map::GridMapRosConverter::toMessage(subMap);
    response->map = *temp;
  } else {
    std::vector<std::string> layers;
    for (const std::string& layer : request->layers) {
      layers.push_back(layer);
    }
    temp = grid_map::GridMapRosConverter::toMessage(subMap, layers);
    response->map = *temp;
  }
  return isSuccess;
}

bool ElevationMapping::disableUpdatesServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  RCLCPP_INFO(nodeHandle_->get_logger(),"Disabling updates.");
  updatesEnabled_ = false;
  return true;
}

bool ElevationMapping::enableUpdatesServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response){
  RCLCPP_INFO(nodeHandle_->get_logger(),"Enabling updates.");
  updatesEnabled_ = true;
  return true;
}

bool ElevationMapping::initializeElevationMap() {
  if (initializeElevationMap_) {
    if (static_cast<elevation_mapping::InitializationMethods>(initializationMethod_) ==
        elevation_mapping::InitializationMethods::PlanarFloorInitializer) {
      tf2::Stamped<tf2::Transform> transform;
      geometry_msgs::msg::TransformStamped transform_msg;

      // Listen to transform between mapFrameId_ and targetFrameInitSubmap_ and use z value for initialization
      try {

        transform_msg = tfBuffer_->lookupTransform(mapFrameId_, targetFrameInitSubmap_, rclcpp::Time(0, 0, nodeHandle_->get_clock()->get_clock_type()), rclcpp::Duration::from_seconds(1.0));
        tf2::fromMsg(transform_msg, transform);

        RCLCPP_INFO_STREAM(nodeHandle_->get_logger(), "Initializing with x: " << transform.getOrigin().x() << " y: " << transform.getOrigin().y()
                                                 << " z: " << transform.getOrigin().z());

        const grid_map::Position positionRobot(transform.getOrigin().x(), transform.getOrigin().y());

        // Move map before we apply the height values. This prevents unwanted behavior from intermediate move() calls in
        // updateMapLocation().
        map_.move(positionRobot);

        map_.setRawSubmapHeight(positionRobot, transform.getOrigin().z() + initSubmapHeightOffset_,
                                lengthInXInitSubmap_, lengthInYInitSubmap_, marginInitSubmap_);
        return true;
      } catch (tf2::TransformException& ex) {
        RCLCPP_DEBUG(nodeHandle_->get_logger(),"%s", ex.what());
        RCLCPP_WARN(nodeHandle_->get_logger(),"Could not initialize elevation map with constant height. (This warning can be ignored if TF tree is not available.)");
        return false;
      }
    }
  }
  return true;
}

bool ElevationMapping::clearMapServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  RCLCPP_INFO(nodeHandle_->get_logger(),"Clearing map...");
  bool success = map_.clear();
  success &= initializeElevationMap();
  RCLCPP_INFO(nodeHandle_->get_logger(),"Map cleared.");

  return success;
}

bool ElevationMapping::maskedReplaceServiceCallback(const std::shared_ptr<grid_map_msgs::srv::SetGridMap::Request> request, std::shared_ptr<grid_map_msgs::srv::SetGridMap::Response> response) {
  RCLCPP_INFO(nodeHandle_->get_logger(),"Masked replacing of map.");
  grid_map::GridMap sourceMap;
  grid_map::GridMapRosConverter::fromMessage(request->map, sourceMap);

  // Use the supplied mask or do not use a mask
  grid_map::Matrix mask;
  if (sourceMap.exists(maskedReplaceServiceMaskLayerName_)) {
    mask = sourceMap[maskedReplaceServiceMaskLayerName_];
  } else {
    mask = Eigen::MatrixXf::Ones(sourceMap.getSize()(0), sourceMap.getSize()(1));
  }

  std::unique_lock<std::recursive_mutex>scopedLockRawData(map_.getRawDataMutex());

  // Loop over all layers that should be set
  for (auto sourceLayerIterator = sourceMap.getLayers().begin(); sourceLayerIterator != sourceMap.getLayers().end();
       sourceLayerIterator++) {
    // skip "mask" layer
    if (*sourceLayerIterator == maskedReplaceServiceMaskLayerName_) {
      continue;
    }
    grid_map::Matrix& sourceLayer = sourceMap[*sourceLayerIterator];
    // Check if the layer exists in the elevation map
    if (map_.getRawGridMap().exists(*sourceLayerIterator)) {
      grid_map::Matrix& destinationLayer = map_.getRawGridMap()[*sourceLayerIterator];
      for (grid_map::GridMapIterator destinationIterator(map_.getRawGridMap()); !destinationIterator.isPastEnd(); ++destinationIterator) {
        // Use the position to find corresponding indices in source and destination
        const grid_map::Index destinationIndex(*destinationIterator);
        grid_map::Position position;
        map_.getRawGridMap().getPosition(*destinationIterator, position);

        if (!sourceMap.isInside(position)) {
          continue;
        }

        grid_map::Index sourceIndex;
        sourceMap.getIndex(position, sourceIndex);
        // If the mask allows it, set the value from source to destination
        if (!std::isnan(mask(sourceIndex(0), sourceIndex(1)))) {
          destinationLayer(destinationIndex(0), destinationIndex(1)) = sourceLayer(sourceIndex(0), sourceIndex(1));
        }
      }
    } else {
      RCLCPP_ERROR(nodeHandle_->get_logger(),"Masked replace service: Layer %s does not exist!", sourceLayerIterator->c_str());
    }
  }

  return true;
}

bool ElevationMapping::saveMapServiceCallback(const std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request, std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response) {
  RCLCPP_INFO(nodeHandle_->get_logger(),"Saving map to file.");
  std::unique_lock<std::recursive_mutex>scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  std::string topic = std::string(nodeHandle_->get_namespace()) + "/elevation_map";
  if (!request->topic_name.empty()) {
    topic = std::string(nodeHandle_->get_namespace()) + "/" + request->topic_name;
  }
  response->success = static_cast<unsigned char>(grid_map::GridMapRosConverter::saveToBag(map_.getFusedGridMap(), request->file_path, topic));
  response->success = static_cast<unsigned char>(
      (grid_map::GridMapRosConverter::saveToBag(map_.getRawGridMap(), request->file_path + "_raw", topic + "_raw")) &&
      static_cast<bool>(response->success));
  return static_cast<bool>(response->success);
}

bool ElevationMapping::loadMapServiceCallback(const std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request, std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response) {
  RCLCPP_WARN(nodeHandle_->get_logger(),"Loading from bag file.");
  std::unique_lock<std::recursive_mutex>scopedLockFused(map_.getFusedDataMutex());
  std::unique_lock<std::recursive_mutex>scopedLockRaw(map_.getRawDataMutex());

  std::string topic = std::string(nodeHandle_->get_namespace()) + "/elevation_map";
  if (!request->topic_name.empty()) {
    topic += "/" + request->topic_name;
  } else {
    topic += "/elevation_map";
  }

  response->success =
      static_cast<unsigned char>(grid_map::GridMapRosConverter::loadFromBag(request->file_path, topic, map_.getFusedGridMap()));
  response->success = static_cast<unsigned char>(
      grid_map::GridMapRosConverter::loadFromBag(request->file_path + "_raw", topic + "_raw", map_.getRawGridMap()) &&
      static_cast<bool>(response->success));

  // Update timestamp for visualization in ROS
  map_.setTimestamp(nodeHandle_->get_clock()->now());
  map_.postprocessAndPublishRawElevationMap();
  return static_cast<bool>(response->success);
}



void ElevationMapping::resetMapUpdateTimer() {
    mapUpdateTimer_->cancel();
    rclcpp::Duration periodSinceLastUpdate = nodeHandle_->get_clock()->now() - map_.getTimeOfLastUpdate();
  if (periodSinceLastUpdate > maxNoUpdateDuration_) {
    periodSinceLastUpdate= rclcpp::Duration::from_seconds(0.0);
  }
  mapUpdateTimer_->reset();
  auto remainingDurationChrono = std::chrono::nanoseconds((maxNoUpdateDuration_ - periodSinceLastUpdate).nanoseconds());
  mapUpdateTimer_ = nodeHandle_->create_wall_timer(remainingDurationChrono, std::bind(&ElevationMapping::mapUpdateTimerCallback, this));
}

void ElevationMapping::stopMapUpdateTimer() {
  mapUpdateTimer_->cancel();
}

}  // namespace elevation_mapping
