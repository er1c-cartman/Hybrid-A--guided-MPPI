/*
 * PostprocessingPipelineFunctor.cpp
 *
 *  Created on: Sep. 14, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 *
 *  Note. Large parts are adopted from grid_map_demos/FiltersDemo.cpp.
 */

#include <grid_map_ros/grid_map_ros.hpp>

#include "elevation_mapping/postprocessing/PostprocessingPipelineFunctor.hpp"

namespace elevation_mapping {

PostprocessingPipelineFunctor::PostprocessingPipelineFunctor(rclcpp::Node::SharedPtr nodeHandle)
    : nodeHandle_(nodeHandle), filterChain_("grid_map::GridMap"), filterChainConfigured_(false) {
  // TODO (magnus) Add logic when setting up failed. What happens actually if it is not configured?
  readParameters();
  const Parameters parameters{parameters_.getData()};
  publisher_ = nodeHandle_->create_publisher<grid_map_msgs::msg::GridMap>(parameters.outputTopic_, 1);

  // Setup filter chain.
  if (!filterChain_.configure(parameters.filterChainParametersName_,
                              nodeHandle_->get_node_logging_interface(),
                              nodeHandle_->get_node_parameters_interface())) {
    RCLCPP_WARN(nodeHandle_->get_logger(), "Could not configure the filter chain. Will publish the raw elevation map without postprocessing!");
    return;
  }
  filterChainConfigured_ = true;
}

// 소멸자 정의
PostprocessingPipelineFunctor::~PostprocessingPipelineFunctor() = default;

void PostprocessingPipelineFunctor::readParameters() {
  Parameters parameters;
  nodeHandle_->declare_parameter<std::string>("output_topic", "elevation_map_raw");
  nodeHandle_->declare_parameter<std::string>("postprocessor_pipeline_name", "postprocessor_pipeline");

  nodeHandle_->get_parameter("output_topic", parameters.outputTopic_);
  nodeHandle_->get_parameter("postprocessor_pipeline_name", parameters.filterChainParametersName_);
  
  parameters_.setData(parameters);
}

grid_map::GridMap PostprocessingPipelineFunctor::operator()(GridMap& inputMap) {
  if (!filterChainConfigured_) {
    RCLCPP_WARN_ONCE(nodeHandle_->get_logger(), "No postprocessing pipeline was configured. Forwarding the raw elevation map!");
    return inputMap;
  }

  grid_map::GridMap outputMap;
  if (!filterChain_.update(inputMap, outputMap)) {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Could not perform the grid map filter chain! Forwarding the raw elevation map!");
    return inputMap;
  }

  return outputMap;
}

void PostprocessingPipelineFunctor::publish(const GridMap& gridMap) const {
  // Publish filtered output grid map.
  grid_map_msgs::msg::GridMap::UniquePtr outputMessage;
  outputMessage = grid_map::GridMapRosConverter::toMessage(gridMap);
  publisher_->publish(*outputMessage);
  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Elevation map raw has been published.");
}

bool PostprocessingPipelineFunctor::hasSubscribers() const {
  return publisher_->get_subscription_count() > 0;
}

}  // namespace elevation_mapping
