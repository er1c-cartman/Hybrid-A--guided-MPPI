/*
 *  InputSourceManager.cpp
 *
 *  Created on: Oct 02, 2020
 *  Author: Magnus Gärtner
 *  Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/input_sources/InputSourceManager.hpp"
#include "elevation_mapping/ElevationMapping.hpp"

namespace elevation_mapping {

InputSourceManager::InputSourceManager(const rclcpp::Node::SharedPtr nodeHandle) : nodeHandle_(nodeHandle) {}

bool InputSourceManager::configureFromRos(const std::string& inputSourcesNamespace) {
  RCLCPP_INFO(nodeHandle_->get_logger(), "Configuring from ROS using namespace: %s", inputSourcesNamespace.c_str());

  nodeHandle_->declare_parameter<std::vector<std::string>>(inputSourcesNamespace, std::vector<std::string>());

  std::vector<std::string> inputSourcesConfiguration;
  if (!nodeHandle_->get_parameter(inputSourcesNamespace, inputSourcesConfiguration)) {
    std::string resolved_name = nodeHandle_->get_namespace() + std::string("/") + inputSourcesNamespace;
    RCLCPP_WARN(nodeHandle_->get_logger(),
        "Could not load the input sources configuration from parameter\n"
        "%s, are you sure it was pushed to the parameter server? Assuming\n"
        "that you meant to leave it empty. Not subscribing to any inputs!\n",
        resolved_name.c_str());
    return false;
  }

  RCLCPP_INFO(nodeHandle_->get_logger(), "Loaded input sources configuration: %s", inputSourcesConfiguration.size() > 0 ? "non-empty" : "empty");

  // inputSourcesConfiguration.push_back("lidar");
  return configure(inputSourcesConfiguration, inputSourcesNamespace);
}

bool InputSourceManager::configure(const std::vector<std::string>& config, const std::string& sourceConfigurationName) {
  RCLCPP_INFO(nodeHandle_->get_logger(), "Configuring input sources.");

  bool successfulConfiguration = true;
  std::set<std::string> subscribedTopics;
  // SensorProcessorBase::GeneralParameters generalSensorProcessorConfig{
  //     nodeHandle_->declare_parameter<std::string>("robot_base_frame_id", std::string("/robot")),
  //     nodeHandle_->declare_parameter<std::string>("map_frame_id", std::string("/map"))};
    SensorProcessorBase::GeneralParameters generalSensorProcessorConfig{};


  for (const auto& inputConfigName : config) {
    RCLCPP_INFO(nodeHandle_->get_logger(), "Configuring source: %s", inputConfigName.c_str());

    rclcpp::Node::SharedPtr inputNode = std::make_shared<rclcpp::Node>(sourceConfigurationName + "_" + inputConfigName);
    Input source{nodeHandle_};

    const bool configured{source.configure(inputConfigName, nodeHandle_, generalSensorProcessorConfig)};
    if (!configured) {
      RCLCPP_ERROR(nodeHandle_->get_logger(), "Failed to configure source: %s", inputConfigName.c_str());
      successfulConfiguration = false;
      continue;
    }

    if (!source.isEnabled()) {
      continue;
    }

    const std::string subscribedTopic{source.getSubscribedTopic()};
    const bool topicIsUnique{subscribedTopics.insert(subscribedTopic).second};

    if (topicIsUnique) {
      sources_.push_back(std::move(source));
    } else {
      RCLCPP_ERROR(nodeHandle_->get_logger(),
          "The input sources specification tried to subscribe to %s "
          "multiple times. Only subscribing once.",
          subscribedTopic.c_str());
      successfulConfiguration = false;
    }
  }

  return successfulConfiguration;
}

int InputSourceManager::getNumberOfSources() {
  return static_cast<int>(sources_.size());
}

}  // namespace elevation_mapping
