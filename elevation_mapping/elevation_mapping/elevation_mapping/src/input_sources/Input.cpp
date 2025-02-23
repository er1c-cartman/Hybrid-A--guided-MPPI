/*
 *  Input.cpp
 *
 *  Created on: Oct 06, 2020
 *  Author: Magnus Gärtner
 *  Institute: ETH Zurich, ANYbotics
 */

#include <memory>

#include "elevation_mapping/input_sources/Input.hpp"

#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp"

namespace elevation_mapping {

Input::Input(rclcpp::Node::SharedPtr nh) : nodeHandle_(nh) {}

bool Input::configure(std::string name, const rclcpp::Node::SharedPtr nodeHandle,
                      const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters) {
  Parameters parameters;
  //In Input.cpp
  nodeHandle->declare_parameter<std::string>(name + ".type", "pointcloud");
  nodeHandle->declare_parameter<std::string>(name + ".topic", "/velodyne_points");
  nodeHandle->declare_parameter<int>(name + ".queue_size", 10);
  nodeHandle->declare_parameter<bool>(name + ".publish_on_update", true);
  nodeHandle->declare_parameter<std::string>(name + ".sensor_processor.type", "laser");
  // Check if the input source is enabled.
  bool isEnabled = true;  // Default to enabled if the parameter is not set.
  // nodeHandle->get_parameter_or(name + ".enabled", isEnabled, isEnabled);
  parameters.isEnabled_ = isEnabled;

  // Check that all required parameters exist and have the appropriate types.
  std::vector<std::string> requiredParameters = {"type", "topic", "queue_size", "publish_on_update", "sensor_processor.type"};
  for (const auto& paramName : requiredParameters) {
    if (!nodeHandle->has_parameter(name + "." + paramName)) {
      RCLCPP_ERROR(nodeHandle->get_logger(), "Could not configure input source %s because no %s was given.", name.c_str(), paramName.c_str());
      return false;
    }
  }

  parameters.name_ = name;
  if (!nodeHandle->get_parameter(name + ".type", parameters.type_)) {
    RCLCPP_ERROR(nodeHandle->get_logger(), "Could not get parameter 'type' for input source %s.", name.c_str());
    return false;
  }
  if (!nodeHandle->get_parameter(name + ".topic", parameters.topic_)) {
    RCLCPP_ERROR(nodeHandle->get_logger(), "Could not get parameter 'topic' for input source %s.", name.c_str());
    return false;
  }
  int queueSize;
  if (!nodeHandle->get_parameter(name + ".queue_size", queueSize)) {
    RCLCPP_ERROR(nodeHandle->get_logger(), "Could not get parameter 'queue_size' for input source %s.", name.c_str());
    return false;
  }
  if (queueSize < 0) {
    RCLCPP_ERROR(nodeHandle->get_logger(), "The specified queue_size is negative for input source %s.", name.c_str());
    return false;
  }
  parameters.queueSize_ = static_cast<unsigned int>(queueSize);

  bool publishOnUpdate;
  if (!nodeHandle->get_parameter(name + ".publish_on_update", publishOnUpdate)) {
    RCLCPP_ERROR(nodeHandle->get_logger(), "Could not get parameter 'publish_on_update' for input source %s.", name.c_str());
    return false;
  }
  parameters.publishOnUpdate_ = publishOnUpdate;

  parameters_.setData(parameters);

  // SensorProcessor
  if (!configureSensorProcessor(name, nodeHandle, generalSensorProcessorParameters)) {
    return false;
  }

  RCLCPP_INFO(nodeHandle->get_logger(), "Configured %s:%s @ %s (publishing_on_update: %s), using %s to process data.",
                parameters.type_.c_str(), parameters.name_.c_str(), parameters.topic_.c_str(),
                parameters.publishOnUpdate_ ? "true" : "false",
                nodeHandle->get_parameter(name + ".sensor_processor.type").as_string().c_str());

  return true;
}


std::string Input::getSubscribedTopic() const {
  const Parameters parameters{parameters_.getData()};
  return parameters.topic_;
}

bool Input::configureSensorProcessor(std::string name, const rclcpp::Node::SharedPtr& nodeHandle,
                                     const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters) {
  std::string paramBaseName = name + ".sensor_processor";

  // Check if the "type" parameter exists and is of type string.
  std::string sensorType;
  if (!nodeHandle->get_parameter(paramBaseName + ".type", sensorType)) {
    RCLCPP_ERROR(nodeHandle->get_logger(), "Could not configure sensor processor of input source %s because no type was given.", name.c_str());
    return false;
  }

  // Check if the sensor type is supported and create the corresponding sensor processor.
  if (sensorType == "structured_light") {
    sensorProcessor_ = std::make_unique<StructuredLightSensorProcessor>(nodeHandle, generalSensorProcessorParameters);
  } else if (sensorType == "stereo") {
    sensorProcessor_ = std::make_unique<StereoSensorProcessor>(nodeHandle, generalSensorProcessorParameters);
  } else if (sensorType == "laser") {
    sensorProcessor_ = std::make_unique<LaserSensorProcessor>(nodeHandle, generalSensorProcessorParameters);
  } else if (sensorType == "perfect") {
    sensorProcessor_ = std::make_unique<PerfectSensorProcessor>(nodeHandle, generalSensorProcessorParameters);
  } else {
    RCLCPP_ERROR(nodeHandle->get_logger(), "The sensor type %s is not available.", sensorType.c_str());
    return false;
  }

  // Read parameters for the sensor processor.
  return sensorProcessor_->readParameters();
}

}  // namespace elevation_mapping
