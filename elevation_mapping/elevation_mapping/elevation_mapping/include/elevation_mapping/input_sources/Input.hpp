/*
 *  Input.hpp
 *
 *  Created on: Oct 06, 2020
 *  Author: Magnus Gärtner
 *  Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "elevation_mapping/ThreadSafeDataWrapper.hpp"
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"

namespace elevation_mapping {
class ElevationMapping;  // Forward declare to avoid cyclic import dependency.

/**
 * @brief An Input feeds data to ElevationMapping callbacks. E.g it holds a subscription to an sensor source and registered an appropriate
 * ElevationMapping callback.
 */
class Input {
 public:
  template <typename MsgT>
  using CallbackT = void (ElevationMapping::*)(const MsgT, bool, const std::shared_ptr<SensorProcessorBase>&);
  /**
   * @brief Constructor.
   * @param nh Reference to the nodeHandle of the manager. Used to subscribe
   * to inputs.
   */
  explicit Input(rclcpp::Node::SharedPtr nh);

  /**
   * Whether the input source is enabled or not.
   */
  bool isEnabled() const {
    const Parameters parameters{parameters_.getData()};
    return parameters.isEnabled_;
  }

  /**
   * @brief Configure the input source.
   * @param name Name of this input source.
   * @param configuration to configure from.
   * @param generalSensorProcessorParameters Parameters shared by all sensor processors.
   * @return True if configuring was successful.
   */
  bool configure(std::string name, const rclcpp::Node::SharedPtr nodeHandle,
                 const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters);

  /**
   * @brief Registers the corresponding callback in the elevationMap.
   * @param map The map we want to link this input source to.
   * @param callback The callback to use for incoming data.
   * @tparam MsgT The message types of the callback.
   */
  template <typename MsgT>
  void registerCallback(ElevationMapping& map, CallbackT<MsgT> callback);
  /**
   * @return The topic (as absolute path, with renames) that this input
   * subscribes to.
   */
  std::string getSubscribedTopic() const;

  /**
   * @return The type of this input source.
   */
  std::string getType() {
    const Parameters parameters{parameters_.getData()};
    return parameters.type_;
  }

 private:
  /**
   * @brief Configures the used sensor processor from the given parameters.
   * @param name The name of this input source
   * @param parameters The parameters of this input source
   * @param generalSensorProcessorParameters  General parameters needed for the sensor processor that are not specific to this sensor
   * processor.
   * @return True if successful.
   */
  bool configureSensorProcessor(std::string name, const rclcpp::Node::SharedPtr& nodeHandle,
                                const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters);

  // ROS connection.
  rclcpp::SubscriptionBase::SharedPtr subscriber_;
  rclcpp::Node::SharedPtr nodeHandle_;

  //! Sensor processor
  std::shared_ptr<SensorProcessorBase> sensorProcessor_;

  // Parameters.
  struct Parameters {
    std::string name_;
    std::string type_;
    bool isEnabled_{true};
    uint32_t queueSize_{0};
    std::string topic_;
    bool publishOnUpdate_{true};
  };
  ThreadSafeDataWrapper<Parameters> parameters_;
};
template <typename MsgT>
void Input::registerCallback(ElevationMapping& map, CallbackT<MsgT> callback) {
  const Parameters parameters{parameters_.getData()};
  subscriber_ = nodeHandle_->create_subscription<sensor_msgs::msg::PointCloud2>(
      std::string(parameters.topic_), parameters.queueSize_,
      [callback, &map, publishOnUpdate = parameters.publishOnUpdate_, sensorProcessor = std::ref(sensorProcessor_)]
      (const MsgT msg) {
          (map.*callback)(msg, publishOnUpdate, sensorProcessor);
      });

}
}  // namespace elevation_mapping