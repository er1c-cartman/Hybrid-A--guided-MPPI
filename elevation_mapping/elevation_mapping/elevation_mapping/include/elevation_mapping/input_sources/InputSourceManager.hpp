// InputSourceManager.hpp

#pragma once

#include "elevation_mapping/input_sources/Input.hpp"
#include "rclcpp/rclcpp.hpp"
#include <utility>  // for std::pair

namespace elevation_mapping {
class ElevationMapping;  // Forward declare to avoid cyclic import dependency.

class InputSourceManager {
 public:
  explicit InputSourceManager(const rclcpp::Node::SharedPtr nodeHandle);

  bool configureFromRos(const std::string& inputSourcesNamespace);
  bool configure(const std::vector<std::string>& config, const std::string& sourceConfigurationName);

  template <typename... MsgTs>
  bool registerCallbacks(ElevationMapping& map, std::pair<const char*, typename Input::CallbackT<MsgTs>>... callbacks);



  int getNumberOfSources();

 protected:
  std::vector<Input> sources_;
  rclcpp::Node::SharedPtr nodeHandle_;
};

// Template definitions
template <typename... MsgTs>
  bool InputSourceManager::registerCallbacks(ElevationMapping& map, std::pair<const char*, typename Input::CallbackT<MsgTs>>... callbacks){
  if (sources_.empty()) {
    RCLCPP_WARN(nodeHandle_->get_logger(), "Not registering any callbacks, no input sources given. Did you configure the InputSourceManager?");
    return true;
  }
  for (Input& source : sources_) {
    bool callbackRegistered = false;
    for (auto& callback : {callbacks...}) {
      if (source.getType() == callback.first) {
        source.registerCallback(map, callback.second);
        callbackRegistered = true;
      }
    }
    if (!callbackRegistered) {
      RCLCPP_WARN(nodeHandle_->get_logger(), "The configuration contains input sources of an unknown type: %s", source.getType().c_str());
      RCLCPP_WARN(nodeHandle_->get_logger(), "Available types are:");
      for (auto& callback : {callbacks...}) {
        RCLCPP_WARN(nodeHandle_->get_logger(), "- %s", callback.first);
      }
      return false;
    }
  }
  return true;
}


}  // namespace elevation_mapping
