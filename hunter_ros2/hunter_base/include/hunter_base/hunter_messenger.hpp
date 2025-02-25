/*
 * hunter_messenger.hpp
 *
 * Created on: Jun 14, 2019 10:24
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef HUNTER_MESSENGER_HPP
#define HUNTER_MESSENGER_HPP

#include <string>
#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ascent/Ascent.h"
#include "ascent/Utility.h"
#include "hunter_base/bicycle_model.hpp"
#include "hunter_base/hunter_params.hpp"

#include "ugv_sdk/mobile_robot/hunter_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"
#include "hunter_msgs/msg/hunter_status.hpp"
#include "hunter_msgs/msg/hunter_light_cmd.hpp"

namespace westonrobot {

template <typename SystemModel>
class SystemPropagator {
 public:
  asc::state_t Propagate(asc::state_t init_state,
                         typename SystemModel::control_t u, double t0,
                         double tf, double dt) {
    double t = t0;
    asc::state_t x = init_state;

    while (t <= tf) {
      integrator_(SystemModel(u), x, t, dt);
      // Note: you may need to add additional constraints to [x]
    }
    return x;
  }

 private:
  asc::RK4 integrator_;
};

template <typename HunterType>
class HunterMessenger {
 public:
  HunterMessenger(std::shared_ptr<HunterType> hunter, rclcpp::Node *node)
      : hunter_(hunter), node_(node) {}
  // SystemPropagator<BicycleKinematics> model_;
  void SetOdometryFrame(std::string frame) { odom_frame_ = frame; }
  void SetBaseFrame(std::string frame) { base_frame_ = frame; }
  void SetOdometryTopicName(std::string name) { odom_topic_name_ = name; }
  void SetWeelbase(float Weelbase){
    l = Weelbase;
  }
  void SetTrack(float Track){
    w = Track;
  }
  void SetMaxSteerAngleCentral(float Angle){
    max_steer_angle_central = Angle;
  }
  void SetMaxSteerAngle(float Angle){
    max_steer_angle = Angle;
  }
  void SetSimulationMode(int loop_rate) {
    simulated_robot_ = true;
    sim_control_rate_ = loop_rate;
  }

  void SetupSubscription() {
    // odometry publisher
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    odom_pub_ =
        node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, 50);
    status_pub_ = node_->create_publisher<hunter_msgs::msg::HunterStatus>(
        "/hunter_status", 10);

    // cmd subscriber
    motion_cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&HunterMessenger::TwistCmdCallback, this,
                  std::placeholders::_1));

    
  }

  void PublishStateToROS() {
    current_time_ = node_->get_clock()->now();

    static bool init_run = true;
    if (init_run) {
      last_time_ = current_time_;
      init_run = false;
      return;
    }
    double dt = (current_time_ - last_time_).seconds();

    auto state = hunter_->GetRobotState();

    // publish hunter state message
    hunter_msgs::msg::HunterStatus status_msg;

    status_msg.header.stamp = current_time_;

    status_msg.linear_velocity = state.motion_state.linear_velocity;
    double phi =ConvertInnerAngleToCentral(state.motion_state.steering_angle);
    status_msg.steering_angle = phi;
    // status_msg.angular_velocity = state.motion_state.steering_angle;

    status_msg.vehicle_state = state.system_state.vehicle_state;
    status_msg.control_mode = state.system_state.control_mode;
    status_msg.error_code = state.system_state.error_code;
    status_msg.battery_voltage = state.system_state.battery_voltage;

    auto actuator = hunter_->GetActuatorState();

    for (int i = 0; i < 3; ++i) {
      // actuator_hs_state
      uint8_t motor_id = actuator.actuator_hs_state[i].motor_id;

      status_msg.actuator_states[motor_id].rpm =
          actuator.actuator_hs_state[i].rpm;
      status_msg.actuator_states[motor_id].current =
          actuator.actuator_hs_state[i].current;
      status_msg.actuator_states[motor_id].pulse_count =
          actuator.actuator_hs_state[i].pulse_count;

      // actuator_ls_state
      motor_id = actuator.actuator_ls_state[i].motor_id;

      status_msg.actuator_states[motor_id].driver_voltage =
          actuator.actuator_ls_state[i].driver_voltage;
      status_msg.actuator_states[motor_id].driver_temperature =
          actuator.actuator_ls_state[i].driver_temp;
      status_msg.actuator_states[motor_id].motor_temperature =
          actuator.actuator_ls_state[i].motor_temp;
      status_msg.actuator_states[motor_id].driver_state =
          actuator.actuator_ls_state[i].driver_state;
    }

    status_pub_->publish(status_msg);

    // publish odometry and tf
    PublishOdometryToROS(state.motion_state, dt);

    // record time for next integration
    last_time_ = current_time_;
  }

 private:
  std::shared_ptr<HunterType> hunter_;
  rclcpp::Node *node_;
  

  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;

  bool simulated_robot_ = false;
  int sim_control_rate_ = 50;

  westonrobot::SystemPropagator<BicycleKinematics> model_;

  std::mutex twist_mutex_;
  geometry_msgs::msg::Twist current_twist_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<hunter_msgs::msg::HunterStatus>::SharedPtr status_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_sub_;
  

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // speed variables
  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;

  static constexpr double steer_angle_tolerance = 0.005;  // ~+-0.287 degrees
  double l = 0.0;
  double w = 0.0;
  double max_steer_angle_central = 0.0;
  double max_steer_angle = 0.0;

  rclcpp::Time last_time_;
  rclcpp::Time current_time_;

  void TwistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    
    if (!simulated_robot_) {
      SetHunterMotionCommand(msg);
    } else {
      std::lock_guard<std::mutex> guard(twist_mutex_);
      current_twist_ = *msg.get();
    }
    // ROS_INFO("Cmd received:%f, %f", msg->linear.x, msg->angular.z);
  }

  // template <typename T,std::enable_if_t<!std::is_base_of<HunterRobot, T>::value,bool> = true>
  void SetHunterMotionCommand(const geometry_msgs::msg::Twist::SharedPtr &msg) {

    std::shared_ptr<HunterRobot> base;

    double radian = 0;
    double phi_i = AngelVelocity2Angel(*msg,radian);

    std::cout << "set steering angle: " << phi_i << std::endl;
    hunter_->SetMotionCommand(msg->linear.x, phi_i);
    // hunter_
 
  }

  double ConvertCentralAngleToInner(double angle)
  {
    double phi = angle;
    double phi_i = 0;
    if (phi > steer_angle_tolerance) {
      // left turn
      phi_i = std::atan(2 * l * std::sin(phi) /
                        (2 * l * std::cos(phi) - w * std::sin(phi)));
    } else if (phi < -steer_angle_tolerance) {
      // right turn
      phi = -phi;
      phi_i = std::atan(2 * l * std::sin(phi) /
                        (2 * l * std::cos(phi) - w * std::sin(phi)));
      phi_i = -phi_i;
    }
    return phi_i;
  }
  double ConvertInnerAngleToCentral(double angle)
  {
    double phi = 0;
    double phi_i = angle;
    if (phi_i > steer_angle_tolerance) {
      // left turn
      double r = l / std::tan(phi_i) + w / 2;
      phi = std::atan(l / r);
    } else if (phi_i < -steer_angle_tolerance) {
      // right turn
      double r = l / std::tan(-phi_i) + w / 2;
      phi = std::atan(l / r);
      phi = -phi;
    }
    return phi;

  }

  geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
  }

  void PublishOdometryToROS(const MotionStateMessage &msg, double dt) {
    // perform numerical integration to get an estimation of pose
    double linear_speed_ = msg.linear_velocity;
    // double angular_speed = msg.angular_velocity;
    double steering_angle_ = ConvertInnerAngleToCentral(msg.steering_angle);
    // double lateral_speed = 0;

    asc::state_t state =
      model_.Propagate({position_x_, position_y_, theta_},
                       {linear_speed_, steering_angle_}, 0, dt, dt / 100);
    position_x_ = state[0];
    position_y_ = state[1];
    theta_ = state[2];


    geometry_msgs::msg::Quaternion odom_quat =
        createQuaternionMsgFromYaw(theta_);

    // publish tf transformation
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    //tf_broadcaster_->sendTransform(tf_msg);

    // publish odometry and tf messages
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = linear_speed_;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.angular.z = linear_speed_ / l * std::tan(steering_angle_);

    odom_msg.pose.covariance = {
    0.001,      0.0,        0.0,        0.0,        0.0,        0.0,
    0.0,        0.001,      0.0,        0.0,        0.0,        0.0,
    0.0,        0.0,        1000000.0,  0.0,        0.0,        0.0,
    0.0,        0.0,        0.0,        1000000.0,  0.0,        0.0,
    0.0,        0.0,        0.0,        0.0,        1000000.0,  0.0,
    0.0,        0.0,        0.0,        0.0,        0.0,        1000.0};
    odom_msg.twist.covariance = {
    0.001,      0.0,        0.0,        0.0,        0.0,        0.0,
    0.0,        0.001,      0.0,        0.0,        0.0,        0.0,
    0.0,        0.0,        1000000.0,  0.0,        0.0,        0.0,
    0.0,        0.0,        0.0,        1000000.0,  0.0,        0.0,
    0.0,        0.0,        0.0,        0.0,        1000000.0,  0.0,
    0.0,        0.0,        0.0,        0.0,        0.0,        1000.0};

    odom_pub_->publish(odom_msg);
  }
  double AngelVelocity2Angel(geometry_msgs::msg::Twist msg,double &radius)
  {
    double linear = fabs(msg.linear.x);
    double angular = fabs(msg.angular.z);
    if(angular == 0)
    {
      return 0.0;
    }

    radius = linear / angular;

    int k = msg.angular.z / fabs(msg.angular.z);
    if ((radius-l)<0 )
    {
      return  k*max_steer_angle;
    }

    double phi_i;
    phi_i = atan(l/(radius-w/2));
    if(msg.linear.x<0)
      phi_i *= -1.0;
    return k*phi_i;
  }
};
}  // namespace westonrobot

#endif /* Hunter_MESSENGER_HPP */
