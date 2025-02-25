#ifndef BASE_DRIVER_H_
#define BASE_DRIVER_H_


#include <inttypes.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <iostream>
#include <unistd.h>
#include <serial/serial.h> //ROS的串口包 http://wjwwood.io/serial/doc/1.1.0/index.html
#include <math.h>
#include <fstream>
#include <fdilink_data_struct.h>
//#include <sensor_msgs/Imu.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <string>
//#include <ros/package.h>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <crc_table.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>
//#include <boost/thread.h>

using namespace std;
using namespace Eigen;
namespace FDILink
{
#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd
#define TYPE_IMU 0x40
#define TYPE_AHRS 0x41
#define TYPE_INSGPS 0x42
#define TYPE_GEODETIC_POS 0x5c
#define TYPE_GROUND 0xf0

#define IMU_LEN  0x38   //56
#define AHRS_LEN 0x30   //48
#define INSGPS_LEN 0x48 //80
#define GEODETIC_POS_LEN 0x20 //32
#define PI 3.141592653589793
#define DEG_TO_RAD 0.017453292519943295


class ahrsBringup : public rclcpp::Node
{
public:
  ahrsBringup();
  ~ahrsBringup();
  void processLoop();
  bool checkCS8(int len);
  bool checkCS16(int len);
  void checkSN(int type);
  void magCalculateYaw(double roll, double pitch, double &magyaw, double magx, double magy, double magz);

private:
  bool if_debug_;
  //sum info
  int sn_lost_ = 0;
  int crc_error_ = 0;
  uint8_t read_sn_ = 0;
  bool frist_sn_;
  int device_type_ = 1;

  //serial
  serial::Serial serial_; //声明串口对象
  std::string serial_port_;
  int serial_baud_;
  int serial_timeout_;
  //data
  FDILink::imu_frame_read  imu_frame_;
  FDILink::ahrs_frame_read ahrs_frame_;
  FDILink::insgps_frame_read insgps_frame_;
  //FDILink::lanlon_frame_read latlon_frame_;
  FDILink::Geodetic_Position_frame_read Geodetic_Position_frame_;
  //frame name
  //std::string imu_frame_id="gyro_link";
  std::string imu_frame_id_;
  std::string insgps_frame_id_;
  std::string latlon_frame_id_;
  //topic
  std::string imu_topic="/imu", mag_pose_2d_topic="/mag_pose_2d";
  std::string latlon_topic="latlon";
  std::string Euler_angles_topic="/Euler_angles", Magnetic_topic="/Magnetic";
  std::string gps_topic="/gps/fix",twist_topic="/system_speed",NED_odom_topic="/NED_odometry";


  //Publisher
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr  imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr mag_pose_pub_;

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr Euler_angles_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr Magnetic_pub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  twist_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr NED_odom_pub_;



}; //ahrsBringup
} // namespace FDILink

#endif
