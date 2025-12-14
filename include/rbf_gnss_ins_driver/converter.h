#ifndef CONVERTER_H
#define CONVERTER_H
#include <rbf_gnss_ins_driver/msg/ecef.hpp>
#include <rbf_gnss_ins_driver/msg/gnss_status.hpp>
#include <rbf_gnss_ins_driver/msg/gnss_vel.hpp>
#include <rbf_gnss_ins_driver/msg/gpnav.hpp>
#include <rbf_gnss_ins_driver/msg/heading.hpp>
#include <rbf_gnss_ins_driver/msg/imu_status.hpp>
#include <rbf_gnss_ins_driver/msg/ins.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/header.hpp>

#include <rbf_gnss_ins_driver/structs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cstdint>

namespace rbf_gnss_ins_driver
{
class Converter
{
private:
  static double raw_gyro_to_deg_s(int32_t raw_gyro);
  static double raw_acc_to_m_s2(int32_t raw_acc);
  static double calc_imu_temperature(const RawImu & raw_imu);
  static double calc_imu_temperature(const RawImux & raw_imux);
  static double degree_to_radian(double degree);
  std_msgs::msg::Header create_header(std::string frame_id);
  int64_t timestamp_;
  bool use_ros_time_;

public:
  enum AltitudeMode : uint8_t { ORTHOMETRIC = 0, ELLIPSOIDAL = 1 };
  AltitudeMode altitude_mode_;
  // Constructor
  Converter() = default;
  Converter(bool use_ros_time, AltitudeMode altitude_mode)
  : use_ros_time_(use_ros_time), altitude_mode_(altitude_mode){};
  // Destructor
  ~Converter() = default;

  bool is_delay_high(int64_t timestamp);

  void set_timestamp(int64_t timestamp) { timestamp_ = timestamp; }

  static bool is_ins_active(uint32_t ins_status) { return ins_status > 1; };
  static bool is_ins_active(const rbf_gnss_ins_driver::msg::Gpnav & gpnav)
  {
    return gpnav.status1 == 6;
  };
  // Function to convert raw imu data to imu status message
  rbf_gnss_ins_driver::msg::ImuStatus raw_imu_to_imu_status(
    const RawImux & raw_imux, std::string frame_id);

  // Function to convert raw heading data to ROS message
  rbf_gnss_ins_driver::msg::Heading heading_to_msg(
    const UniHeading & heading, std::string frame_id);

  // Function to convert raw ecef data to ROS message
  rbf_gnss_ins_driver::msg::ECEF ecef_to_msg(const ECEF & ecef, std::string frame_id);

  // Function to convert raw gnss velocity data to ROS message
  rbf_gnss_ins_driver::msg::GnssVel gnss_vel_to_msg(
    const BestGnssVel & gnss_vel, std::string frame_id);

  // Function to convert raw gnss position data to ROS message
  rbf_gnss_ins_driver::msg::GnssStatus gnss_pos_to_gnss_status_msg(
    const BestGnssPos & gnss_pos, std::string frame_id);

  // Function to convert raw ins data to ROS message
  rbf_gnss_ins_driver::msg::Ins ins_to_msg(const InsPvax & ins_pva, std::string frame_id);

  // Function to convert raw imu data to IMU ROS message
  sensor_msgs::msg::Imu raw_imu_to_imu_msg(const RawImux & raw_imux, std::string frame_id);

  // Function to convert imu data to IMU ROS message
  sensor_msgs::msg::Imu imu_data_to_imu_msg(const ImuData & imu_data, std::string frame_id);

  // Function to convert raw gnss position data to NavSatFix ROS message
  sensor_msgs::msg::NavSatFix gnss_pos_to_nav_sat_fix_msg(
    const BestGnssPos & gnss_pos, std::string frame_id);

  // Function to convert raw imu data to Temperature ROS message
  sensor_msgs::msg::Temperature raw_imu_to_temperature_msg(
    const RawImux & raw_imux, std::string frame_id);

  // Function to convert ECEF to twist message
  geometry_msgs::msg::TwistWithCovarianceStamped ecef_to_twist_msg(
    const ECEF & ecef, const RawImux & raw_imux, std::string frame_id);

  // Function to convert ins data to NavSatFix ROS message
  sensor_msgs::msg::NavSatFix ins_to_nav_sat_fix_msg(const InsPvax & ins_pva, std::string frame_id);
  sensor_msgs::msg::NavSatFix gpnav_to_nav_sat_fix_msg(
    const rbf_gnss_ins_driver::msg::Gpnav & gpnav, std::string frame_id);

  // Function to convert ins data to IMU ROS message
  sensor_msgs::msg::Imu ins_to_imu_msg(
    const InsPvax & ins_pva, const RawImux & raw_imux, std::string frame_id);
  sensor_msgs::msg::Imu gpnav_to_imu_msg(
    const rbf_gnss_ins_driver::msg::Gpnav & gpnav, const ImuData & imu, std::string frame_id);

  // Function to convert ins and imu data to Odometry ROS message
  nav_msgs::msg::Odometry convert_to_odometry_msg(
    const InsPvax & ins_pva, const RawImux & raw_imux, double x, double y, double z,
    std::string frame_id);
  nav_msgs::msg::Odometry convert_to_odometry_msg(
    rbf_gnss_ins_driver::msg::Gpnav & gpnav, const ImuData & imu, double x, double y, double z,
    std::string frame_id);

  geometry_msgs::msg::TransformStamped create_transform(
    const geometry_msgs::msg::Pose & pose, std::string frame_id);

  // Function to convert INSPVA to twist message
  geometry_msgs::msg::TwistWithCovarianceStamped ins_to_twist_msg(
    const InsPvax & ins_pva, const RawImux & raw_imux, std::string frame_id);
  geometry_msgs::msg::TwistWithCovarianceStamped gpnav_to_twist_msg(
    const rbf_gnss_ins_driver::msg::Gpnav & gpnav, const ImuData & imu_data, std::string frame_id);

  // Function to convert gnss navigation data to ROS message
  rbf_gnss_ins_driver::msg::Gpnav create_gpnavigation_msg(
    const std::string & sentence, const std::string & frame_id);
};

}  // namespace rbf_gnss_ins_driver

#endif  // CONVERTER_H