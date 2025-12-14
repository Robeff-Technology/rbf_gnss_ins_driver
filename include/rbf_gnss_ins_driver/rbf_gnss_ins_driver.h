#ifndef RBF_GNSS_INS_DRIVER_H
#define RBF_GNSS_INS_DRIVER_H

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rbf_gnss_ins_driver/msg/ecef.hpp>
#include <rbf_gnss_ins_driver/msg/gnss_status.hpp>
#include <rbf_gnss_ins_driver/msg/gnss_vel.hpp>
#include <rbf_gnss_ins_driver/msg/gpnav.hpp>
#include <rbf_gnss_ins_driver/msg/heading.hpp>
#include <rbf_gnss_ins_driver/msg/imu_status.hpp>
#include <rbf_gnss_ins_driver/msg/ins.hpp>
#include <rbf_gnss_ins_driver/msg/rtcm_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <mavros_msgs/msg/rtcm.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <rbf_gnss_ins_driver/converter.h>
#include <rbf_gnss_ins_driver/ll_to_utm_transform.h>
#include <rbf_gnss_ins_driver/parser.h>
#include <rbf_gnss_ins_driver/serial_port.h>
#include <rbf_gnss_ins_driver/structs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace rbf_gnss_ins_driver
{
class GnssInsDriver : public rclcpp::Node
{
public:
  struct ConfigParams
  {
    int working_frequency_;
    bool use_ros_time_;
    int altitude_mode_;

    struct serial
    {
      std::string serial_port_;
      int baudrate_;
    };

    struct topics
    {
      std::string rtcm_topic_;
      std::string imu_topic_;
      std::string nav_sat_fix_topic_;
      std::string twist_topic_;
      std::string temperature_topic_;
      std::string odometry_topic_;
    };

    struct frames
    {
      std::string gnss_frame_;
      std::string imu_frame_;
      std::string odometry_frame_;
    };

    struct odometry
    {
      bool use_odometry_;
      double lat_origin_;
      double long_origin_;
      double alt_origin_;
    };

    serial serial_;
    topics topics_;
    frames frames_;
    odometry odometry_;
  };
  ConfigParams config_params_;
  GnssInsDriver(const rclcpp::NodeOptions & options);

private:
  void load_parameters();
  struct RtcmStatus
  {
    uint32_t received_size_;
    uint32_t transmitted_size_;
  };

  std::shared_ptr<GnssStreamParser> gnss_parser_;
  std::shared_ptr<SerialPort> serial_port_ptr_;
  std::shared_ptr<Converter> converter_;
  rclcpp::TimerBase::SharedPtr timer_;

  /*STRUCTS*/
  ImuData imu_data_;
  RawImux raw_imux_;
  RawImu raw_imu_;
  BestGnssPos gnss_pos_;
  BestGnssVel gnss_vel_;
  InsPvax ins_pva_;
  UniHeading heading_;
  ECEF ecef_;
  RtcmStatus rtcm_status_;
  rbf_gnss_ins_driver::msg::Gpnav gpnav_;

  /*CUSTOM PUBLISHERS*/
  rclcpp::Publisher<rbf_gnss_ins_driver::msg::Ins>::SharedPtr pub_ins_;
  rclcpp::Publisher<rbf_gnss_ins_driver::msg::Heading>::SharedPtr pub_heading_;
  rclcpp::Publisher<rbf_gnss_ins_driver::msg::ECEF>::SharedPtr pub_ecef_;
  rclcpp::Publisher<rbf_gnss_ins_driver::msg::ImuStatus>::SharedPtr pub_imu_status_;
  rclcpp::Publisher<rbf_gnss_ins_driver::msg::GnssVel>::SharedPtr pub_gnss_vel_;
  rclcpp::Publisher<rbf_gnss_ins_driver::msg::GnssStatus>::SharedPtr pub_gnss_status_;
  rclcpp::Publisher<rbf_gnss_ins_driver::msg::RTCMStatus>::SharedPtr pub_rtcm_status_;
  rclcpp::Publisher<rbf_gnss_ins_driver::msg::Gpnav>::SharedPtr pub_gpnavigation_;

  /*DIAGNOSTIC UPDATER*/
  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;

  /*WITHOUT INS STD MSGS PUBLISHERS*/
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_raw_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_nav_sat_fix_raw_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temperature_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_ecef_twist_;

  /*WITH INS STD MSGS PUBLISHERS*/
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_nav_sat_fix_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_;

  /*ODOMETRY PUBLISHERS*/
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcast_;
  std::shared_ptr<LlToUtmTransform> ll_to_utm_transform_;

  void init_publishers();

  /*SUBSCRIBERS*/
  rclcpp::Subscription<mavros_msgs::msg::RTCM>::SharedPtr sub_rtcm_;

  /*CALLBACKS*/
  void binary_callback(const uint8_t * data, GnssStreamParser::MessageId id);
  void nmea_callback(const std::string & nmea_msg);
  void timer_callback();
  void rtcm_callback(const mavros_msgs::msg::RTCM::SharedPtr msg);
  void diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper & stat);
};
}  // namespace rbf_gnss_ins_driver
#endif  // RBF_GNSS_INS_DRIVER_H
