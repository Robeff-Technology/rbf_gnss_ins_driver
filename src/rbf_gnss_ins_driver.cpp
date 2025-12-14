#include "rclcpp/rclcpp.hpp"
#include "rbf_gnss_ins_driver/rbf_gnss_ins_driver.h"

namespace rbf_gnss_ins_driver
{
    GnssInsDriver::GnssInsDriver(const rclcpp::NodeOptions &options) : Node("rbf_gnss_ins_driver", options)
    {
        load_parameters();
        init_publishers();
        try
        {
            serial_port_ptr_ = std::make_shared<SerialPort>(config_params_.serial_.serial_port_.c_str());
            serial_port_ptr_->open();
            serial_port_ptr_->configure(config_params_.serial_.baudrate_, 8, 'N', 1);
        }
        catch (const SerialPortException &e)
        {
            RCLCPP_ERROR(get_logger(), e.what());
            rclcpp::shutdown();
        }
        diagnostic_updater_ = std::make_shared<diagnostic_updater::Updater>(this);
        diagnostic_updater_->setHardwareID("ROBINS_GNSS_INS");
        diagnostic_updater_->add("GNSS_INS_DRIVER", this, &GnssInsDriver::diagnostic_callback);

        converter_ = std::make_shared<Converter>(config_params_.use_ros_time_, static_cast<Converter::AltitudeMode>(config_params_.altitude_mode_));
        gnss_parser_ = std::make_shared<GnssStreamParser>(std::bind(&GnssInsDriver::binary_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&GnssInsDriver::nmea_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / config_params_.working_frequency_), std::bind(&GnssInsDriver::timer_callback, this));
        sub_rtcm_ = this->create_subscription<mavros_msgs::msg::RTCM>(config_params_.topics_.rtcm_topic_, 10, std::bind(&GnssInsDriver::rtcm_callback, this, std::placeholders::_1));
    }

    void GnssInsDriver::load_parameters()
    {
        config_params_.working_frequency_ = this->declare_parameter("working_frequency", 200);

        config_params_.serial_.serial_port_ = this->declare_parameter("serial_config.port", "/dev/ttyUSB0");
        config_params_.serial_.baudrate_ = this->declare_parameter("serial_config.baudrate", 460800);

        config_params_.topics_.rtcm_topic_ = this->declare_parameter("topic_config.rtcm_topic", "rtcm");
        config_params_.topics_.imu_topic_ = this->declare_parameter("topic_config.imu_topic", "imu");
        config_params_.topics_.nav_sat_fix_topic_ = this->declare_parameter("topic_config.nav_sat_fix_topic", "nav_sat_fix");
        config_params_.topics_.twist_topic_ = this->declare_parameter("topic_config.twist_topic", "twist");
        config_params_.topics_.temperature_topic_ = this->declare_parameter("topic_config.temperature_topic", "temperature");

        config_params_.frames_.gnss_frame_ = this->declare_parameter("frame_config.gnss_frame", "gnss");
        config_params_.frames_.imu_frame_ = this->declare_parameter("frame_config.imu_frame", "imu");

        config_params_.use_ros_time_ = this->declare_parameter("time_config.use_ros_time", true);

        config_params_.frames_.odometry_frame_ = this->declare_parameter("odometry_config.odometry_frame", "odometry");
        config_params_.topics_.odometry_topic_ = this->declare_parameter("odometry_config.odometry_topic", "odometry");
        config_params_.odometry_.use_odometry_ = this->declare_parameter("odometry_config.use_odometry", false);

        config_params_.odometry_.lat_origin_ = this->declare_parameter("origin_config.latitude", 0.0);
        config_params_.odometry_.long_origin_ = this->declare_parameter("origin_config.longitude", 0.0);
        config_params_.odometry_.alt_origin_ = this->declare_parameter("origin_config.altitude", 0.0);

        config_params_.altitude_mode_ = this->declare_parameter("altitude_config.altitude_mode", 0);

        RCLCPP_INFO(this->get_logger(), "-----------------------PARAMS-----------------------");
        RCLCPP_INFO(this->get_logger(), "working_frequency: %d", config_params_.working_frequency_);
        RCLCPP_INFO(this->get_logger(), "serial_port: %s", config_params_.serial_.serial_port_.c_str());
        RCLCPP_INFO(this->get_logger(), "baudrate: %d", config_params_.serial_.baudrate_);
        RCLCPP_INFO(this->get_logger(), "rtcm_topic: %s", config_params_.topics_.rtcm_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "imu_topic: %s", config_params_.topics_.imu_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "nav_sat_fix_topic: %s", config_params_.topics_.nav_sat_fix_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "twist_topic: %s", config_params_.topics_.twist_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "temperature_topic: %s", config_params_.topics_.temperature_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "gnss_frame: %s", config_params_.frames_.gnss_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "imu_frame: %s", config_params_.frames_.imu_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "use_ros_time: %d", config_params_.use_ros_time_);
        RCLCPP_INFO(this->get_logger(), "odometry_frame: %s", config_params_.frames_.odometry_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "odometry_topic: %s", config_params_.topics_.odometry_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "use_odometry: %d", config_params_.odometry_.use_odometry_);
        RCLCPP_INFO(this->get_logger(), "latitude: %f", config_params_.odometry_.lat_origin_);
        RCLCPP_INFO(this->get_logger(), "longitude: %f", config_params_.odometry_.long_origin_);
        RCLCPP_INFO(this->get_logger(), "altitude: %f", config_params_.odometry_.alt_origin_);
        RCLCPP_INFO(this->get_logger(), "altitude_mode: %d", config_params_.altitude_mode_);
        RCLCPP_INFO(this->get_logger(), "----------------------------------------------------");
    }

    void GnssInsDriver::init_publishers()
    {
        /*CUSTOM MSGS PUBLISHERS*/
        pub_ins_ = this->create_publisher<rbf_gnss_ins_driver::msg::Ins>("/robins/raw/ins", 10);
        pub_heading_ = this->create_publisher<rbf_gnss_ins_driver::msg::Heading>("/robins/raw/heading", 10);
        pub_ecef_ = this->create_publisher<rbf_gnss_ins_driver::msg::ECEF>("/robins/raw/ecef", 10);
        pub_imu_status_ = this->create_publisher<rbf_gnss_ins_driver::msg::ImuStatus>("/robins/status/imu_status", 10);
        pub_gnss_vel_ = this->create_publisher<rbf_gnss_ins_driver::msg::GnssVel>("/robins/raw/gnss_vel", 10);
        pub_gnss_status_ = this->create_publisher<rbf_gnss_ins_driver::msg::GnssStatus>("/robins/status/gnss_status", 10);
        pub_rtcm_status_ = this->create_publisher<rbf_gnss_ins_driver::msg::RTCMStatus>("/robins/status/rtcm_status", 10);
        pub_gpnavigation_ = this->create_publisher<rbf_gnss_ins_driver::msg::Gpnav>("/robins/raw/gpnav", 10);

        /*STD MSGS without INS*/
        pub_imu_raw_ = this->create_publisher<sensor_msgs::msg::Imu>("/robins/raw/imu", 10);
        pub_nav_sat_fix_raw_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/robins/raw/nav_sat_fix", 10);
        pub_temperature_ = this->create_publisher<sensor_msgs::msg::Temperature>("/robins/raw/temperature", 10);
        pub_ecef_twist_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/robins/raw/ecef_twist", 10);

        /*STANDARD MSGS PUBLISHERS*/
        pub_nav_sat_fix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(config_params_.topics_.nav_sat_fix_topic_, 10);
        pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(config_params_.topics_.twist_topic_, 10);
        pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>(config_params_.topics_.imu_topic_, 10);

        /*ODOM PUBLISHERS IF ENABLED*/
        if (config_params_.odometry_.use_odometry_)
        {
            pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>(config_params_.topics_.odometry_topic_, 10);
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            tf_static_broadcast_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
            ll_to_utm_transform_ = std::make_shared<LlToUtmTransform>(config_params_.odometry_.lat_origin_, config_params_.odometry_.long_origin_, config_params_.odometry_.alt_origin_);
            geometry_msgs::msg::Pose pos_msg;
            pos_msg.position.x = ll_to_utm_transform_->m_utm0_.easting;
            pos_msg.position.y = ll_to_utm_transform_->m_utm0_.northing;
            pos_msg.position.z = ll_to_utm_transform_->m_utm0_.altitude;
            auto msg = converter_->create_transform(pos_msg, config_params_.frames_.odometry_frame_);
            tf_static_broadcast_->sendTransform(msg);
        }
    }

    void GnssInsDriver::binary_callback(const uint8_t *data, GnssStreamParser::MessageId id)
    {
        converter_->set_timestamp(gnss_parser_->get_unix_time_ns());
        if (GnssStreamParser::MessageId::ECEF == id)
        {
            ecef_ = *reinterpret_cast<const ECEF *>(data);
            auto ecef_msg = converter_->ecef_to_msg(ecef_, config_params_.frames_.gnss_frame_);
            pub_ecef_->publish(ecef_msg);

            auto ecef_twist_msg = converter_->ecef_to_twist_msg(ecef_, raw_imux_, config_params_.frames_.gnss_frame_);
            pub_ecef_twist_->publish(ecef_twist_msg);
        }
        else if (GnssStreamParser::MessageId::RAWIMUX == id)
        {
            raw_imux_ = *reinterpret_cast<const RawImux *>(data);
            auto imu_status_msg = converter_->raw_imu_to_imu_status(raw_imux_, config_params_.frames_.imu_frame_);
            pub_imu_status_->publish(imu_status_msg);

            auto imu_msg = converter_->raw_imu_to_imu_msg(raw_imux_, config_params_.frames_.imu_frame_);
            pub_imu_raw_->publish(imu_msg);

            auto temperature_msg = converter_->raw_imu_to_temperature_msg(raw_imux_, config_params_.frames_.imu_frame_);
            pub_temperature_->publish(temperature_msg);
        }
        else if(GnssStreamParser::MessageId::IMUDATA == id) 
        {
            imu_data_ = *reinterpret_cast<const ImuData *>(data);

            auto imu_msg = converter_->imu_data_to_imu_msg(imu_data_, config_params_.frames_.imu_frame_);
            pub_imu_raw_->publish(imu_msg);
        }
        else if (GnssStreamParser::MessageId::HEADING == id)
        {
            heading_ = *reinterpret_cast<const UniHeading *>(data);
            auto heading_msg = converter_->heading_to_msg(heading_, config_params_.frames_.gnss_frame_);
            pub_heading_->publish(heading_msg);
        }
        else if (GnssStreamParser::MessageId::GNSSPOS == id || GnssStreamParser::MessageId::GNSSPOS_1 == id)
        {
            gnss_pos_ = *reinterpret_cast<const BestGnssPos *>(data);
            auto gnss_status_msg = converter_->gnss_pos_to_gnss_status_msg(gnss_pos_, config_params_.frames_.gnss_frame_);
            pub_gnss_status_->publish(gnss_status_msg);

            auto nav_sat_fix_msg = converter_->gnss_pos_to_nav_sat_fix_msg(gnss_pos_, config_params_.frames_.gnss_frame_);
            pub_nav_sat_fix_raw_->publish(nav_sat_fix_msg);
        }
        else if (GnssStreamParser::MessageId::GNSSVEL == id || GnssStreamParser::MessageId::GNSSVEL_1 == id)
        {
            gnss_vel_ = *reinterpret_cast<const BestGnssVel *>(data);
            auto gnss_vel_msg = converter_->gnss_vel_to_msg(gnss_vel_, config_params_.frames_.gnss_frame_);
            pub_gnss_vel_->publish(gnss_vel_msg);
        }
        else if (GnssStreamParser::MessageId::INSPVAX == id)
        {
            ins_pva_ = *reinterpret_cast<const InsPvax *>(data);
            auto ins_msg = converter_->ins_to_msg(ins_pva_, config_params_.frames_.gnss_frame_);
            pub_ins_->publish(ins_msg);

            if (Converter::is_ins_active(ins_pva_.ins_status))
            {
                /*ODOM SECTION*/
                if (config_params_.odometry_.use_odometry_)
                {
                    double x, y, z;
                    ll_to_utm_transform_->transform(ins_pva_.latitude, ins_pva_.longitude, ins_pva_.height, x, y, z);

                    auto odometry_msg = converter_->convert_to_odometry_msg(ins_pva_, raw_imux_, x, y, z, config_params_.frames_.odometry_frame_);
                    pub_odometry_->publish(odometry_msg);

                    auto transform = converter_->create_transform(odometry_msg.pose.pose, config_params_.frames_.odometry_frame_);
                    tf_broadcaster_->sendTransform(transform);
                }

                auto nav_sat_fix_msg = converter_->ins_to_nav_sat_fix_msg(ins_pva_, config_params_.frames_.gnss_frame_);
                pub_nav_sat_fix_->publish(nav_sat_fix_msg);

                auto imu_msg = converter_->ins_to_imu_msg(ins_pva_, raw_imux_, config_params_.frames_.imu_frame_);
                pub_imu_->publish(imu_msg);

                auto twist_msg = converter_->ins_to_twist_msg(ins_pva_, raw_imux_, config_params_.frames_.gnss_frame_);
                pub_twist_->publish(twist_msg);
            }
        }
    }

    void GnssInsDriver::nmea_callback(const std::string & nmea_msg)
    {
        auto nmea_msg_ = converter_->create_gpnavigation_msg(nmea_msg, config_params_.frames_.gnss_frame_);
        pub_gpnavigation_->publish(nmea_msg_);
    }

    void GnssInsDriver::rtcm_callback(const mavros_msgs::msg::RTCM::SharedPtr msg)
    {
        rtcm_status_.received_size_ += msg->data.size();
        if(Converter::is_ins_active(ins_pva_.ins_status) != true)
        {
            return;
        }
        try
        {
            serial_port_ptr_->write(reinterpret_cast<const char *>(msg->data.data()), msg->data.size());
            rtcm_status_.transmitted_size_ += msg->data.size();
        }
        catch (const SerialPortException &e)
        {
            RCLCPP_ERROR(get_logger(), e.what());
        }

        auto rtcm_status_msg = rbf_gnss_ins_driver::msg::RTCMStatus();
        rtcm_status_msg.header.stamp = this->now();
        rtcm_status_msg.received_msg_size = rtcm_status_.received_size_;
        rtcm_status_msg.transmitted_msg_size = rtcm_status_.transmitted_size_;
        pub_rtcm_status_->publish(rtcm_status_msg);
    }

    void GnssInsDriver::timer_callback()
    {
        uint8_t buffer[512];
        try
        {
            int len = serial_port_ptr_->read(reinterpret_cast<char *>(buffer), sizeof(buffer));
            if (len > 0)
            {
                gnss_parser_->parse(buffer, len);
            }
        }
        catch (const SerialPortException &e)
        {
            RCLCPP_ERROR(get_logger(), e.what());
        }
    }

    void GnssInsDriver::diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        converter_->is_delay_high(gnss_parser_->get_unix_time_ns()) ? stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "HIGH_DELAY") : stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
        stat.add("INS_STATUS", Converter::is_ins_active(ins_pva_.ins_status) ? "ACTIVE" : "INACTIVE");
    }

} // namespace rbf_gnss_ins_driver

// Register the node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rbf_gnss_ins_driver::GnssInsDriver)
