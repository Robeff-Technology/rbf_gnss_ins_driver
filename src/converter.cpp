#include "rbf_gnss_ins_driver/converter.h"
#include <cmath>
#include <rclcpp/clock.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace rbf_gnss_ins_driver
{
    constexpr double accel_scale_factor = 0.000000186;
    constexpr double hz_to_second = 100;
    constexpr double gyro_scale_factor = 0.000001006;

    std_msgs::msg::Header Converter::create_header(std::string frame_id)
    {
        std_msgs::msg::Header header;
        if (use_ros_time_)
        {
            header.stamp = rclcpp::Clock().now();
        }
        else if (is_delay_high(timestamp_))
        {
            header.stamp = rclcpp::Clock().now();
        }
        else
        {
            header.stamp = rclcpp::Time(timestamp_);
        }
        header.frame_id = std::move(frame_id);
        return header;
    }

    double Converter::degree_to_radian(double degree)
    {
        return degree * M_PI / 180.0;
    }

    bool Converter::is_delay_high(int64_t timestamp)
    {
        return ((rclcpp::Clock().now().nanoseconds() - timestamp) > 100000000 || (rclcpp::Clock().now().nanoseconds() - timestamp) < 0);
    }

    double Converter::calc_imu_temperature(const RawImux &raw_imux)
    {
        auto raw_temperature = static_cast<int16_t>((raw_imux.imu_status >> 16U) & 0xFFFFU);
        return static_cast<double>(raw_temperature) * 0.1;
    }

    double Converter::raw_gyro_to_deg_s(int32_t raw_gyro)
    {
        return static_cast<double>(raw_gyro) * gyro_scale_factor * hz_to_second;
    }

    double Converter::raw_acc_to_m_s2(int32_t raw_acc)
    {
        return static_cast<double>(raw_acc) * accel_scale_factor * hz_to_second;
        ;
    }

    rbf_gnss_ins_driver::msg::ImuStatus Converter::raw_imu_to_imu_status(const RawImux &raw_imux, std::string frame_id)
    {
        rbf_gnss_ins_driver::msg::ImuStatus imu;
        imu.header = create_header(std::move(frame_id));

        imu.status = raw_imux.imu_status & 0x0000FFFFU;
        return imu;
    }

    rbf_gnss_ins_driver::msg::Heading Converter::heading_to_msg(const UniHeading &heading, std::string frame_id)
    {
        rbf_gnss_ins_driver::msg::Heading heading_msg;
        heading_msg.header = create_header(std::move(frame_id));
        heading_msg.sol_status = heading.sol_status;
        heading_msg.pos_type = heading.pos_type;
        heading_msg.heading = heading.heading;
        heading_msg.std_dev_heading = heading.std_dev_heading;
        heading_msg.pitch = heading.pitch;
        heading_msg.std_dev_pitch = heading.std_dev_pitch;
        heading_msg.baseline_len = heading.baseline_length;
        heading_msg.base_station_id = heading.base_station_id;
        heading_msg.num_sats_tracked = heading.num_sats_tracked;
        heading_msg.num_sats_in_used = heading.num_sats_in_used;
        heading_msg.num_sats_above_elevation_mask_angle = heading.num_sats_above_elevation_mask_angle;
        heading_msg.num_sats_above_elevation_mask_angle_l2 = heading.num_sats_above_elevation_mask_angle_L2;
        heading_msg.ext_sol_stat = heading.ext_sol_stat;
        heading_msg.signal_mask_gal_bds3 = heading.signal_mask_gal_bds3;
        heading_msg.signal_mask_gps_glo_bds2 = heading.signal_mask_gps_glo_bds2;
        return heading_msg;
    }

    rbf_gnss_ins_driver::msg::GnssStatus Converter::gnss_pos_to_gnss_status_msg(const BestGnssPos &gnss_pos, std::string frame_id)
    {
        rbf_gnss_ins_driver::msg::GnssStatus gnss_status_msg;
        gnss_status_msg.header = create_header(std::move(frame_id));
        gnss_status_msg.sol_status = gnss_pos.sol_status;
        gnss_status_msg.pos_type = gnss_pos.pos_type;
        gnss_status_msg.number_of_satellite_tracked = gnss_pos.num_sats_tracked;
        gnss_status_msg.number_of_satellite_used = gnss_pos.num_sats_in_solution;
        gnss_status_msg.diff_age = gnss_pos.diff_age;
        return gnss_status_msg;
    }

    rbf_gnss_ins_driver::msg::GnssVel Converter::gnss_vel_to_msg(const BestGnssVel &gnss_vel, std::string frame_id)
    {
        rbf_gnss_ins_driver::msg::GnssVel gnss_vel_msg;
        gnss_vel_msg.header = create_header(std::move(frame_id));
        gnss_vel_msg.sol_status = gnss_vel.sol_status;
        gnss_vel_msg.vel_type = gnss_vel.vel_type;
        gnss_vel_msg.latency = gnss_vel.latency;
        gnss_vel_msg.diff_age = gnss_vel.age;
        gnss_vel_msg.hor_spd = gnss_vel.horizontal_speed;
        gnss_vel_msg.trk_gnd = gnss_vel.track_angle;
        gnss_vel_msg.vert_spd = gnss_vel.vertical_speed;
        return gnss_vel_msg;
    }

    rbf_gnss_ins_driver::msg::Ins Converter::ins_to_msg(const InsPvax &ins_pva, std::string frame_id)
    {
        rbf_gnss_ins_driver::msg::Ins ins_msg;
        ins_msg.header = create_header(std::move(frame_id));
        ins_msg.ins_status = ins_pva.ins_status;
        ins_msg.pos_type = ins_pva.pos_type;
        ins_msg.latitude = ins_pva.latitude;
        ins_msg.longitude = ins_pva.longitude;
        ins_msg.height = ins_pva.height;
        ins_msg.undulation = ins_pva.undulation;
        ins_msg.north_velocity = ins_pva.north_velocity;
        ins_msg.east_velocity = ins_pva.east_velocity;
        ins_msg.up_velocity = ins_pva.up_velocity;
        ins_msg.roll = ins_pva.roll;
        ins_msg.pitch = ins_pva.pitch;
        ins_msg.azimuth = ins_pva.azimuth;
        ins_msg.std_dev_latitude = ins_pva.std_dev_latitude;
        ins_msg.std_dev_longitude = ins_pva.std_dev_longitude;
        ins_msg.std_dev_height = ins_pva.std_dev_height;
        ins_msg.std_dev_north_velocity = ins_pva.std_dev_north_velocity;
        ins_msg.std_dev_east_velocity = ins_pva.std_dev_east_velocity;
        ins_msg.std_dev_up_velocity = ins_pva.std_dev_up_velocity;
        ins_msg.std_dev_roll = ins_pva.std_dev_roll;
        ins_msg.std_dev_pitch = ins_pva.std_dev_pitch;
        ins_msg.std_dev_azimuth = ins_pva.std_dev_azimuth;
        ins_msg.extended_solution_stat = ins_pva.extended_solution_stat;
        ins_msg.time_since_update = ins_pva.time_since_update;
        return ins_msg;
    }

    sensor_msgs::msg::Imu Converter::raw_imu_to_imu_msg(const RawImux &raw_imux, std::string frame_id)
    {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header = create_header(std::move(frame_id));
        imu_msg.linear_acceleration.x = raw_acc_to_m_s2(raw_imux.x_accel_output);
        imu_msg.linear_acceleration.y = -1.0 * raw_acc_to_m_s2(raw_imux.y_accel_output);
        imu_msg.linear_acceleration.z = raw_acc_to_m_s2(raw_imux.z_accel_output);
        imu_msg.angular_velocity.x = degree_to_radian(raw_gyro_to_deg_s(raw_imux.x_gyro_output));
        imu_msg.angular_velocity.y = -1.0 * degree_to_radian(raw_gyro_to_deg_s(raw_imux.y_gyro_output));
        imu_msg.angular_velocity.z = degree_to_radian(raw_gyro_to_deg_s(raw_imux.z_gyro_output));
        imu_msg.orientation_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        imu_msg.linear_acceleration_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        imu_msg.angular_velocity_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        return imu_msg;
    }

    sensor_msgs::msg::Imu Converter::imu_data_to_imu_msg(const ImuData &imu_data, std::string frame_id)
    {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header = create_header(std::move(frame_id));
        imu_msg.linear_acceleration.x = imu_data.lateral_acc;
        imu_msg.linear_acceleration.y = imu_data.longitudinal_acc;
        imu_msg.linear_acceleration.z = imu_data.vertical_acc;
        imu_msg.angular_velocity.x = imu_data.pitch_rate;
        imu_msg.angular_velocity.y = imu_data.roll_rate;
        imu_msg.angular_velocity.z = imu_data.yaw_rate;
        imu_msg.orientation_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        imu_msg.linear_acceleration_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        imu_msg.angular_velocity_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        return imu_msg;
    }

    sensor_msgs::msg::Temperature Converter::raw_imu_to_temperature_msg(const RawImux &raw_imux, std::string frame_id)
    {
        sensor_msgs::msg::Temperature temperature_msg;
        temperature_msg.header = create_header(std::move(frame_id));
        temperature_msg.temperature = calc_imu_temperature(raw_imux);
        return temperature_msg;
    }

    sensor_msgs::msg::NavSatFix Converter::gnss_pos_to_nav_sat_fix_msg(const BestGnssPos &gnss_pos, std::string frame_id)
    {
        sensor_msgs::msg::NavSatFix nav_sat_fix_msg;
        nav_sat_fix_msg.header = create_header(std::move(frame_id));
        if (gnss_pos.pos_type >= 32)
        {
            nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        }
        else if (gnss_pos.pos_type == 18)
        {
            nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        }
        else if (gnss_pos.pos_type == 16)
        {
            nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
        }
        else
        {
            nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        }
        nav_sat_fix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        nav_sat_fix_msg.latitude = gnss_pos.latitude;
        nav_sat_fix_msg.longitude = gnss_pos.longitude;

        if (altitude_mode_ == AltitudeMode::ORTHOMETRIC)
        {
            nav_sat_fix_msg.altitude = gnss_pos.height + gnss_pos.undulation;
        }
        else
        {
            nav_sat_fix_msg.altitude = gnss_pos.height;
        }

        nav_sat_fix_msg.position_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        nav_sat_fix_msg.position_covariance[0] = gnss_pos.std_dev_latitude * gnss_pos.std_dev_latitude;
        nav_sat_fix_msg.position_covariance[4] = gnss_pos.std_dev_longitude * gnss_pos.std_dev_longitude;
        nav_sat_fix_msg.position_covariance[8] = gnss_pos.std_dev_height * gnss_pos.std_dev_height;

        nav_sat_fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        return nav_sat_fix_msg;
    }

    rbf_gnss_ins_driver::msg::ECEF Converter::ecef_to_msg(const ECEF &ecef, std::string frame_id)
    {
        rbf_gnss_ins_driver::msg::ECEF ecef_msg;
        ecef_msg.header = create_header(std::move(frame_id));
        ecef_msg.sol_status = ecef.sol_status;
        ecef_msg.pos_type = ecef.pos_type;

        ecef_msg.pos_x = ecef.pos_x;
        ecef_msg.pos_y = ecef.pos_y;
        ecef_msg.pos_z = ecef.pos_z;

        ecef_msg.std_pos_x = ecef.std_pos_x;
        ecef_msg.std_pos_y = ecef.std_pos_y;
        ecef_msg.std_pos_z = ecef.std_pos_z;

        ecef_msg.v_sol_status = ecef.v_sol_status;
        ecef_msg.vel_type = ecef.vel_type;

        ecef_msg.vel_x = ecef.vel_x;
        ecef_msg.vel_y = ecef.vel_y;
        ecef_msg.vel_z = ecef.vel_z;

        ecef_msg.std_vel_x = ecef.std_vel_x;
        ecef_msg.std_vel_y = ecef.std_vel_y;
        ecef_msg.std_vel_z = ecef.std_vel_z;
        return ecef_msg;
    }

    geometry_msgs::msg::TwistWithCovarianceStamped Converter::ecef_to_twist_msg(const ECEF &ecef, const RawImux &raw_imux, std::string frame_id)
    {
        geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
        twist_msg.header = create_header(std::move(frame_id));
        twist_msg.twist.twist.linear.x = ecef.vel_x;
        twist_msg.twist.twist.linear.y = ecef.vel_y;
        twist_msg.twist.twist.linear.z = ecef.vel_z;
        twist_msg.twist.twist.angular.x = degree_to_radian(raw_gyro_to_deg_s(raw_imux.x_gyro_output));
        twist_msg.twist.twist.angular.y = -1.0 * degree_to_radian(raw_gyro_to_deg_s(raw_imux.y_gyro_output));
        twist_msg.twist.twist.angular.z = degree_to_radian(raw_gyro_to_deg_s(raw_imux.z_gyro_output));

        twist_msg.twist.covariance[0] = ecef.std_vel_x * ecef.std_vel_x;
        twist_msg.twist.covariance[7] = ecef.std_vel_y * ecef.std_vel_y;
        twist_msg.twist.covariance[14] = ecef.std_vel_z * ecef.std_vel_z;
        return twist_msg;
    }

    sensor_msgs::msg::NavSatFix Converter::ins_to_nav_sat_fix_msg(const InsPvax &ins_pva, std::string frame_id)
    {
        sensor_msgs::msg::NavSatFix nav_sat_fix_msg;
        nav_sat_fix_msg.header = create_header(std::move(frame_id));

        if (ins_pva.pos_type == 56 || ins_pva.pos_type == 55)
        {
            nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        }
        else if (ins_pva.pos_type == 54)
        {
            nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        }
        else if (ins_pva.pos_type < 54 && ins_pva.pos_type >= 52)
        {
            nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
        }
        else
        {
            nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        }

        nav_sat_fix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        nav_sat_fix_msg.latitude = ins_pva.latitude;
        nav_sat_fix_msg.longitude = ins_pva.longitude;
        if (altitude_mode_ == AltitudeMode::ORTHOMETRIC)
        {
            nav_sat_fix_msg.altitude = ins_pva.height + ins_pva.undulation;
        }
        else
        {
            nav_sat_fix_msg.altitude = ins_pva.height;
        }
        nav_sat_fix_msg.position_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        nav_sat_fix_msg.position_covariance[0] = ins_pva.std_dev_latitude * ins_pva.std_dev_latitude;
        nav_sat_fix_msg.position_covariance[4] = ins_pva.std_dev_longitude * ins_pva.std_dev_longitude;
        nav_sat_fix_msg.position_covariance[8] = ins_pva.std_dev_height * ins_pva.std_dev_height;
        nav_sat_fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        return nav_sat_fix_msg;
    }

    sensor_msgs::msg::Imu Converter::ins_to_imu_msg(const InsPvax &ins_pva, const RawImux &raw_imux, std::string frame_id)
    {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header = create_header(std::move(frame_id));
        tf2::Quaternion q;
        /*
         * in clap b7 roll-> y-axis pitch-> x axis azimuth->left-handed rotation around z-axis
         * in ros imu msg roll-> x-axis pitch-> y axis azimuth->right-handed rotation around z-axis
         */
        q.setRPY(degree_to_radian(ins_pva.pitch), degree_to_radian(ins_pva.roll), degree_to_radian(-ins_pva.azimuth));

        imu_msg.orientation.w = q.getW();
        imu_msg.orientation.x = q.getX();
        imu_msg.orientation.y = q.getY();
        imu_msg.orientation.z = q.getZ();
        imu_msg.linear_acceleration.x = raw_acc_to_m_s2(raw_imux.x_accel_output);
        imu_msg.linear_acceleration.y = -1.0 * raw_acc_to_m_s2(raw_imux.y_accel_output);
        imu_msg.linear_acceleration.z = raw_acc_to_m_s2(raw_imux.z_accel_output);
        imu_msg.angular_velocity.x = degree_to_radian(raw_gyro_to_deg_s(raw_imux.x_gyro_output));
        imu_msg.angular_velocity.y = -1.0 * degree_to_radian(raw_gyro_to_deg_s(raw_imux.y_gyro_output));
        imu_msg.angular_velocity.z = degree_to_radian(raw_gyro_to_deg_s(raw_imux.z_gyro_output));
        imu_msg.orientation_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        imu_msg.linear_acceleration_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        imu_msg.angular_velocity_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

        imu_msg.orientation_covariance[0] = ins_pva.std_dev_pitch * ins_pva.std_dev_pitch;
        imu_msg.orientation_covariance[4] = ins_pva.std_dev_roll * ins_pva.std_dev_roll;
        imu_msg.orientation_covariance[8] = ins_pva.std_dev_azimuth * ins_pva.std_dev_azimuth;

        return imu_msg;
    }

    nav_msgs::msg::Odometry Converter::convert_to_odometry_msg(const InsPvax &ins_pva, const RawImux &raw_imux, double x, double y, double z, std::string frame_id)
    {
        nav_msgs::msg::Odometry odometry_msg;
        odometry_msg.header = create_header(std::move(frame_id));
        odometry_msg.child_frame_id = "base_link";
        odometry_msg.pose.pose.position.x = x;
        odometry_msg.pose.pose.position.y = y;
        odometry_msg.pose.pose.position.z = z;
        tf2::Quaternion q;
        q.setRPY(degree_to_radian(ins_pva.pitch), degree_to_radian(ins_pva.roll), degree_to_radian(-ins_pva.azimuth));
        odometry_msg.pose.pose.orientation.w = q.getW();
        odometry_msg.pose.pose.orientation.x = q.getX();
        odometry_msg.pose.pose.orientation.y = q.getY();
        odometry_msg.pose.pose.orientation.z = q.getZ();

        odometry_msg.twist.twist.linear.x = ins_pva.east_velocity;
        odometry_msg.twist.twist.linear.y = ins_pva.north_velocity;
        odometry_msg.twist.twist.linear.z = ins_pva.up_velocity;

        odometry_msg.twist.twist.angular.x = degree_to_radian(raw_gyro_to_deg_s(raw_imux.x_gyro_output));
        odometry_msg.twist.twist.angular.y = -1.0 * degree_to_radian(raw_gyro_to_deg_s(raw_imux.y_gyro_output));
        odometry_msg.twist.twist.angular.z = degree_to_radian(raw_gyro_to_deg_s(raw_imux.z_gyro_output));

        odometry_msg.twist.covariance[0 * 6 + 0] = ins_pva.std_dev_east_velocity * ins_pva.std_dev_east_velocity;
        odometry_msg.twist.covariance[1 * 6 + 1] = ins_pva.std_dev_north_velocity * ins_pva.std_dev_north_velocity;
        odometry_msg.twist.covariance[2 * 6 + 2] = ins_pva.std_dev_up_velocity * ins_pva.std_dev_up_velocity;
        odometry_msg.twist.covariance[3 * 6 + 3] = 0.00;
        odometry_msg.twist.covariance[4 * 6 + 4] = 0.00;
        odometry_msg.twist.covariance[5 * 6 + 5] = 0.00;
        return odometry_msg;
    }

    geometry_msgs::msg::TransformStamped Converter::create_transform(const geometry_msgs::msg::Pose &pose, std::string frame_id)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = rclcpp::Clock().now();
        transform.header.frame_id = frame_id;
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = pose.position.x;
        transform.transform.translation.y = pose.position.y;
        transform.transform.translation.z = pose.position.z;
        transform.transform.rotation.x = pose.orientation.x;
        transform.transform.rotation.y = pose.orientation.y;
        transform.transform.rotation.z = pose.orientation.z;
        transform.transform.rotation.w = pose.orientation.w;
        return transform;
    }

    geometry_msgs::msg::TwistWithCovarianceStamped Converter::ins_to_twist_msg(const InsPvax &ins_pva, const RawImux &raw_imux, std::string frame_id)
    {
        geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
        twist_msg.header = create_header(std::move(frame_id));
        twist_msg.twist.twist.linear.x = ins_pva.east_velocity;
        twist_msg.twist.twist.linear.y = ins_pva.north_velocity;
        twist_msg.twist.twist.linear.z = ins_pva.up_velocity;
        twist_msg.twist.twist.angular.x = degree_to_radian(raw_gyro_to_deg_s(raw_imux.x_gyro_output));
        twist_msg.twist.twist.angular.y = -1.0 * degree_to_radian(raw_gyro_to_deg_s(raw_imux.y_gyro_output));
        twist_msg.twist.twist.angular.z = degree_to_radian(raw_gyro_to_deg_s(raw_imux.z_gyro_output));

        twist_msg.twist.covariance[0] = ins_pva.std_dev_east_velocity * ins_pva.std_dev_east_velocity;
        twist_msg.twist.covariance[7] = ins_pva.std_dev_north_velocity * ins_pva.std_dev_north_velocity;
        twist_msg.twist.covariance[14] = ins_pva.std_dev_up_velocity * ins_pva.std_dev_up_velocity;
        return twist_msg;
    }

    rbf_gnss_ins_driver::msg::Gpnav Converter::create_gpnavigation_msg(const std::string& sentence, const std::string& frame_id) {
        rbf_gnss_ins_driver::msg::Gpnav msg;
        msg.header.frame_id = frame_id;
        msg.raw_sentence = sentence;
    
        size_t asterisk_pos = sentence.find('*');
        std::string body = sentence.substr(1, asterisk_pos - 1); // skip '$' and exclude '*'
    
        std::stringstream ss(body);
        std::string token;
        std::vector<std::string> fields;
    
        while (std::getline(ss, token, ',')) {
            fields.push_back(token);
        }
    
        if (fields.size() >= 37 && fields[0] == "GPNAV") {
            msg.date = fields[1];
            msg.utc_time = fields[2];
    
            msg.latitude = std::stod(fields[6]);
            msg.longitude = std::stod(fields[7]);
            msg.altitude = std::stod(fields[8]);
            msg.separation = std::stod(fields[9]);
    
            msg.tracking_angle = std::stod(fields[10]);
            msg.heading = std::stod(fields[11]);
            msg.pitch = std::stod(fields[12]);
            msg.roll = fields[13].empty() ? 0.0 : std::stod(fields[13]);
    
            msg.ve = std::stod(fields[14]);
            msg.vn = std::stod(fields[15]);
            msg.vu = std::stod(fields[16]);
            msg.vg = std::stod(fields[17]);
    
            msg.status1 = std::stoi(fields[18]);
            msg.status2 = fields[19];
            msg.system_mask = std::stoi(fields[20]);
            msg.baseline = std::stod(fields[21]);
    
            msg.gps_used = fields[22].empty() ? 0 : std::stoi(fields[22]);
            msg.glo_used = fields[23].empty() ? 0 : std::stoi(fields[23]);
            msg.bds_used = fields[24].empty() ? 0 : std::stoi(fields[24]);
    
            msg.gps_tracked = fields[27].empty() ? 0 : std::stoi(fields[27]);
            msg.glo_tracked = fields[28].empty() ? 0 : std::stoi(fields[28]);
            msg.bds_tracked = fields[29].empty() ? 0 : std::stoi(fields[29]);
        }
    
        return msg;
    }

    autoware_sensing_msgs::msg::GnssInsOrientationStamped Converter::ins_to_orientation_stamped_msg(const InsPvax& ins_pva, std::string frame_id) {
        autoware_sensing_msgs::msg::GnssInsOrientationStamped orientation_msg;
        orientation_msg.header = create_header(std::move(frame_id));
        
        tf2::Quaternion q;
        q.setRPY(degree_to_radian(ins_pva.pitch), degree_to_radian(ins_pva.roll), degree_to_radian(ins_pva.azimuth));
        orientation_msg.orientation.orientation.x = q.getX();
        orientation_msg.orientation.orientation.y = q.getY();
        orientation_msg.orientation.orientation.z = q.getZ();
        orientation_msg.orientation.orientation.w = q.getW();

        orientation_msg.orientation.rmse_rotation_x = ins_pva.std_dev_pitch * ins_pva.std_dev_pitch;
        orientation_msg.orientation.rmse_rotation_y = ins_pva.std_dev_roll * ins_pva.std_dev_roll;
        orientation_msg.orientation.rmse_rotation_z = ins_pva.std_dev_azimuth * ins_pva.std_dev_azimuth;

        return orientation_msg;
    }
} // namespace rbf_gnss_ins_driver
