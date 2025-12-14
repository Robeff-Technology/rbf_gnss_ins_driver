
#ifndef STRUCTS_H
#define STRUCTS_H

#include <cstdint>

namespace rbf_gnss_ins_driver
{
#pragma pack(push, 1)
struct Header
{
  uint8_t synch_1{};
  uint8_t synch_2{};
  uint8_t synch_3{};
  uint8_t header_length{};
  uint16_t msg_id{};
  uint8_t msg_type{};
  uint8_t port_addr{};
  uint16_t msg_len{};
  uint16_t sequence{};
  uint8_t idle_time{};
  uint8_t time_stats{};
  uint16_t ref_week_num{};
  uint32_t week_ms{};
  uint32_t receiver_status{};
  uint16_t reserved{};
  uint16_t receiver_sw_ver{};
};

struct InsPvax
{
  uint32_t ins_status{};
  uint32_t pos_type{};
  double latitude{};
  double longitude{};
  double height{};
  float undulation{};
  double north_velocity{};
  double east_velocity{};
  double up_velocity{};
  double roll{};
  double pitch{};
  double azimuth{};
  float std_dev_latitude{};
  float std_dev_longitude{};
  float std_dev_height{};
  float std_dev_north_velocity{};
  float std_dev_east_velocity{};
  float std_dev_up_velocity{};
  float std_dev_roll{};
  float std_dev_pitch{};
  float std_dev_azimuth{};
  uint32_t extended_solution_stat{};
  uint16_t time_since_update{};
};

struct ImuData
{
  double pitch_rate;
  double roll_rate;
  double yaw_rate;
  double lateral_acc;
  double longitudinal_acc;
  double vertical_acc;
};

struct RawImu
{
  uint32_t gnss_week{};
  double seconds_into_week{};
  uint32_t imu_status{};
  int32_t z_accel_output{};
  int32_t y_accel_output{};
  int32_t x_accel_output{};
  int32_t z_gyro_output{};
  int32_t y_gyro_output{};
  int32_t x_gyro_output{};
};

struct RawImux
{
  uint8_t imu_error{};
  uint8_t imu_type{};
  uint16_t gnss_week{};
  double seconds_into_week{};
  uint32_t imu_status{};
  int32_t z_accel_output{};
  int32_t y_accel_output{};
  int32_t x_accel_output{};
  int32_t z_gyro_output{};
  int32_t y_gyro_output{};
  int32_t x_gyro_output{};
};

struct BestGnssVel
{
  uint32_t sol_status{};
  uint32_t vel_type{};
  float latency{};
  float age{};
  double horizontal_speed{};
  double track_angle{};
  double vertical_speed{};
  uint32_t reserved{};
};

struct BestGnssPos
{
  uint32_t sol_status{};
  uint32_t pos_type{};
  double latitude{};
  double longitude{};
  double height{};
  float undulation{};
  uint32_t datum_id{};
  float std_dev_latitude{};
  float std_dev_longitude{};
  float std_dev_height{};
  int32_t station_id{};
  float diff_age{};
  float solution_age{};
  uint8_t num_sats_tracked{};
  uint8_t num_sats_in_solution{};
  uint8_t reserved_1{};
  uint8_t reserved_2{};
  uint8_t reserved_3{};
  uint8_t ext_sol_stat{};
  uint8_t gal_beidou_sig_mask{};
  uint8_t gps_glonass_sig_mask{};
};

struct UniHeading
{
  uint32_t sol_status{};
  uint32_t pos_type{};
  float baseline_length{};
  float heading{};
  float pitch{};
  float reserved_float1{};
  float std_dev_heading{};
  float std_dev_pitch{};
  int32_t base_station_id{};
  uint8_t num_sats_tracked{};
  uint8_t num_sats_in_used{};
  uint8_t num_sats_above_elevation_mask_angle{};
  uint8_t num_sats_above_elevation_mask_angle_L2{};
  uint8_t reserved_char1{};
  uint8_t ext_sol_stat{};
  uint8_t signal_mask_gal_bds3{};
  uint8_t signal_mask_gps_glo_bds2{};
};

struct ECEF
{
  uint32_t sol_status;
  uint32_t pos_type;

  double pos_x;
  double pos_y;
  double pos_z;

  float std_pos_x;
  float std_pos_y;
  float std_pos_z;

  uint32_t v_sol_status;
  uint32_t vel_type;

  double vel_x;
  double vel_y;
  double vel_z;

  float std_vel_x;
  float std_vel_y;
  float std_vel_z;
};
#pragma pack(pop)
}  // namespace rbf_gnss_ins_driver
#endif  // STRUCTS_H
