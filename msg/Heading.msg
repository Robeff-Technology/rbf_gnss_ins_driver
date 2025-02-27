std_msgs/Header header


# 0 SOLUTION_COMPUTED
# 1 INSUFFICIENT_OBS
# 2 NO_CONVERGENCE
# 3 COV_TRACE
uint16 status


#clockwise angle between True North and baseline vector from master antenna to the slave antenna
#True north heading angle(0 to 360 degree)
float32 heading

#standard deviation of calculation heading angle
float32 std_dev_heading

#pitch angle(-90 to +90 degree)
float32 pitch

#standard deviation of calculation pitch angle
float32 std_dev_pitch

#distance between master antenna and slave antenna in meter
float32 baseline_len

int32 base_station_id

uint8 num_sats_tracked

uint8 num_sats_in_used

uint8 num_sats_above_elevation_mask_angle

uint8 num_sats_above_elevation_mask_angle_l2

uint8 ext_sol_stat

uint8 signal_mask_gal_bds3

uint8 signal_mask_gps_glo_bds2