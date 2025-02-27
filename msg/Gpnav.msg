# msg/Gpnavigation.msg

std_msgs/Header header
string raw_sentence

# Time
string date         # yyyymmdd
string utc_time     # hhmmss.ss

# Position
float64 latitude
float64 longitude
float64 altitude
float64 separation

# Orientation
float64 tracking_angle
float64 heading
float64 pitch
float64 roll

# Velocities
float64 ve
float64 vn
float64 vu
float64 vg

# Status
int32 status1
string status2
int32 system_mask
float64 baseline

# Satellite info
int32 gps_used
int32 glo_used
int32 bds_used
int32 gps_tracked
int32 glo_tracked
int32 bds_tracked
