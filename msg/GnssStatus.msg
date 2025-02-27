std_msgs/Header header

#0 SOL_COMPUTED         Solution computed
#1 INSUFFICIENT_OBS     Insufficient observations
#2 NO_CONVERGENCE       No convergence
#4 COV_TRACE            Covariance trace exceeds maximum (trace > 1000 m)
uint32 sol_status

#1-51    Reserved
#52      INS                 INS position solution
#53      INS_PSRSP           Intergrated solution of INS and single point positioning – no DGPS correction
#54      INS_PSRDIFF         Intergrated solution of INS and DGPS
#55      INS_RTKFLOAT        Intergrated solution of INS and the RTK floating
#56      INS_RTKFIXED        Intergrated solution of INS and the RTK fixed
#57-74   Reserved
uint32 pos_type

uint8 number_of_satellite_tracked
uint8 number_of_satellite_used

#the age of the differential correction [s]
float32 diff_age