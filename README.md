# emulate GPS

- GPS setHIL takes timestamp in milliseconds, hence you should update it in a frequency less than 5hz, to have reasonable amount of difference in timestamps for extrapolation. Hence, odom is published at 4 Hz  
- Files changed: 2 files from ardupilot, One mavros_vision plugin.  
- param set EKF_GPS_TYPE,2 COMPASS_USE,1 AHRS_GPS_GAIN,0.0  
- Use velocity from GPS == false in setHIL  
- All necessary packages added to this repo
- To reduce GT publish rate: measurement_divisor="100" measurement_delay="100" in albatross_spawn.launch
- Takes odometry position input from `/albatross/ground_truth/odometry` and sends MAVLINK msg to APM with emulated GPS.

maintained by: Manash Pratim Das (mpdmanash at iitkgp . ac . in)
