uint64 timestamp	# time since system start (microseconds)

float32[3] omega_r	# reference body rates [rad/s]
float32[3] omega_r_dot	# time derivative of omega_r [rad/s^2]
float32[3] beta_v	# attitude error vector part (geometric control)
float32[3] tau_s	# compensation torque tau_s from (31) [N*m]
