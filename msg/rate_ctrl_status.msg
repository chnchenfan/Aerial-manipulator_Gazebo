uint64 timestamp		# time since system start (microseconds)

# rate controller integrator status
float32 rollspeed_integ
float32 pitchspeed_integ
float32 yawspeed_integ
float32 additional_integ1	# FW: wheel rate integrator (optional)

# ESO diagnostics. Populated by eso_rate_control, left zero by legacy controllers.
float32[3] eso_rate
float32[3] eso_rate_sp
float32[3] eso_rate_hat
float32[3] eso_rate_error
float32[3] eso_disturbance_hat
float32[3] eso_tau_s_raw
float32[3] eso_tau_s_used
float32[3] eso_omega_r_dot
float32[3] eso_torque
