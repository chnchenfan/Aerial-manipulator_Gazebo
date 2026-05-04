uint64 timestamp		# time since system start (microseconds)

float32 nav_roll
float32 nav_pitch
float32 nav_bearing 	# set to NAN if not valid

float32 target_bearing	# set to NAN if not valid
float32 xtrack_error	# set to NAN if not valid

float32 wp_dist

float32 acceptance_radius		# the optimal distance to a waypoint to switch to the next

float32 yaw_acceptance			# NaN if not set

float32 altitude_acceptance		# the optimal vertical distance to a waypoint to switch to the next

uint8 type

# ESO position-loop diagnostics. Populated by eso_pos_control, left zero by legacy publishers.
float32[3] eso_pos
float32[3] eso_pos_hat
float32[3] eso_pos_error
float32[3] eso_vel
float32[3] eso_vel_hat
float32[3] eso_vel_error
float32[3] eso_disturbance_hat
float32[3] eso_input
