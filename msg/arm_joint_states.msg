uint64 timestamp          # time since system start (microseconds)
float32[4] q              # Joint angles [rad] (Supports up to 4 DOF arm)
float32[4] dq             # Joint velocities [rad/s]
bool valid                # Validity flag
