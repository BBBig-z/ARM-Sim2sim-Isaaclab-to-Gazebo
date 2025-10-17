# Single motor/joint command
# Used for PD control with feedforward torque

float64 q      # Target position (rad)
float64 dq     # Target velocity (rad/s)
float64 tau    # Feedforward torque (Nm)
float64 kp     # Position gain
float64 kd     # Velocity gain

