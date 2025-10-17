# ARM Debug Status Message for real-time monitoring
# Published at high frequency for PlotJuggler visualization

std_msgs/Header header

# Target end-effector pose
geometry_msgs/Pose target_pose

# Actual end-effector pose
geometry_msgs/Pose actual_pose

# Position error (target - actual) in meters
float64 error_x
float64 error_y
float64 error_z
float64 error_norm  # sqrt(x^2 + y^2 + z^2)

# Orientation error in degrees
float64 error_roll
float64 error_pitch
float64 error_yaw

# Joint states (6 DOF)
float64[6] joint_positions      # radians
float64[6] joint_velocities     # rad/s
float64[6] joint_torques        # Nm

# Target joint positions
float64[6] target_joint_positions  # radians

# Joint tracking errors
float64[6] joint_position_errors   # radians

