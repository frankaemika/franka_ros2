# Robot mode constants
uint8 ROBOT_MODE_OTHER=0
uint8 ROBOT_MODE_IDLE=1
uint8 ROBOT_MODE_MOVE=2
uint8 ROBOT_MODE_GUIDING=3
uint8 ROBOT_MODE_REFLEX=4
uint8 ROBOT_MODE_USER_STOPPED=5
uint8 ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY=6

std_msgs/Header header

# Indicates which contact level is activated in which Cartesian dimension (x,y,z,R,Y,P).
# After contact disappears, the value stays the same until a reset command is sent.
float64[6] cartesian_collision

# Indicates which contact level is activated in which Cartesian dimension (x,y,z,R,Y,P).
# After contact disappears, the value turns to zero.
float64[6] cartesian_contact

# Measured joint position. Unit: rad.
float64[7] q

# Desired joint position. Unit: rad.
float64[7] q_d

# Measured joint velocity. Unit: rad/s.
float64[7] dq

# Desired joint velocity. Unit: rad/s.
float64[7] dq_d

 # Desired joint acceleration. Unit: rad/(s^2).
float64[7] ddq_d

# Motor position. Unit: rad.
float64[7] theta

# Motor velocity. Unit: rad/s.
float64[7] dtheta

# Measured link-side joint torque sensor signals. Unit: Nm.
float64[7] tau_j

# Derivative of measured link-side joint torque sensor signals. Unit: Nm/s.
float64[7] dtau_j

# Desired link-side joint torque sensor signals without gravity. Unit: Nm.
float64[7] tau_j_d

# Estimated external wrench (force, torque) acting on stiffness frame,
# expressed relative to the stiffness frame.
geometry_msgs/WrenchStamped k_f_ext_hat_k

# Elbow configuration.
float64[2] elbow

# Desired elbow configuration.
float64[2] elbow_d

# Commanded elbow configuration.
float64[2] elbow_c

# Commanded elbow velocity.
float64[2] delbow_c

# Commanded elbow acceleration.
float64[2] ddelbow_c

# Indicates which contact level is activated in which joint.
# After contact disappears, the value stays the same until a reset command is sent.
float64[7] joint_collision

# Indicates which contact level is activated in which joint. After contact disappears, value turns to zero.
float64[7] joint_contact

# Estimated external wrench acting on stiffness frame, expressed relative to the base frame.
geometry_msgs/WrenchStamped o_f_ext_hat_k

# Desired end effector twist in base frame.
geometry_msgs/TwistStamped o_dp_ee_d

# Last commanded end effector twist in base frame.
geometry_msgs/TwistStamped o_dp_ee_c

# Last commanded end effector acceleration in base frame.
geometry_msgs/AccelStamped o_ddp_ee_c

# External torque, filtered. Unit: Nm.
float64[7] tau_ext_hat_filtered

# Inertia of the end effector load with respect to center of mass.
geometry_msgs/InertiaStamped ee

# Inertia of the external load with respect to center of mass.
geometry_msgs/InertiaStamped load

# Combined inertia of the end effector load and
# the external load with respect to the center of mass.
geometry_msgs/InertiaStamped total

# Measured end effector pose in base frame.
geometry_msgs/TransformStamped o_t_ee

# Last desired end effector pose of motion generation in base frame.
geometry_msgs/TransformStamped o_t_ee_d

# Last commanded end effector pose of motion generation in base frame.
geometry_msgs/TransformStamped o_t_ee_c

# End effector frame pose in flange frame.
geometry_msgs/TransformStamped f_t_ee

# Nominal end effector frame pose in flange frame.
geometry_msgs/TransformStamped f_t_ne

# End effector frame pose in nominal end effector frame.
geometry_msgs/TransformStamped ne_t_ee

# Stiffness frame pose in end effector frame.
geometry_msgs/TransformStamped ee_t_k

# Strictly monotonically increasing timestamp since robot start.
builtin_interfaces/Duration robot_time

# Percentage of the last 100 control commands that were successfully received by the robot.
float64 control_command_success_rate

# Current robot mode.
uint8 robot_mode

# Current error state.
Errors current_errors

# Contains the errors that aborted the previous motion.
Errors last_motion_errors