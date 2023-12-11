### Default parts of the message
std_msgs/Header header

### Indicates which dimensions have an active contact/collision flag raised
CollisionIndicators collision_indicators

### The state of the arm in joint space
# The joint state consisting out of position (q), velocity (dq) and effort (tau_J)
sensor_msgs/JointState measured_joint_state

# The desired joint state consisting out of position (q_d), velocity (dq_d) and effort (tau_J_d)
sensor_msgs/JointState desired_joint_state

# The measured motor state of the joints consisting out of position (theta) and velocity (dtheta)
sensor_msgs/JointState measured_joint_motor_state

# The desired joint acceleration
float64[7] ddq_d
# The derivative of the measured torque signal
float64[7] dtau_j
# Filtered external torque. The JointState consists out of effort (tau_ext_hat_filtered)
sensor_msgs/JointState tau_ext_hat_filtered

### The state of the elbow
Elbow elbow

### The active wrenches acting on the stiffness frame expressed relative to
# stiffness frame
geometry_msgs/WrenchStamped k_f_ext_hat_k
# base frame
geometry_msgs/WrenchStamped o_f_ext_hat_k

### The different inertias of the arm
# The end-effector inertia
geometry_msgs/InertiaStamped inertia_ee
# The load inertia
geometry_msgs/InertiaStamped inertia_load
# The total (end-effector + load) inertia
geometry_msgs/InertiaStamped inertia_total

### The poses describing the transformations between different frames of the arm
# Measured end-effector pose in base frame
geometry_msgs/PoseStamped o_t_ee
# Last desired end-effector pose of motion generation in base frame
geometry_msgs/PoseStamped o_t_ee_d
# Last commanded end-effector pose of motion generation in base frame
geometry_msgs/PoseStamped o_t_ee_c
# Flange to end-effector frame
geometry_msgs/PoseStamped f_t_ee
# End-effector to stiffness frame
geometry_msgs/PoseStamped ee_t_k

# Desired end effector twist in base frame
geometry_msgs/TwistStamped o_dp_ee_d
# Last commanded end effector twist in base frame
geometry_msgs/TwistStamped o_dp_ee_c
# Last commanded end effector acceleration in base frame
geometry_msgs/AccelStamped o_ddp_ee_c

### Additional information
float64 time
float64 control_command_success_rate

uint8 ROBOT_MODE_OTHER=0
uint8 ROBOT_MODE_IDLE=1
uint8 ROBOT_MODE_MOVE=2
uint8 ROBOT_MODE_GUIDING=3
uint8 ROBOT_MODE_REFLEX=4
uint8 ROBOT_MODE_USER_STOPPED=5
uint8 ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY=6
uint8 robot_mode

Errors current_errors
Errors last_motion_errors
