# Message-Type for republishing the actual state of the robot
#currently, a maximum of 8 dofs under control is presumed to avoid allocation overhead
time stamp
int32 mode
int32 dofs
float64[8] q
float64[8] dq
float64[8] ddq
float64[8] tau
float64[8] tau_ext
float64[8] qd
float64[8] dqd

float64[16] ee_htransform_base
float64[6]  ee_wrench_ee
float64[42] ee_jacobian_ee
float64[42] ee_dotjacobian_ee




