Header header
# sensor_msgs/JointState joint_states

float32[3] pel_pos
float32[3] pel_vel
float32[3] pel_pos_des
float32[3] pel_vel_des
float32[3] pel_rot
float32[3] pel_omg
float32[4] pel_ref
float32[2] foot_pos
float32[7] obs_info 

float32 traj_time
int32 stance_leg
float32[3] left_toe_pos
float32[3] left_toe_pos_ref
float32[3] left_toe_vel
float32[3] left_toe_vel_ref
float32[3] right_toe_pos
float32[3] right_toe_pos_ref
float32[3] right_toe_vel
float32[3] right_toe_vel_ref

float32[12] torque
float32[12] torq
float32[2] contact
float32[3] pel_pos_actual

