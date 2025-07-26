#!/bin/bash                                                                     

rosbag record /cam_high/image_raw/compressed /cam_left_wrist/image_raw/compressed /cam_low/image_raw/compressed /cam_right_wrist/image_raw/compressed /left_arm/follower/joint_states /left_arm/follower/joint_states_unit /left_arm/leader/joint_states /left_arm/leader/online_joint_states /right_arm/follower/joint_states /right_arm/follower/joint_states_unit /right_arm/leader/joint_states /right_arm/leader/online_joint_states --duration $1
