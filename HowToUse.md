
# Teaching
roscore
roslaunch follower_controller follower_bringup.launch
roslaunch leader_controller leader_bringup.launch
roslaunch aloha_camera camera.launch


# Motion Generation
roscore
roslaunch follower_controller follower_bringup.launch leader_freq:=10
roslaunch aloha_camera camera.launch