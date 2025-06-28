# Dynamixel Teleop system

## Quick start guide
### Requirements
 - Ubuntu 20.04 + ROS 1 noetic

pip3 install -U pip
pip3 install dynamixel_sdk

## dynamixel serial ID setting

$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/dynamixel-workbench/master/99-dynamixel-workbench-cdc.rules
$ sudo cp ./99-dynamixel-workbench-cdc.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger




Step 1: Connect 4 robots to the computer via USB, and power on. *Do not use extension cable or usb hub*.
- To check if the robot is connected, install dynamixel wizard [here](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)
- Dynamixel wizard is a very helpful debugging tool that connects to individual motors of the robot. It allows
things such as rebooting the motor (very useful!), torque on/off, and sending commands.
However, it has no knowledge about the kinematics of the robot, so be careful about collisions.
The robot *will* collapse if motors are torque off i.e. there is no automatically engaged brakes in joints.
- Open Dynamixel wizard, go into ``options`` and select:
  - Protocal 2.0
  - All ports
  - 1000000 bps
  - ID range from 0-10
- Note: repeat above everytime before you scan.
- Then hit ``Scan``. There should be 4 devices showing up, each with 9 motors.


- One issue that arises is the port each robot binds to can change over time, e.g. a robot that
is initially ``ttyUSB0`` might suddenly become ``ttyUSB5``. To resolve this, we bind each robot to a fixed symlink
port with the following mapping:
  - ``ttyDXL_master_right``: right master robot (master: the robot that the operator would be holding)
  - ``ttyDXL_puppet_right``: right puppet robot (puppet: the robot that performs the task)
  - ``ttyDXL_master_left``: left master robot
  - ``ttyDXL_puppet_left``: left puppet robot
- Take ``ttyDXL_master_right``: right master robot as an example:
  1. Find the port that the right master robot is currently binding to, e.g. ``ttyUSB0``
  2. run ``udevadm info --name=/dev/ttyUSB0 --attribute-walk | grep serial`` to obtain the serial number. Use the first one that shows up, the format should look similar to ``FT6S4DSP``.
  3. ``sudo vim /etc/udev/rules.d/99-fixed-interbotix-udev.rules`` and add the following line: 

         SUBSYSTEM=="tty", ATTRS{serial}=="<serial number here>", ENV{ID_MM_DEVICE_IGNORE}="1", ATTR{device/latency_timer}="1", SYMLINK+="ttyDXL_master_right"

  4. This will make sure the right master robot is *always* binding to ``ttyDXL_master_right``
  5. Repeat with the rest of 3 arms.
- To apply the changes, run ``sudo udevadm control --reload && sudo udevadm trigger``
- If successful, you should be able to find ``ttyDXL*`` in your ``/dev``





Find the port that the right master robot is currently binding to, e.g. ttyUSB0

run udevadm info --name=/dev/ttyUSB0 --attribute-walk | grep serial to obtain the serial number. Use the first one that shows up, the format should look similar to FT6S4DSP.

sudo vim /etc/udev/rules.d/99-fixed-interbotix-udev.rules and add the following line:

> SUBSYSTEM=="tty", ATTRS{serial}=="<serial number here>", ENV{ID_MM_DEVICE_IGNORE}="1", ATTR{device/latency_timer}="1", SYMLINK+="ttyDXL_master_right"



# cd ~catkin_ws/src/
# git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
# cd ../
 catkin build







ito@X1-Extreme:~$ roslaunch leader_controller leader_bringup.launch 
... logging to /home/ito/.ros/log/24a58960-53f7-11f0-826d-19bfa58791db/roslaunch-X1-Extreme-26637.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://X1-Extreme:40721/

SUMMARY
========

PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.16.0

NODES
  /
    leader_node (leader_controller/leader_node.py)
    load_config_node_X1_Extreme_26637_4282705886319671877 (robot_description/load_config.py)

ROS_MASTER_URI=http://localhost:11311

process[load_config_node_X1_Extreme_26637_4282705886319671877-1]: started with pid [26651]
process[leader_node-2]: started with pid [26652]
[load_config_node_X1_Extreme_26637_4282705886319671877-1] process has finished cleanly
log file: /home/ito/.ros/log/24a58960-53f7-11f0-826d-19bfa58791db/load_config_node_X1_Extreme_26637_4282705886319671877-1*.log
[WARN] [1751098147.543844]: Initialize



ito@X1-Extreme:~$ roslaunch follower_controller follower_bringup.launch 
... logging to /home/ito/.ros/log/24a58960-53f7-11f0-826d-19bfa58791db/roslaunch-X1-Extreme-26866.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://X1-Extreme:41333/

SUMMARY
========

PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.16.0

NODES
  /
    follower_node (follower_controller/follower_node.py)
    interpolation_node (follower_controller/interpolation_node.py)
    load_config_node_X1_Extreme_26866_4334606723929592779 (robot_description/load_config.py)

ROS_MASTER_URI=http://localhost:11311

process[load_config_node_X1_Extreme_26866_4334606723929592779-1]: started with pid [26880]
process[follower_node-2]: started with pid [26881]
process[interpolation_node-3]: started with pid [26882]
[load_config_node_X1_Extreme_26866_4334606723929592779-1] process has finished cleanly
log file: /home/ito/.ros/log/24a58960-53f7-11f0-826d-19bfa58791db/load_config_node_X1_Extreme_26866_4334606723929592779-1*.log
[WARN] [1751098260.878750]: OnlineExecutor.run(): Recieved first data
[WARN] [1751098261.067711]: Initialize
[WARN] [1751098261.790308]: OnlineExecutor.run(): Starting execution
[WARN] [1751098261.791096]: OnlineExecutor.run(): Wait for first target







ito@X1-Extreme:~$ rostopic hz /leader/joint_states 
subscribed to [/leader/joint_states]
average rate: 10.000
        min: 0.100s max: 0.100s std dev: 0.00005s window: 10
average rate: 10.001
        min: 0.100s max: 0.100s std dev: 0.00005s window: 20
average rate: 10.000
        min: 0.100s max: 0.100s std dev: 0.00005s window: 30
average rate: 10.000
        min: 0.100s max: 0.100s std dev: 0.00005s window: 40
ito@X1-Extreme:~$ rostopic hz /leader/online_joint_states 
subscribed to [/leader/online_joint_states]
average rate: 99.991
        min: 0.010s max: 0.010s std dev: 0.00006s window: 100
average rate: 100.003
        min: 0.010s max: 0.010s std dev: 0.00006s window: 200
average rate: 99.999
        min: 0.010s max: 0.010s std dev: 0.00006s window: 300
average rate: 99.999
        min: 0.010s max: 0.010s std dev: 0.00006s window: 400
average rate: 99.999
        min: 0.010s max: 0.010s std dev: 0.00006s window: 500
ito@X1-Extreme:~$ rostopic hz /follower/joint_states
subscribed to [/follower/joint_states]
average rate: 99.966
        min: 0.010s max: 0.010s std dev: 0.00019s window: 100
average rate: 99.998
        min: 0.010s max: 0.010s std dev: 0.00018s window: 200
average rate: 100.001
        min: 0.010s max: 0.011s std dev: 0.00017s window: 300
average rate: 99.998
        min: 0.009s max: 0.011s std dev: 0.00020s window: 401
average rate: 100.001
        min: 0.009s max: 0.011s std dev: 0.00026s window: 501

    