#!/usr/bin/env python3
import time
import numpy as np
from dynamixel_sdk import *
from dynamixel_driver.dynamixel_core import *
from dynamixel_driver.dynamixel_utils import *

# ROS
import rospy
from sensor_msgs.msg import JointState


class FollowerController(DynamixelCore):
    def __init__(self):
        self.ns = rospy.get_namespace()
        self.param = rospy.get_param(self.ns + "follower")
        self.baudrate = self.param["baudrate"]
        self.freq = self.param["control_freq"]
        self.r = rospy.Rate(self.freq)
        self.target_position = None

        # Initialize PortHandler/PacketHandler Structs
        self.port_handler = PortHandler(self.param["device"])
        self.packet_handler = PacketHandler(self.param["version"])
        self.readh = GroupSyncRead(
            self.port_handler, self.packet_handler, ADDR_PRESENT_CURRENT, 10
        )
        self.write_position = GroupSyncWrite(
            self.port_handler, self.packet_handler, ADDR_GOAL_POSITION, 4
        )

        # publisher and subscriber
        rospy.Subscriber(
            self.ns + "leader/online_joint_states", JointState, self.joint_callback
        )
        self.joint_pub = rospy.Publisher(
            self.ns + "follower/joint_states", JointState, queue_size=1
        )
        self.joint_pub_unit = rospy.Publisher(
            self.ns + "follower/joint_states_unit", JointState, queue_size=1
        )
        self.joint_msg = JointState()
        self.joint_msg_unit = JointState()

        # Setup
        self.serial_open()
        self.initialization()

    def initialization(self):
        self.joint_name = []
        self.joint_id = []
        self.joint_minmax = []
        self.home_position = []

        for p in self.param:
            if "arm/joint" in p:
                id = self.param[p]["id"]
                self.set_torque(id, False)
                self.joint_name.append(p)
                self.joint_id.append(id)
                self.joint_minmax.append(
                    [self.param[p]["min_position"], self.param[p]["max_position"]]
                )
                self.home_position.append(self.param[p]["home_position"])
                self.set_operating_mode(id, self.param[p]["operating_mode"])
                self.set_current_limit(id, self.param[p]["current_limit"])
                # self.set_position_kp_gain(id, self.param[p]["position_kp_gain"])
                self.set_delay_time(id, 0)
                self.set_torque(id, True)
                self.readh.addParam(id)

        self.joint_msg.name = self.joint_name
        self.joint_minmax = np.array(self.joint_minmax).T
        self.home_position = np.array(self.home_position)
        self.current_position = np.array(self.home_position)
        self.current_velocity = np.zeros(len(self.joint_id))
        self.current_torque = np.zeros(len(self.joint_id))

        self.move_to_goal_pose(self.home_position, 2)

    def serial_close(self):
        self.port_handler.closePort()
        rospy.loginfo("Close serial port")

    def clean_shutdown(self):
        if "left" in self.ns:
            sleep_pose = [2048, 841, 3090, 2048, 1650, 2048, 2100]
        else:
            sleep_pose = [2048, 884, 3090, 2048, 2430, 2048, 2100]

        self.move_to_goal_pose(sleep_pose, 2)
        for id in self.joint_id:
            self.set_torque(id, False)
        self.serial_close()

    def move_to_goal_pose(self, goal_pose, exptime):
        # initialize
        retp, _ = self.get_joint_states()
        if retp != 0:
            rospy.logerr("[initialize_robot_pose] Cannot receive current joint state.")
            exit()

        nloop = exptime * self.freq
        initialize_trajectory = np.linspace(
            self.current_position, goal_pose, nloop, dtype=np.int32
        )
        rospy.logwarn("Initialize")

        for loop_ct in range(nloop):
            for n, id in enumerate(self.joint_id):
                param_goal_position = self._sync_value(
                    initialize_trajectory[loop_ct, n]
                )

                retp = self.write_position.addParam(id, param_goal_position)
                if retp != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % id)
                    quit()

            # Syncwrite goal position
            retp = self.write_position.txPacket()
            self._validate(retp, 0)
            self.write_position.clearParam()
            self.r.sleep()

    def get_joint_states(self):
        retp = self.readh.txRxPacket()
        if retp == 0:
            for i, id in enumerate(self.joint_id):
                _position = self.readh.getData(id, 132, 4)
                if _position > int(0xFFFFFFFF / 2):
                    _position -= 0xFFFFFFFF
                self.current_position[i] = _position
                self.current_velocity[i] = self.readh.getData(id, 128, 4)
                self.current_torque[i] = self.readh.getData(id, 126, 2)

            # raw
            self.joint_msg.header.stamp = rospy.Time.now()
            self.joint_msg.position = np.array(self.current_position, dtype=np.int32)
            self.joint_msg.velocity = np.array(self.current_velocity, dtype=np.int32)
            self.joint_msg.effort = np.array(self.current_torque, dtype=np.int32)
            self.joint_pub.publish(self.joint_msg)

            # unit
            self.joint_msg_unit.header.stamp = rospy.Time.now()
            self.joint_msg_unit.position = convertValue2Radian(
                np.array(self.current_position)
            )
            self.joint_msg_unit.velocity = convertValue2Velocity(
                np.array(self.current_velocity)
            )
            self.joint_msg_unit.effort = convertValue2Current(
                np.array(self.current_torque)
            )
            self.joint_pub_unit.publish(self.joint_msg_unit)

        return retp, [self.current_position, self.current_velocity, self.current_torque]

    def set_joint_states(self):
        if self.target_position is not None:
            position = np.clip(
                self.target_position, self.joint_minmax[0], self.joint_minmax[1]
            )

            for n, id in enumerate(self.joint_id):
                param_goal_position = self._sync_value(position[n].astype(np.int32))
                retp = self.write_position.addParam(id, param_goal_position)
                if retp != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % id)
                    quit()

            # Syncwrite goal position
            retp = self.write_position.txPacket()
            self._validate(retp, 0)
            self.write_position.clearParam()

    def joint_callback(self, msg):
        self.target_position = np.array(msg.position, dtype=np.int32)

    def run(self):
        while not rospy.is_shutdown():
            self.get_joint_states()
            self.set_joint_states()
            self.r.sleep()

        self.clean_shutdown()


if __name__ == "__main__":
    try:
        rospy.init_node("follower_node", anonymous=True)
        robot = FollowerController()
        robot.run()
    except rospy.ROSInterruptException:
        pass
