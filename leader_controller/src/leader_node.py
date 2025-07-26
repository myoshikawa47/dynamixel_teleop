#!/usr/bin/env python3
import time
import numpy as np
from dynamixel_sdk import *
from dynamixel_driver.dynamixel_core import *
from dynamixel_driver.dynamixel_utils import *

# ROS
import rospy
from sensor_msgs.msg import JointState


def normalization(data, indataRange, outdataRange):
    data = (data - indataRange[0]) / (indataRange[1] - indataRange[0])
    data = data * (outdataRange[1] - outdataRange[0]) + outdataRange[0]
    return data


class LeaderContrller(DynamixelCore):
    def __init__(self):
        self.ns = rospy.get_namespace()
        self.param = rospy.get_param(self.ns + "leader")
        self.baudrate = self.param["baudrate"]
        self.freq = self.param["control_freq"]
        self.r = rospy.Rate(self.param["control_freq"])
        self.target_position = None

        # Initialize PortHandler/PacketHandler Structs
        self.port_handler = PortHandler(self.param["device"])
        self.packet_handler = PacketHandler(self.param["version"])
        self.readh = GroupSyncRead(
            self.port_handler, self.packet_handler, ADDR_PRESENT_POSITION, 4
        )
        self.write_position = GroupSyncWrite(
            self.port_handler, self.packet_handler, ADDR_GOAL_POSITION, 4
        )

        # publisher and subscriber
        self.joint_pub = rospy.Publisher(
            self.ns + "leader/joint_states", JointState, queue_size=1
        )
        self.joint_msg = JointState()

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
                self.set_delay_time(id, 1)
                self.set_torque(id, True)
                self.readh.addParam(id)

        self.joint_msg.name = self.joint_name
        self.joint_id = np.array(self.joint_id)
        self.joint_minmax = np.array(self.joint_minmax).T
        self.home_position = np.array(self.home_position)
        self.current_position = np.array(self.home_position)
        self.move_to_goal_pose(self.home_position, 2)

    def serial_close(self):
        self.port_handler.closePort()
        rospy.loginfo("Close serial port")

    def clean_shutdown(self):
        for id in self.joint_id:
            self.set_torque(id, False)
        self.serial_close()

    def torque_off(self):
        for id in self.joint_id:
            self.set_torque(id, False)

    def get_joint_states(self):
        retp = self.readh.txRxPacket()
        if retp == 0:
            for i, id in enumerate(self.joint_id):
                _current_position = self.readh.getData(id, 132, 4)
                if id == 9:
                    _current_position = normalization(
                        _current_position, (2048, 2660), (2048, 3500)
                    )
                self.current_position[i] = np.int32(_current_position)

        return retp, self.current_position

    def publish_joint_states(self):
        # convert teleop angle to gripper angle
        self.joint_msg.header.stamp = rospy.Time.now()
        self.joint_msg.position = np.array(self.current_position, dtype=np.int32)
        self.joint_pub.publish(self.joint_msg)

    def move_to_goal_pose(self, goal_pose, exptime, freq=100):
        # initialize
        retp, _ = self.get_joint_states()
        if retp != 0:
            rospy.logerr("[initialize_robot_pose] Cannot receive current joint state.")
            exit()

        nloop = exptime * freq
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
            time.sleep(1.0 / freq)

        self.torque_off()

    def run(self):
        while not rospy.is_shutdown():
            self.get_joint_states()
            self.publish_joint_states()
            self.r.sleep()

        self.clean_shutdown()


if __name__ == "__main__":
    try:
        rospy.init_node("teleop_node", anonymous=True)
        teleop = LeaderContrller()
        teleop.run()
    except rospy.ROSInterruptException:
        pass
