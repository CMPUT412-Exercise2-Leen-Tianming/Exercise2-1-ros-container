#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped
import math


HOST_NAME = os.environ["VEHICLE_NAME"]


HOST_NAME = os.environ["VEHICLE_NAME"]


class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.pub = rospy.Publisher(f'/{HOST_NAME}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.rate = rospy.Rate(60)  # in Hz

    def run(self):
        i = 0
        j = 0
        k = 0
        l = 0
        speeds = [0.2, 0.33, 0.5, 1, 1.5]
        times = [0.5, 1]
        while not rospy.is_shutdown():
            speed = speeds[i]
            time = times[k]
            print(f'i:{i},j:{j},k:{k},speed:{speed},time:{time}')
            left_speed = speed
            right_speed = speed
            if j == 1:
                left_speed = 0
            elif j == 2:
                right_speed = 0
            elif j == 3:
                left_speed = -speed
            elif j == 4:
                right_speed = speed
            elif j == 5:
                left_speed = 0.5 * speed
            elif j == 6:
                right_speed = 0.5 * speed

            print('forward')
            self.driveForTime(left_speed, right_speed, time)
            print('stop')
            self.driveForTime(0, 0, 1)
            print('backward')
            self.driveForTime(-left_speed, -right_speed, time)
            print('stop')
            self.driveForTime(0, 0, 1)

            i += 1
            if i == len(speeds):
                i = 0
                j += 1
                if j == 7:
                    j = 0
                    k += 1
                    if k == len(times):
                        k = 0
                        l += 1
                        if l == 2:
                            break

    
    def drive(self, left_speed, right_speed):
        msg = WheelsCmdStamped()
        msg.vel_left = left_speed
        msg.vel_right = right_speed
        self.pub.publish(msg)
    
    def driveForTime(self, left_speed, right_speed, tsec):
        self.drive(left_speed, right_speed)
        for i in range(int(math.ceil(tsec * 60))):
            self.rate.sleep()

if __name__ == '__main__':
    print(f'running on robot {HOST_NAME}')
    node = MyPublisherNode('my_publisher_node')
    node.run()
    rospy.spin()






