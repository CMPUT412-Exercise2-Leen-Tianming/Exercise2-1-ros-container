#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String

class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.pub = rospy.Publisher('Chatter', String, queue_size=10)

    def run(self):
        rate = rospy.Rate(1)  # in Hz
        while not rospy.is_shutdown():
            message = 'Hello world'
            rospy.loginfo(f'Publishing message: {message}')
            self.pub.publish(message)
            rate.sleep()

# if __name__ == '__main__':
#     print(f'running on robot {os.environ["VEHICLE_NAME"]}')
#     node = MyPublisherNode('my_publisher_node')
#     node.run()
#     rospy.spin()



