#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2


HOST_NAME = os.environ["VEHICLE_NAME"]


class MySubscriberNode(DTROS):
    def __init__(self, node_name):
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.sub = rospy.Subscriber(f'{HOST_NAME}/camera_node/image/compressed', CompressedImage, self.callback)
        self.pub = rospy.Publisher(f'{HOST_NAME}/compressed', CompressedImage, queue_size=10)
        self.image = None
        self.seq = 0
    
    def callback(self, msg):
        # how to decode compressed image
        # reference: http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
        compressed_image = np.frombuffer(msg.data, np.uint8)
        im = cv2.imdecode(compressed_image, cv2.IMREAD_COLOR)
        rospy.loginfo(f'subscriber: image size is {np.shape(im)}')
        # cv2.imwrite('/home/duckie/image.jpg', im)
        self.image = im
    
    def run(self):
        rate = rospy.Rate(10)  # in Hz
        while not rospy.is_shutdown():
            if self.image is not None:
                msg = CompressedImage()
                msg.header.seq = self.seq
                msg.header.stamp = rospy.Time.now()
                msg.format = 'jpeg'
                ret, buffer = cv2.imencode('.jpg', self.image)
                if not ret:
                    print('failed to encode image!')
                else:
                    msg.data = np.array(buffer).tostring()
                    print(f'publishing an image to custom topic with header {str(msg.header)}')
                    self.pub.publish(msg)
                    self.seq += 1
            rate.sleep()


if __name__ == '__main__':
    node = MySubscriberNode('my_subscriber_node')
    node.run()
    rospy.spin()