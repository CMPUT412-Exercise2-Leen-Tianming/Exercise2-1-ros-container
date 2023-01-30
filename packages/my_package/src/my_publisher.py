#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped
from duckietown_msgs.msg import WheelEncoderStamped
import math
import wheel_int


HOST_NAME = os.environ["VEHICLE_NAME"]


class PilotNode(DTROS):
    def __init__(self, node_name, wheel_integration: wheel_int.WheelPositionIntegration):
        super(PilotNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.sub_left = rospy.Subscriber(f'{HOST_NAME}/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_callback)
        self.sub_right = rospy.Subscriber(f'{HOST_NAME}/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_callback)
        self.pub = rospy.Publisher(f'/{HOST_NAME}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.rate = rospy.Rate(60)  # in Hz
        self.wheel_integration = wheel_integration
        self.speed = 0.8
    
    def left_callback(self, msg):
        self.wheel_integration.update_left(msg.data, msg.header.stamp)
    
    def right_callback(self, msg):
        self.wheel_integration.update_right(msg.data, msg.header.stamp)

    def run(self):
        DISTANCE = 500
        while not rospy.is_shutdown():

            self.driveForTime(0, 0, 3)
            self.driveForDistance(DISTANCE)
            self.driveBackwardForDistance(DISTANCE)
            self.driveForTime(0, 0, 3)
            self.adjustRotation(math.pi / 2)
            self.driveForTime(0, 0, 2)
            self.adjustRotation(math.pi / 2)
            self.driveForTime(0, 0, 2)
            self.adjustRotation(math.pi / 2)
            self.driveForTime(0, 0, 2)
            self.adjustRotation(math.pi / 2)
            self.driveForTime(0, 0, 3)
            self.driveForTime(0.5, 1.5, 8)
            self.driveForTime(0, 0, 2)
            break

    
    def drive(self, left_speed, right_speed):
        msg = WheelsCmdStamped()
        msg.vel_left = left_speed
        msg.vel_right = right_speed
        self.pub.publish(msg)
        x, y, theta = self.wheel_integration.get_state()
        print(f'cur x{x} y{y} theta{theta}')

    
    def driveForTime(self, left_speed, right_speed, tsec):
        for i in range(int(math.ceil(tsec * 60))):
            self.drive(left_speed, right_speed)
            self.rate.sleep()
    
    def driveToPoint(self, x, y):
        print(f'f-drive to point {x} {y}')
        while True:
            curx, cury, curtheta = self.wheel_integration.get_state()
            dx, dy = x - curx, y - cury
            print(f'f dx{dx} dy{dy}')
            target_dist = math.sqrt(dx ** 2 + dy ** 2)
            if target_dist < 50:  # if within 50 millimeter then we think we are already on the target
                self.drive(0, 0)
                break

            to_target = math.atan2(dy, dx)
            to_adjust = math.fmod((to_target - curtheta) + math.pi, math.pi * 2) - math.pi
            OFF_THRESHOLD = 0.6
            if abs(to_adjust) > OFF_THRESHOLD:
                self.adjustRotation(to_adjust)
            else:
                speed = self.speed
                q = to_adjust / OFF_THRESHOLD * 0.5
                self.drive(speed * (1 - q), speed * (1 + q))
                self.rate.sleep()
    
    def driveBackwardToPoint(self, x, y):
        print(f'b-drive to point {x} {y}')
        while True:
            curx, cury, curtheta = self.wheel_integration.get_state()
            dx, dy = x - curx, y - cury
            print(f'b dx{dx} dy{dy}')
            target_dist = math.sqrt(dx ** 2 + dy ** 2)
            if target_dist < 50:  # if within 50 millimeter then we think we are already on the target
                self.drive(0, 0)
                break

            to_target = math.atan2(dy, dx)
            to_adjust = math.fmod(to_target - curtheta, math.pi * 2) - math.pi
            OFF_THRESHOLD = 0.6
            if abs(to_adjust) > OFF_THRESHOLD:
                self.adjustRotation(to_adjust)
            else:
                speed = self.speed
                q = to_adjust / OFF_THRESHOLD * 0.5
                self.drive(-speed * (1 + q), -speed * (1 - q))
                self.rate.sleep()
        print('backward complete')
    
    def adjustRotation(self, to_adjust):
        print(f'adjust rotation {to_adjust}')
        curx, cury, curtheta = self.wheel_integration.get_state()
        target_theta = curtheta + to_adjust
        if to_adjust > 0:
            while curtheta < target_theta - 0.13:
                self.drive(-self.speed, self.speed)
                curx, cury, curtheta = self.wheel_integration.get_state()
                self.rate.sleep()
        else:
            while curtheta > target_theta + 0.13:
                self.drive(self.speed, -self.speed)
                curx, cury, curtheta = self.wheel_integration.get_state()
                self.rate.sleep()
        print('adjust complete')
        self.drive(0, 0)
    
    def adjustRotationWhileDriving(self, to_adjust):
        curx, cury, curtheta = self.wheel_integration.get_state()
        target_theta = curtheta + to_adjust
        if to_adjust > 0:
            while curtheta < target_theta:
                self.drive(self.speed * 0.5, self.speed * 1.5)
                curx, cury, curtheta = self.wheel_integration.get_state()
                self.rate.sleep()
        else:
            while curtheta > target_theta:
                self.drive(self.speed * 1.5, self.speed * 0.5)
                curx, cury, curtheta = self.wheel_integration.get_state()
                self.rate.sleep()
        self.drive(0, 0)
    
    def adjustToTargetRotation(self, adjust_target_radian):
        curx, cury, curtheta = self.wheel_integration.get_state()
        to_adjust = math.fmod((adjust_target_radian - curtheta) + math.pi, math.pi * 2) - math.pi
        if abs(to_adjust) > 0.05:
            self.adjustRotation(to_adjust)

    def driveForDistance(self, distance):
        curx, cury, curtheta = self.wheel_integration.get_state()

        targetx, targety = curx + distance * math.cos(curtheta), cury + distance * math.sin(curtheta)
        self.driveToPoint(targetx, targety)
        self.adjustToTargetRotation(curtheta)  # make sure the rotation is the same as initial rotation
    
    def driveBackwardForDistance(self, distance):
        curx, cury, curtheta = self.wheel_integration.get_state()

        targetx, targety = curx - distance * math.cos(curtheta), cury - distance * math.sin(curtheta)
        self.driveBackwardToPoint(targetx, targety)
        self.adjustToTargetRotation(curtheta)  # make sure the rotation is the same as initial rotation
        



if __name__ == '__main__':
    print(f'running on robot {HOST_NAME}')
    node = PilotNode('my_pilot_node', wheel_int.WheelPositionIntegration(29))
    node.run()
    rospy.spin()

