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
        self.speed = 0.6
        self.count = 0
        self.LEFT_TICKS = None
        self.RIGHT_TICKS = None
        self.initial_left = None
        self.initial_right = None
    
    def left_callback(self, msg):
        self.wheel_integration.update_left(msg.data, rospy.get_rostime())
        # if not self.initial_left:
        #     self.initial_left = msg.data	
        # self.LEFT_TICKS = msg.data - self.initial_left
    def right_callback(self, msg):
        self.wheel_integration.update_right(msg.data, rospy.get_rostime())
        # if not self.initial_right:
        #     self.initial_right = msg.data   	
        # self.RIGHT_TICKS = msg.data - self.initial_right

    def run(self):
        DISTANCE = 1.25
        while not rospy.is_shutdown():
            self.driveForTime(0, 0, 4)
            self.driveForDistance(DISTANCE)
            self.driveBackwardForDistance(DISTANCE)
            self.driveForTime(0, 0, 4)
            self.adjustRotation(math.pi / 2)
            self.driveForTime(0, 0, 2)
            self.adjustRotation(math.pi / 2)
            self.driveForTime(0, 0, 2)
            self.adjustRotation(math.pi / 2)
            self.driveForTime(0, 0, 2)
            self.adjustRotation(math.pi / 2)
            self.driveForTime(0, 0, 4)
            self.driveForTime(0.25, 0.75, 6)
            self.driveForTime(0, 0, 2)
            break
        rospy.signal_shutdown("finished")

    def stop_momentum(self):
        """
        stay still to reduce the momentum of the car to zero after carrying out some movement
        """
        self.driveForTime(0, 0, 0.8)
    
    def drive(self, left_speed, right_speed):
        msg = WheelsCmdStamped()
        msg.vel_left = left_speed
        msg.vel_right = right_speed
        self.pub.publish(msg)
        x, y, theta = self.wheel_integration.get_state_meters()
        print(f'x{:.3f} y{y:.3f} theta{theta:.3f}, decision: vleft{left_speed:.3f}, vright{right_speed:.3f}')

    
    def driveForTime(self, left_speed, right_speed, tsec):
        for i in range(int(math.ceil(tsec * 60))):
            self.drive(left_speed, right_speed)
            self.rate.sleep()
    
    def driveToPoint(self, x, y):
        print(f'f-drive to point {x:.3f} {y:.3f}')
        while True:
            curx, cury, curtheta = self.wheel_integration.get_state_meters()
            dx, dy = x - curx, y - cury
            target_dist = math.sqrt(dx ** 2 + dy ** 2)
            if target_dist < 0.05:  # if within 50 millimeter then we think we are already on the target
                self.drive(0, 0)
                break

            to_target = math.atan2(dy, dx)
            to_adjust = math.fmod((to_target - curtheta) + math.pi, math.pi * 2) - math.pi
            OFF_THRESHOLD = 0.9
            if abs(to_adjust) > OFF_THRESHOLD:
                print('angle is off, adjust rotation')
                self.stop_momentum()
                self.adjustRotation(to_adjust)
                self.stop_momentum()
            else:
                speed = self.speed
                q = to_adjust / OFF_THRESHOLD * 0.5
                self.drive(speed * (1 - q), speed * (1 + q))
                self.rate.sleep()
        print('forward complete')
    
    def driveBackwardToPoint(self, x, y):
        print(f'b-drive to point {x:.3f} {y:.3f}')
        while True:
            curx, cury, curtheta = self.wheel_integration.get_state_meters()
            dx, dy = x - curx, y - cury
            target_dist = math.sqrt(dx ** 2 + dy ** 2)
            if target_dist < 0.05:  # if within 50 millimeter then we think we are already on the target
                self.drive(0, 0)
                break

            to_target = math.atan2(dy, dx)
            to_adjust = math.fmod(to_target - curtheta, math.pi * 2) - math.pi
            OFF_THRESHOLD = 0.9
            if abs(to_adjust) > OFF_THRESHOLD:
                print('angle is off, adjust rotation')
                self.stop_momentum()
                self.adjustRotation(to_adjust)
                self.stop_momentum()
            else:
                speed = self.speed
                q = to_adjust / OFF_THRESHOLD * 0.5
                self.drive(-speed * (1 + q), -speed * (1 - q))
                self.rate.sleep()
        print('backward complete')
    
    def adjustRotation(self, to_adjust):
        print(f'adjust rotation {to_adjust}')
        curx, cury, curtheta = self.wheel_integration.get_state_meters()
        target_theta = curtheta + to_adjust
        if to_adjust > 0:
            while curtheta < target_theta - 0.13:
                self.drive(-self.speed, self.speed)
                curx, cury, curtheta = self.wheel_integration.get_state_meters()
                self.rate.sleep()
        else:
            while curtheta > target_theta + 0.13:
                self.drive(self.speed, -self.speed)
                curx, cury, curtheta = self.wheel_integration.get_state_meters()
                self.rate.sleep()
        print('adjust complete')
        self.drive(0, 0)
    
    def adjustRotationWhileDriving(self, to_adjust):
        curx, cury, curtheta = self.wheel_integration.get_state_meters()
        target_theta = curtheta + to_adjust
        if to_adjust > 0:
            while curtheta < target_theta:
                self.drive(self.speed * 0.5, self.speed * 1.5)
                curx, cury, curtheta = self.wheel_integration.get_state_meters()
                self.rate.sleep()
        else:
            while curtheta > target_theta:
                self.drive(self.speed * 1.5, self.speed * 0.5)
                curx, cury, curtheta = self.wheel_integration.get_state_meters()
                self.rate.sleep()
        self.drive(0, 0)
    
    def adjustToTargetRotation(self, adjust_target_radian):
        curx, cury, curtheta = self.wheel_integration.get_state_meters()
        to_adjust = math.fmod((adjust_target_radian - curtheta) + math.pi, math.pi * 2) - math.pi
        if abs(to_adjust) > 0.05:
            self.adjustRotation(to_adjust)

    def driveForDistance(self, distance):
        curx, cury, curtheta = self.wheel_integration.get_state_meters()

        targetx, targety = curx + distance * math.cos(curtheta), cury + distance * math.sin(curtheta)
        self.driveToPoint(targetx, targety)
        self.stop_momentum()
        self.adjustToTargetRotation(curtheta)  # make sure the rotation is the same as initial rotation
    
    def driveBackwardForDistance(self, distance):
        curx, cury, curtheta = self.wheel_integration.get_state_meters()

        targetx, targety = curx - distance * math.cos(curtheta), cury - distance * math.sin(curtheta)
        self.driveBackwardToPoint(targetx, targety)
        self.stop_momentum()
        self.adjustToTargetRotation(curtheta)  # make sure the rotation is the same as initial rotation
        



if __name__ == '__main__':
    try:
        print(f'running on robot {HOST_NAME}')
        node = PilotNode('my_pilot_node', wheel_int.WheelPositionIntegration(50))
        node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


