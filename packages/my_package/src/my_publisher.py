#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped
from duckietown_msgs.msg import WheelEncoderStamped
from duckietown_msgs.msg import Pose2DStamped
from duckietown_msgs.srv import ChangePattern
from std_msgs.msg import String
import math
import wheel_int
import time


HOST_NAME = os.environ["VEHICLE_NAME"]


def clip_0_2pi(rad):
    return rad % (2 * math.pi)


def clip_npi_pi(rad):
    return (rad + math.pi) % (2 * math.pi) - math.pi


class PilotNode(DTROS):
    def __init__(self, node_name, wheel_integration: wheel_int.WheelPositionIntegration):
        super(PilotNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.sub_left = rospy.Subscriber(f'{HOST_NAME}/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_callback)
        self.sub_right = rospy.Subscriber(f'{HOST_NAME}/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_callback)
        self.pub = rospy.Publisher(f'/{HOST_NAME}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.pose_pub = rospy.Publisher(f'/{HOST_NAME}/pose2d', Pose2DStamped, queue_size=10)
        self.rate = rospy.Rate(60)  # in Hz
        self.wheel_integration = wheel_integration
        self.speed = 0.6
        self.count = 0
        self.LEFT_TICKS = None
        self.RIGHT_TICKS = None
        self.initial_left = None
        self.initial_right = None
    
    def change_pattern(self, patternStr):
        rospy.wait_for_service(f'/{HOST_NAME}/led_emitter_node/set_pattern')
        try:
            changePatternSrv = rospy.ServiceProxy(f'/{HOST_NAME}/led_emitter_node/set_pattern', ChangePattern)
            msg = String()
            msg.data = patternStr
            changePatternSrv(msg)
        except rospy.ServiceException as e:
            print('Service request failed')
            print(e)

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
            t1 = time.time()
            self.change_pattern('RED')
            self.driveForTime(0, 0, 5)

            self.change_pattern('WHITE')
            self.adjustToTargetRotation(0)
            self.driveToPoint(1.25, 0)
            self.adjustToTargetRotation(math.pi / 2)
            self.driveToPoint(1.25, 1.25)
            self.adjustToTargetRotation(math.pi)
            self.driveToPoint(0, 1.25)

            self.change_pattern('RED')
            self.driveForTime(0, 0, 5)

            # state 3
            self.change_pattern('BLUE')
            self.driveToPoint(0, 0)
            self.adjustToTargetRotation(math.pi / 2)

            self.change_pattern('RED')
            self.driveForTime(0, 0, 5)

            self.change_pattern('GREEN')
            self.driveForTime(0.75, 0.25, 6)

            self.driveForTime(0, 0, 5)

            t2 = time.time()

            # print the vehicle position and angle
            x, y, theta = self.wheel_integration.get_state_meters()
            print(f'final position:\nx{x:.3f} y{y:.3f} theta{theta:.3f}')

            print('execution time: ' + str(t2 - t1))

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
        pose = Pose2DStamped()
        pose.header.t = rospy.get_rostime()
        pose.x = x
        pose.y = y
        pose.theta = clip_npi_pi(theta)
        self.pose_pub.publish(pose)
        # print(f'x{x:.3f} y{y:.3f} theta{theta:.3f}, decision: vleft{left_speed:.3f}, vright{right_speed:.3f}')

    
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
            to_adjust = clip_npi_pi(to_target - curtheta)
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
            to_adjust = (to_target - curtheta) % (math.pi * 2) - math.pi
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
            while curtheta < target_theta - 0.21:
                self.drive(-self.speed, self.speed)
                curx, cury, curtheta = self.wheel_integration.get_state_meters()
                self.rate.sleep()
        else:
            while curtheta > target_theta + 0.21:
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
        to_adjust = ((adjust_target_radian - curtheta) + math.pi) % (math.pi * 2) - math.pi
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
        node = PilotNode('my_pilot_node', wheel_int.WheelPositionIntegration(29, 0, 0, math.pi / 2))
        node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


