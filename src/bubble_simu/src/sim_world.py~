#!/usr/bin/env python
# coding=utf-8

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np
import matplotlib.pyplot as plt
import tf.transformations as tf
from models.Boat import Boat

# 1 = Left
# 2 = Right

class world():
    def __init__(self):
        rospy.init_node('display_python')

        # Subscriber
        self.TL_sub = rospy.Subscriber('commandT1', Float32, self.updateTL)
        self.TR_sub = rospy.Subscriber('commandT2', Float32, self.updateTR)

        # Publisher
        self.pose_pub = rospy.Publisher('pose_real', Pose, queue_size=1)
        self.twist_pub = rospy.Publisher('twist_real', Twist, queue_size=1)

        # Internal variable
        self.dt = 0.1
        self.boat = Boat(0,0,0,0,0,0)

    def updateTL(self, msg):
        print 'received TL : ',msg
        self.boat.TL = msg.data

    def updateTR(self, msg):
        print 'received TR : ',msg
        self.boat.TR = msg.data

    def updateWorld(self):
        coeffFrot = 10
        dx,dy,dtheta,dv = self.boat.move(coeffFrot)

        # print 'dx : ',dx,dy,dtheta,dv

        self.boat.update(
            self.boat.x     + (dx    )*self.dt,
            self.boat.y     + (dy    )*self.dt,
            self.boat.theta + (dtheta)*self.dt,
            self.boat.v     + (dv    )*self.dt,
            dtheta
        )


    def spin(self):

        rate = rospy.Rate(1/self.dt)

        while not rospy.is_shutdown():

            self.updateWorld()

            # print 'Pose : ', self.boat.pose
            # print 'self.boat.pose.orientation.x : ',self.boat.pose.orientation.x
            # print 'self.boat.pose.orientation.y : ',self.boat.pose.orientation.y
            # print 'self.boat.pose.orientation.z : ',self.boat.pose.orientation.z
            # print 'self.boat.pose.orientation.w : ',self.boat.pose.orientation.w
            # print 'self.boat.pose.position.x : ',self.boat.pose.position.x
            # print 'self.boat.pose.position.y : ',self.boat.pose.position.y
            # print 'self.boat.pose.position.z : ',self.boat.pose.position.z

            self.pose_pub.publish(self.boat.pose)
            self.twist_pub.publish(self.boat.twist)

            rate.sleep()


if __name__ == '__main__':
    w = world()
    w.spin()
