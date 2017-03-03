#!/usr/bin/env python
# coding=utf-8

import rospy
from geometry_msgs.msg import Pose
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
        self.T1_sub = rospy.Subscriber('commandT1', Float32, self.updateT1)
        self.T2_sub = rospy.Subscriber('commandT2', Float32, self.updateT2)

        # Publisher
        self.pose_pub = rospy.Publisher('pose_real', Pose, queue_size=1)

        # Internal variable
        self.dt = 0.1
        self.boat = Boat(0,0,0,0,0,0)

    def updateT1(self, msg):
        print 'received T1 : ',msg
        self.boat.T1 = msg.data

    def updateT2(self, msg):
        print 'received T2 : ',msg
        self.boat.T2 = msg.data

    def updateWorld(self):
        x,y,theta,v,delta = self.boat.move()
        print 'dx',x,y,theta,v,delta
        self.boat.update(
            self.boat.x     + x*self.dt,
            self.boat.y     + y*self.dt,
            self.boat.theta + theta*self.dt,
            self.boat.v     + v*self.dt,
            self.boat.delta + delta*self.dt
        )


    def spin(self):

        rate = rospy.Rate(1/self.dt)

        while not rospy.is_shutdown():

            self.updateWorld()

            print 'Pose : ', self.boat.pose
            print 'self.boat.pose.orientation.x : ',self.boat.pose.orientation.x
            print 'self.boat.pose.orientation.y : ',self.boat.pose.orientation.y
            print 'self.boat.pose.orientation.z : ',self.boat.pose.orientation.z
            print 'self.boat.pose.orientation.w : ',self.boat.pose.orientation.w
            print 'self.boat.pose.position.x : ',self.boat.pose.position.x
            print 'self.boat.pose.position.y : ',self.boat.pose.position.y
            print 'self.boat.pose.position.z : ',self.boat.pose.position.z

            self.pose_pub.publish(self.boat.pose)

            rate.sleep()


if __name__ == '__main__':
    w = world()
    w.spin()