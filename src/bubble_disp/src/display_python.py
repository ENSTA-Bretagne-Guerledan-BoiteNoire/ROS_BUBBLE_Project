#!/usr/bin/env python
# coding=utf-8

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
import numpy as np
import matplotlib.pyplot as plt
import tf.transformations as tf

# 1 = Left
# 2 = Right

class display():
    def __init__(self):
        rospy.init_node('display_python')

        self.pose_sub = rospy.Subscriber('pose_est', Pose, self.updatePose)

        # Data to display
        self.x = 0
        self.y = 0
        self.theta = 0

        # trace
        self.xt, self.yt, self.thetat = [], [], []

        self.closed = False

    def updatePose(self, msg):
        print "Received pose : ",msg
        self.x = msg.position.x
        self.y = msg.position.y

        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w)
        (roll,pitch,yaw) = tf.euler_from_quaternion(quaternion)
        self.theta = yaw

    def update_trace(self):
        MAX_SIZE = 500
        self.xt.append(self.x)
        self.yt.append(self.y)
        self.thetat.append(self.theta)
        if len(self.xt) > MAX_SIZE:
            del(self.xt[0])
        if len(self.yt) > MAX_SIZE:
            del(self.yt[0])
        if len(self.thetat) > MAX_SIZE:
            del(self.thetat[0])

    def draw_boat(self):
        # Original
        hull = np.array([[-1,  5,  7, 7, 5, -1, -1, -1],
                         [-2, -2, -1, 1, 2,  2, -2, -2],
                         [ 1,  1,  1, 1, 1,  1,  1,  1]])
        # Rotation matrix
        R = np.array([[np.cos(self.theta), -np.sin(self.theta), self.x],
                      [np.sin(self.theta),  np.cos(self.theta), self.y],
                      [                 0,                   0,      1]])
        # Rotate
        hull = np.dot(R, hull)
        return hull

    def handle_close(self,event):
        print 'Plot window closed !'
        self.closed = True

    def spin(self):

        # Figure for display
        fig = plt.figure("Display")
        fig.canvas.mpl_connect('close_event', self.handle_close)
        plt.show(block=False)

        rate = rospy.Rate(10)

        while not rospy.is_shutdown() and not self.closed:

            plt.clf()

            self.update_trace()
            print "====== Plotting trace"
            plt.plot(self.x, self.y, 'ro')
            plt.plot(self.xt, self.yt, 'g')

            print "====== Plotting boat"
            hull = self.draw_boat()
            plt.plot(hull[0], hull[1], 'k', linewidth=2)

            plt.axis([self.x - 150, self.x + 150, self.y - 150, self.y + 150])
            plt.axis('equal')

            plt.pause(rate.sleep_dur.to_sec())
            # rate.sleep()


if __name__ == '__main__':
    d = display()
    d.spin()