import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
import tf.transformations as tf

class Boat():

    def __init__(self,x=0,y=0,theta=0,v=0,delta=0,T1=0,T2=0):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.delta = delta

        self.T1 = T1
        self.T2 = T2

        self.pose = Pose()

        print 'pose 1 : ',self.pose

        self.pose.position.x = x
        self.pose.position.y = y
        self.pose.position.z = 0
        q = tf.quaternion_from_euler(0, 0, theta)
        self.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        print 'pose 2 : ',self.pose

    def move(boat):
        print 'prev T1 :',boat.T1
        print 'prev T2 :',boat.T2
        print 'prev v :',boat.v

        x = boat.v*np.cos(boat.theta)
        y = boat.v*np.sin(boat.theta)
        theta = boat.v*boat.T1
        v = -(boat.T1 + np.sin(boat.delta))*boat.T2 - boat.v
        delta = -boat.v*(boat.T1 + np.sin(boat.delta))

        return x,y,theta,v,delta

    def update(self,x,y,theta,v,delta):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.delta = delta

        self.pose.position.x = x
        self.pose.position.y = y
        q = tf.quaternion_from_euler(0, 0, theta)
        self.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])