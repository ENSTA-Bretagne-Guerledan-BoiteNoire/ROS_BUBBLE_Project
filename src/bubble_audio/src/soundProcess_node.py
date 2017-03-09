#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import Float64, Int8
from audio.AlgoMusicAuto import process
import numpy as np

# 1 = Left
# 2 = Right

class soundProcess():
    def __init__(self):
        rospy.init_node('display_python')

        # Publisher
        self.angle_pub = rospy.Publisher('angle_ping', Float64, queue_size=1)
        self.cmdState_pub = rospy.Publisher('cmd_state', Int8, queue_size=1)

        # Subscriber
        self.cmdState_sub = rospy.Subscriber('cmd_state', Int8, self.updateState)

        # Data
        self.angle = -180

        self.stateMap = {'manual' : 0,
                         'linefollowing' : 1,
                         'stationkeeping' : 2}
        self.state = self.stateMap['manual']

    def updateState(self, msg):
        self.state = msg.data

    def spin(self):

        rate = rospy.Rate(0.1) # 10

        while not rospy.is_shutdown():

            print 'State : ',self.state
            #if self.state!=self.stateMap['manual']:
            #print 'Publishing state : ',self.stateMap['stationkeeping']
            #self.cmdState_pub.publish(self.stateMap['stationkeeping'])
            print 'Processing'
            self.angle = (process())/180*np.pi # Blocking operation
            print 'Publishing angle : ',self.angle/np.pi*180
            self.angle_pub.publish(self.angle)

            #if self.state!=self.stateMap['manual']:
            #print 'Publishing state : ',self.stateMap['linefollowing']
            #self.cmdState_pub.publish(self.stateMap['linefollowing'])

            rate.sleep()

if __name__ == '__main__':
    sp = soundProcess()
    sp.spin()
