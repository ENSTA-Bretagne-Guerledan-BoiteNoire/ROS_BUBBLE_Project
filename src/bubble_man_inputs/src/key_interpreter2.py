#!/usr/bin/env python
# coding=utf-8
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist

class key_interpreter():
    def __init__(self):
        self.key_mapping = { 'z': [ 0, 1],  's': [0, -1],
                             'q': [-1, 0],  'd': [1, 0],
                             ' ': ['STOP', 'STOP'],
                             'r': ['BACK', 'BACK'],

                             '8': [ 0, -1],  '2': [0, 1],
                             '4': [-1, 0],  '6': [1, 0],
                             '5': ['STOP', 'STOP'],
                             '0': ['BACK', 'BACK'],
                             'NoInputReceived': ['SLOW', 'SLOW'],


                             'm': ['s','manual'], 'l': ['s','linefollowing'], 'k': ['s','stationkeeping']}

        self.linCmd = 0
        self.angCmd = 0

        self.stateMap = {'manual' : 0,
                         'linefollowing' : 1,
                         'stationkeeping' : 2}
        self.state = self.stateMap['manual']

        self.slowRate = 5 # en pourdixmille diminué à chaque pas

    def keys_cb(self,msg, twist_pub):

        if len(msg.data) == 0 or not self.key_mapping.has_key(msg.data):
	    print 'len(msg.data) == 0 : ',len(msg.data) == 0
	    print 'self.key_mapping.has_key(msg.data) : ',self.key_mapping.has_key(msg.data)
	    print 'unknown key : ',msg.data
            return # unknown key

        vals = self.key_mapping[msg.data]
	if msg.data!='NoInputReceived':
	    print "received mgs.data : ",msg.data
	    print 'vals : ',vals

        # Gestion des entr�es
        if isinstance(vals[0],int):

            self.angCmd = vals[0]
            self.linCmd += vals[1]/20.0
	    print 'self.angCmd : ',self.angCmd
	    print 'self.linCmd : ',self.linCmd

	    print 'self.linCmd : ',self.linCmd
	    print 'self.angCmd : ',self.angCmd
	    
        elif vals[0]=='STOP':
            self.angCmd = 0
            self.linCmd = 0
        elif vals[0]=='SLOW':
            self.angCmd *= 1-self.slowRate/100.0
            #self.linCmd *= 1-self.slowRate/100.0

	    print 'Slowing self.angCmd : ',1-self.slowRate/100.0,'/',self.angCmd
	    #print 'Slowing self.linCmd : ',1-self.slowRate/100.0,'/',self.linCmd

        elif vals[0]=='BACK':
	    if self.angCmd != 0:
            	self.angCmd = max(min( -self.angCmd/abs(self.angCmd) , 1), -1)
            if self.linCmd != 0:
	        self.linCmd = max(min( -self.linCmd/abs(self.linCmd) , 1), -1)


        elif vals[0]=='s':
            self.state = self.stateMap[ vals[1] ]
            print 'cmd state : ',vals[1]

	self.angCmd = max(min(self.angCmd, 1), -1)
        self.linCmd = max(min(self.linCmd, 1), -1)

        # Creation du message twist
        print 'cmd motors :  angular[',self.angCmd,'] / linear[',self.linCmd,']'
        t = Twist()

        t.angular.x = 0
        t.angular.y = 0
        t.angular.z = self.angCmd

        t.linear.x = self.linCmd
        t.linear.y = 0
        t.linear.z = 0

        # Publish
        twist_pub.publish(t)
        state_pub.publish(self.state)

if __name__ == '__main__':

    ki = key_interpreter()
    rospy.init_node('keys_to_twist')

    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    state_pub = rospy.Publisher('cmd_state', Int8, queue_size=1)

    rospy.Subscriber('keys', String, ki.keys_cb, twist_pub)

    rospy.spin()
