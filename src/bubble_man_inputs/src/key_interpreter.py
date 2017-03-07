#!/usr/bin/env python
# coding=utf-8
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist

class key_interpreter():
    def __init__(self):
        self.key_mapping = { 'z': [ 0 , 1  ], 'x': [ 0 , -1],
                             'q': [-1 , 0  ], 'd': [ 1 , 0 ],
                             's': [ 0 , 0  ],
                             'a': ['v', 0.1], 'e': ['v',-0.1],

                             '2': [ 0 , 1  ], '8': [ 0 , -1],
                             '4': [-1 , 0  ], '6': [ 1 , 0 ],
                             '5': [ 0 , 0  ],
                             '1': ['v', 0.1], '3': ['v',-0.1],

                             'm': ['s','manual'], 'l': ['s','linefollowing'], 'k': ['s','stationkeeping']}

        self.speed = 0.1
        self.linCmd = 0
        self.angCmd = 0

        self.stateMap = {'manual' : 0,
                         'linefollowing' : 1,
                         'stationkeeping' : 2}
        self.state = self.stateMap['manual']

    def keys_cb(self,msg, twist_pub):

        if len(msg.data) == 0 or not self.key_mapping.has_key(msg.data[0]):
            return # unknown key

        vals = self.key_mapping[msg.data[0]]

        # Gestion des entrées
        if isinstance(vals[0],int):
            self.angCmd = vals[0]
            self.linCmd = vals[1]
        elif vals[0]=='v':
            self.speed += vals[1]
            if self.speed < 0:
                self.speed = 0
            elif self.speed > 1:
                self.speed = 1
        elif vals[0]=='s':
            self.state = self.stateMap[ vals[1] ]
            print 'cmd state : ',vals[1], ' : ', self.state


        # Creation du message twist
        print 'cmd motors :  angular[',self.angCmd,'] / linear[',self.linCmd,'] / speed : ',self.speed
        t = Twist()

        t.angular.x = 0
        t.angular.y = 0
        t.angular.z = self.angCmd * self.speed

        t.linear.x = self.linCmd * self.speed
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