#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class KeyPressSubscriber:

    def __init__(self):
       
        rospy.init_node('keypress_subscriber', anonymous=True)    # Initialize the node
        self.subscriber = rospy.Subscriber('/keypress', String, self.callback)   #topic----->keypress

    def callback(self, msg):
        
        key_press = msg.data
        
        #if msg.data=='w':
        #     ACTION
        # if msg.data=='S'
        #     ACTION
        # if msg.data=='D':
        #     ACTION
        # if msg.data=='A'
        #     ACTION
        #else:
        #     Action

    def run(self):
        
        rospy.spin() #a method to keed the node running

if __name__ == '__main__':
    try:
        key_press_subscriber = KeyPressSubscriber()
        key_press_subscriber.run()
    except rospy.ROSInterruptException:
        pass
