#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from pynput import keyboard
msg = """
<<<<Reading from the keyboard>>>>
---------------------------
Moving Keys:
   Q    W    E
   A    k    D
   Z    S    X

"""
ctrl=False
class KeyboardReadings:
    global ctrl
    def __init__(self):
       
        rospy.init_node("keypress_publisher", anonymous=True)   ##Initialise the node

        self.keyboard_publisher = rospy.Publisher('/keypress', String, queue_size=1) #topic----->keypress
       
    def publish_keypress(self, keyPress):
        
        self.keyboard_publisher.publish(keyPress)  ##published the pressed key to the Topic

    def on_press(self, key):
        try:
            
            key_str = str(key.char) if key.char else str(key) 
            self.publish_keypress(key_str)
            
        except AttributeError:  
             print('')

    def on_release(self, key):

        if key==keyboard.Key.ctrl_l or keyboard.Key.ctrl_r:
            ctrl=True
        if key==keyboard.KeyCode.from_char('c')and ctrl:
          return False # Stop listener

    def keyboard_listener(self):
        
        with keyboard.Listener(
                on_press=self.on_press,
                on_release=self.on_release
        ) as listener:
         
            listener.join()
         
          

if __name__ == '__main__':
    try:
        print(msg)
        keyboard_publisher = KeyboardReadings()
        keyboard_publisher.keyboard_listener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    