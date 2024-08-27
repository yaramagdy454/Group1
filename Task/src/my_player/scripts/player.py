#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn, Kill
from std_msgs.msg import String

class TurtlePlayer:
    def __init__(self):
        rospy.init_node('turtle_player')
        self.turtle_health = {
            'turtle1': 100  # Initialize health for the default turtle
        }
        
        # Wait for the spawn and kill services to be available
        rospy.loginfo("Waiting for /spawn service...")
        rospy.wait_for_service('/spawn')
        self.spawn_service = rospy.ServiceProxy('/spawn', Spawn)

        rospy.Subscriber('string_topic', String, self.update_health)

        rospy.loginfo("Waiting for /kill service...")
        rospy.wait_for_service('/kill')
        self.kill_service = rospy.ServiceProxy('/kill', Kill)

    def spawn_turtle(self, x, y, theta, name):
        try:
            response = self.spawn_service(x, y, theta, name)
            self.turtle_health[name] = 100  # Initial health
            rospy.loginfo(f"Turtle {name} spawned with ID {response.name} and health {self.turtle_health[name]}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def update_health(self, msg):
        if msg.data in self.turtle_health:
            self.turtle_health[msg.data] -= 20
            rospy.loginfo(f"Turtle {msg.data} health updated to {self.turtle_health[msg.data]}")
        else:
            rospy.logwarn(f"Turtle {msg.data} not found in health record")

    def remove_turtle(self, name):
        try:
            self.kill_service(name)
            if name in self.turtle_health:
                del self.turtle_health[name]
            rospy.loginfo(f"Turtle {name} removed.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def run(self):
        # Make sure turtles are spawned before running spin
        rospy.loginfo("Spawning turtles and waiting before running the main loop...")
        rospy.sleep(1)  # Short sleep to ensure services are ready and turtles are spawned
        rospy.spin()

if __name__ == '__main__':
    try:
        manager = TurtlePlayer()
        # Spawn turtles with specific names
        manager.spawn_turtle(5.0, 7.0, 0.0, 'turtle2')
        manager.spawn_turtle(7.0, 4.0, 0.0, 'turtle3')

        # Allow time for turtles to spawn
        rospy.sleep(2.0)

        # all of coming code is just a test you must delete it in final combination of other nodes

#------------------------------------------------------------------------------------------------------------------------------------------
        # Publish messages to update health of turtles
        string_pub = rospy.Publisher('string_topic', String, queue_size=10)
        rospy.sleep(1.0)  # Ensure publisher is ready

        # Example of publishing a message to update the health of 'turtle1'
        str_msg = String()
        str_msg.data = "turtle1"
        string_pub.publish(str_msg)
        rospy.loginfo("Published message to update health.")

        # Allow time for health update
        rospy.sleep(3.0)

        # Remove turtles
#        manager.remove_turtle("turtle1")  # This will remove the default turtle
#--------------------------------------------------------------------------------------------------------------------------
        manager.run()

    except rospy.ROSInterruptException:
        pass
