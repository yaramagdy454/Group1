#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

class TurtleAttackGame:
    def __init__(self):
        rospy.init_node('attacking', anonymous=True)
        
        self.attack_radius = 0.02  # 2 cm
        self.attack_duration = 1.0  # 1 second
        self.max_attacks = 10
        
        self.attacks = {'turtle1': self.max_attacks, 'turtle2': self.max_attacks, 'turtle3': self.max_attacks}
        self.turtle_poses = {'turtle1': None, 'turtle2': None, 'turtle3': None}
        
        
        self.velocity_publishers = {
            'turtle1': rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10),
            'turtle2': rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10),
            'turtle3': rospy.Publisher('/turtle3/cmd_vel', Twist, queue_size=10)
        }
        
        # Subscribers to get turtles' positions
        rospy.Subscriber('/turtle1/pose', Pose, self.update_pose, callback_args='turtle1')
        rospy.Subscriber('/turtle2/pose', Pose, self.update_pose, callback_args='turtle2')
        rospy.Subscriber('/turtle3/pose', Pose, self.update_pose, callback_args='turtle3')
        
        self.rate = rospy.Rate(1)  # 1 Hz for the main loop

    def update_pose(self, data, turtle_name):
        self.turtle_poses[turtle_name] = data

    def distance(self, pose1, pose2):
        return math.sqrt((pose1.x - pose2.x)**2 + (pose1.y - pose2.y)**2)

    def attack(self, attacker, target):
        if self.attacks[attacker] > 0 and self.attacks[target] > 0:
            attacker_pose = self.turtle_poses[attacker]
            target_pose = self.turtle_poses[target]
            
            if attacker_pose and target_pose and self.distance(attacker_pose, target_pose) <= self.attack_radius:
                rospy.loginfo(f"{attacker} attacks {target}!")
                self.attacks[attacker] -= 1
                rospy.sleep(self.attack_duration)
                rospy.loginfo(f"{attacker} has {self.attacks[attacker]} attacks left.")
                rospy.loginfo(f"{target} has {self.attacks[target]} attacks left.")

    def run_game(self):
        while not rospy.is_shutdown():
            if all(attacks == 0 for attacks in self.attacks.values()):
                rospy.loginfo("Game over! All turtles have no attacks left.")
                break
            self.attack('turtle1', 'turtle2')
            self.attack('turtle2', 'turtle3')
            self.attack('turtle3', 'turtle1')

            self.rate.sleep()

if __name__ == '__main__':
    game = TurtleAttackGame()
    try:
        game.run_game()
    except rospy.ROSInterruptException:
        pass
