#!/usr/bin/env python
import random

import math

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Transform, Quaternion
from rws2020_msgs.msg import MakeAPlay
from rws2020_lib.utils import movePlayer, randomizePlayerPose, getDistanceAndAngleToTarget


class Player:

    def __init__(self, player_name):

        self.player_name = player_name
        self.listener = tf.TransformListener()

        red_team = rospy.get_param('/red_team')
        green_team = rospy.get_param('/green_team')
        blue_team = rospy.get_param('/blue_team')

        if self.player_name in red_team:
            self.my_team, self.prey_team, self.hunter_team = 'red', 'green', 'blue'
            self.my_players, self.preys, self.hunters = red_team, green_team, blue_team

        elif self.player_name in green_team:
            self.my_team, self.prey_team, self.hunter_team = 'green', 'blue', 'red'
            self.my_players, self.preys, self.hunters = green_team, blue_team, red_team

        elif self.player_name in blue_team:
            self.my_team, self.prey_team, self.hunter_team = 'blue', 'red', 'green'
            self.my_players, self.preys, self.hunters = blue_team, red_team, green_team

        else:
            rospy.logerr('My name is not in any team. I want to play!')
            exit(0)

        rospy.logwarn(self.player_name + ' starting to play ... be very afraid!!!')

        self.br = tf.TransformBroadcaster()
        self.transform = Transform()
        randomizePlayerPose(self.transform)

        rospy.Subscriber("make_a_play", MakeAPlay, self.makeAPlayCallBack)  # Subscribe make a play msg

    def makeAPlayCallBack(self, msg):

        max_vel, max_angle = msg.turtle,  math.pi / 30

        if msg.green_alive:  # PURSUIT MODE: Follow any green player (only if there is at least one green alive)
            target = msg.green_alive[0]  # select the first alive green player (I am hunting green)
            distance, angle = getDistanceAndAngleToTarget(self.listener,
                                                          self.player_name, target)

            if angle is None:
                angle = 0
            vel = max_vel  # full throttle
            rospy.loginfo(self.player_name + ': Hunting ' + str(target) + '(' + str(distance) + ' away)')
        else:  # what else to do? Lets just move towards the center
            target = 'world'
            distance, angle = getDistanceAndAngleToTarget(self.listener, self.player_name, target)
            vel = max_vel  # full throttle
            rospy.loginfo(self.player_name + ': Moving to the center of the arena.')
            rospy.loginfo('I am ' + str(distance) + ' from ' + target)

        # Actually move the player
        movePlayer(self.br, self.player_name, self.transform, vel, angle, max_vel)


def main():
    rospy.init_node('moliveira', anonymous=False)
    player = Player('moliveira')
    rospy.spin()


if __name__ == "__main__":
    main()
