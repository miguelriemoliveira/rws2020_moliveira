#!/usr/bin/env python
import random

import math

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Transform, Quaternion
from rws2020_msgs.msg import MakeAPlay
from rws2020_lib.utils import movePlayer


class Player:

    def __init__(self, player_name):

        self.player_name = player_name

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

        rospy.logwarn(
            'I am ' + self.player_name + ' and I am from this team ' + self.my_team + '. ' + self.prey_team + ' players are all going die!')
        rospy.loginfo('I am afraid of ' + str(self.hunters))

        # Subscribe make a play msg
        rospy.Subscriber("make_a_play", MakeAPlay, self.makeAPlayCallBack)
        self.br = tf.TransformBroadcaster()
        self.transform = Transform()
        # Initial_R = 8 * random.random()
        # Initial_Theta = 2 * math.pi * random.random()
        # Initial_X = Initial_R * math.cos(Initial_Theta)
        # Initial_Y = Initial_R * math.sin(Initial_Theta)
        # Initial_Rotation = 2 * math.pi * random.random()
        # self.transform.translation.x = Initial_X
        # self.transform.translation.y = Initial_Y
        # # q = tf.transformations.quaternion_from_euler(0, 0, Initial_Rotation)
        # self.transform.rotation = Quaternion( q[0], q[1], q[2], q[3])

        self.transform.translation.x = 5
        self.transform.translation.y = 5
        self.transform.translation.z = 0
        # self.transform.rotation.x = 0
        # self.transform.rotation.y = 0
        # self.transform.rotation.z = 0
        # self.transform.rotation.w = 1


    def makeAPlayCallBack(self, msg):

        max_vel = msg.turtle
        max_angle = math.pi / 30
        print('Received message make a play ... my max velocity is ' + str(max_vel))

        # Make a play
        vel = max_vel  # full throttle
        angle = max_angle
        angle = 0

        movePlayer(self.br, self.player_name, self.transform, vel, angle, max_vel)

    # def move(self, transform_now, vel, angle):
    #
    #     if angle > self.max_angle:
    #         angle = self.max_angle
    #     elif angle < -self.max_angle:
    #         angle = -self.max_angle
    #
    #     if vel > self.max_vel:
    #         vel = self.max_vel
    #
    #     T1 = transform_now
    #
    #     T2 = Transform()
    #     T2.rotation = tf.transformations.quaternion_from_euler(0, 0, angle)
    #     T2.translation.x = vel
    #     matrix_trans = tf.transformations.translation_matrix((T2.translation.x,
    #                                                           T2.translation.y,
    #                                                           T2.translation.z))
    #
    #     matrix_rot = tf.transformations.quaternion_matrix((T2.rotation[0],
    #                                                        T2.rotation[1],
    #                                                        T2.rotation[2],
    #                                                        T2.rotation[3]))
    #     matrixT2 = np.matmul(matrix_trans, matrix_rot)
    #
    #     matrix_trans = tf.transformations.translation_matrix((T1.translation.x,
    #                                                           T1.translation.y,
    #                                                           T1.translation.z))
    #
    #     matrix_rot = tf.transformations.quaternion_matrix((T1.rotation.x,
    #                                                        T1.rotation.y,
    #                                                        T1.rotation.z,
    #                                                        T1.rotation.w))
    #     matrixT1 = np.matmul(matrix_trans, matrix_rot)
    #
    #     matrix_new_transform = np.matmul(matrixT1, matrixT2)
    #
    #     quat = tf.transformations.quaternion_from_matrix(matrix_new_transform)
    #     trans = tf.transformations.translation_from_matrix(matrix_new_transform)
    #
    #     T1.rotation = Quaternion(quat[0], quat[1], quat[2], quat[3])
    #     T1.translation.x = trans[0]
    #     T1.translation.y = trans[1]
    #     T1.translation.z = trans[2]
    #
    #     self.br.sendTransform(trans, quat, rospy.Time.now(), self.player_name, "world")


def callback(msg):
    print("Recevied a message containing string " + msg.data)


def main():
    print("Hello player node!")

    rospy.init_node('moliveira', anonymous=False)
    player = Player('moliveira')

    # rospy.Subscriber("chatter", String, callback)
    rospy.spin()


if __name__ == "__main__":
    main()
