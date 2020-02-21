#!/usr/bin/env python
import math

import rospy
import tf
from geometry_msgs.msg import Transform, Quaternion
import numpy as np


def movePlayer(tf_broadcaster, player_name, transform_now, vel, angle, max_vel):
    max_angle = math.pi / 30

    if angle > max_angle:
        angle = max_angle
    elif angle < -max_angle:
        angle = -max_angle

    if vel > max_vel:
        vel = max_vel

    T1 = transform_now

    T2 = Transform()
    T2.rotation = tf.transformations.quaternion_from_euler(0, 0, angle)
    T2.translation.x = vel
    matrix_trans = tf.transformations.translation_matrix((T2.translation.x,
                                                          T2.translation.y,
                                                          T2.translation.z))

    matrix_rot = tf.transformations.quaternion_matrix((T2.rotation[0],
                                                       T2.rotation[1],
                                                       T2.rotation[2],
                                                       T2.rotation[3]))
    matrixT2 = np.matmul(matrix_trans, matrix_rot)

    matrix_trans = tf.transformations.translation_matrix((T1.translation.x,
                                                          T1.translation.y,
                                                          T1.translation.z))

    matrix_rot = tf.transformations.quaternion_matrix((T1.rotation.x,
                                                       T1.rotation.y,
                                                       T1.rotation.z,
                                                       T1.rotation.w))
    matrixT1 = np.matmul(matrix_trans, matrix_rot)

    matrix_new_transform = np.matmul(matrixT1, matrixT2)

    quat = tf.transformations.quaternion_from_matrix(matrix_new_transform)
    trans = tf.transformations.translation_from_matrix(matrix_new_transform)

    T1.rotation = Quaternion(quat[0], quat[1], quat[2], quat[3])
    T1.translation.x = trans[0]
    T1.translation.y = trans[1]
    T1.translation.z = trans[2]

    tf_broadcaster.sendTransform(trans, quat, rospy.Time.now(), player_name, "world")
