#!/usr/bin/env python
# Copyright 2016 Lucas Walter

import math
import rospy
import tf2_py as tf2
import tf2_ros

from geometry_msgs.msg import Point, PointStamped, TransformStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker


class Acker():
    def __init__(self):
        rospy.init_node("acker")
        self.rate = rospy.get_param("~rate", 20.0)
        self.period = 1.0 / self.rate

        self.tf_buffer = tf2_ros.Buffer()
        self.tf = tf2_ros.TransformListener(self.tf_buffer)

        # get a dict of joints and their link locations
        self.steered = rospy.get_param("~steered_joints",
                                       [{'link': 'front_left_steer',
                                         'joint': 'front_left_steer_joint'},
                                        {'link': 'front_right_steer',
                                         'joint': 'front_right_steer_joint'}])

        # the fixed back axle- all the fixed wheels rotate around the y-axis
        # of the back axle
        self.back_axle_link = rospy.get_param("~back_axle_link", "back_axle")
        # TODO(lucasw) for now assume all the links have no rotation
        # with respect to each other, later do this robustly with tf lookups

        # self.marker_pub = rospy.Publisher("marker", Marker, queue_size=3)
        self.point_pub = rospy.Publisher("spin_center", PointStamped, queue_size=1)
        self.joint_pub = rospy.Publisher("steered_joint_states", JointState, queue_size=1)
        self.steer = rospy.get_param("~steer", {'link': 'front_steer',
                                                'joint': 'front_steer_joint'})
        self.joint_sub = rospy.Subscriber("joint_states", JointState,
                                          self.steer_callback, queue_size=4)

        self.timer = rospy.Timer(rospy.Duration(self.period), self.update)

    def update(self, event):
        pass

    def steer_callback(self, msg):
        if self.steer['joint'] not in msg.name:
            rospy.logwarn("no %s joint in joint state msg" % (self.steer['joint']))
            rospy.logwarn(msg)
            return

        joint_states = JointState()
        joint_states.header = msg.header

        steer_ind = msg.name.index(self.steer['joint'])
        steer_angle = msg.position[steer_ind]
        # steer_velocity = msg.velocity[steer_ind]
        # steer_effort = msg.effort[steer_ind]

        if steer_angle == 0.0:
            # TODO(lwalter) assume no rotation of link holding this joint
            # with respect to the back axle link
            for i in range(len(self.steered)):
                joint = self.steered[i]['joint']
                joint_states.name.append(joint)
                joint_states.position.append(steer_angle)
                # joint_states.velocity.append(steer_velocity)
                # joint_states.effort.append(steer_effort)
            self.joint_pub.publish(joint_states)
            return

        # find spin center given steer joint
        steer_ts = self.tf_buffer.lookup_transform(self.back_axle_link, self.steer['link'],
                                                   msg.header.stamp, rospy.Duration(4.0))

        spin_center = PointStamped()
        # R cos(steer_angle) = y
        # R sin(steer_angle) = x
        # x = steer_ts.position.x
        # R = steer_ts.position.x / sin(steer_angle)
        y = steer_ts.transform.translation.x / math.tan(-steer_angle)
        spin_center.point.y = y + steer_ts.transform.translation.y
        spin_center.header.stamp = msg.header.stamp
        spin_center.header.frame_id = self.back_axle_link
        self.point_pub.publish(spin_center)

        # TODO(lucasw) assume no rotation for now
        for i in range(len(self.steered)):
            joint = self.steered[i]['joint']
            link = self.steered[i]['link']

            # lookup the position of each link in the back axle frame
            ts = self.tf_buffer.lookup_transform(self.back_axle_link, link,
                                                 msg.header.stamp, rospy.Duration(4.0))

            dy = ts.transform.translation.y - spin_center.point.y
            dx = ts.transform.translation.x - spin_center.point.x
            angle = math.atan2(dx, abs(dy))
            if steer_angle < 0:
                angle = -angle

            # print link, angle, dx, dy
            joint_states.name.append(joint)
            joint_states.position.append(angle)

        self.joint_pub.publish(joint_states)

if __name__ == '__main__':
    acker = Acker()
    rospy.spin()
