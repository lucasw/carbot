#!/usr/bin/env python
# Copyright 2016 Lucas Walter

import math
import numpy
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
        # TODO(lucasw) need to know wheel radius to compute velocity
        # correctly, for now assume all wheels are same radius
        self.steered = rospy.get_param("~steered_joints",
                                       [{'link': 'front_left_steer',
                                         'joint': 'front_left_steer_joint',
                                         'wheel_joint': 'wheel_front_left_axle'},
                                        {'link': 'front_right_steer',
                                         'joint': 'front_right_steer_joint',
                                         'wheel_joint': 'wheel_front_right_axle'}])
        self.unsteered = rospy.get_param("~unsteered_joints",
                                         [{'link': 'back_left',
                                           'wheel_joint': 'wheel_back_left_axle'},
                                          {'link': 'back_right',
                                           'wheel_joint': 'wheel_back_left_axle'}])

        # use this to store all the positions persistently
        self.wheel_joint_states = JointState()
        for wheel in self.steered:
            # TODO(lucasw) read the current angle to initialize
            self.wheel_joint_states.name.append(wheel['wheel_joint'])
            self.wheel_joint_states.position.append(0.0)
            self.wheel_joint_states.velocity.append(0.0)

        # the fixed back axle- all the fixed wheels rotate around the y-axis
        # of the back axle
        self.back_axle_link = rospy.get_param("~back_axle_link", "back_axle")
        # TODO(lucasw) for now assume all the links have no rotation
        # with respect to each other, later do this robustly with tf lookups

        self.marker = Marker()
        self.marker.id = 0
        self.marker.type = Marker.LINE_STRIP
        self.marker.frame_locked = True
        self.marker.action = Marker.ADD
        self.marker.header.frame_id = self.back_axle_link
        self.marker.scale.x = 0.07
        self.marker.scale.y = 0.07
        self.marker.scale.z = 0.07
        self.marker.color.r = 0.5
        self.marker.color.g = 0.5
        self.marker.color.b = 0.5
        self.marker.color.a = 0.5
        self.marker.pose.orientation.w = 1.0

        self.marker_pub = rospy.Publisher("marker", Marker, queue_size=len(self.steered) * 2)
        self.point_pub = rospy.Publisher("spin_center", PointStamped, queue_size=1)
        self.joint_pub = rospy.Publisher("steered_joint_states", JointState, queue_size=3)
        self.steer = rospy.get_param("~steer", {'link': 'lead_steer',
                                                'joint': 'lead_steer_joint',
                                                'wheel_joint': 'wheel_lead_axle'})
        self.joint_sub = rospy.Subscriber("joint_states", JointState,
                                          self.steer_callback, queue_size=4)

        self.timer = rospy.Timer(rospy.Duration(self.period), self.update)

    def update(self, event):
        pass

    def get_angle(self, link, spin_center, steer_angle, stamp):
        # lookup the position of each link in the back axle frame
        ts = self.tf_buffer.lookup_transform(self.back_axle_link, link,
                                             stamp, rospy.Duration(4.0))

        dy = ts.transform.translation.y - spin_center.point.y
        dx = ts.transform.translation.x - spin_center.point.x
        angle = math.atan2(dx, abs(dy))
        if steer_angle < 0:
            angle = -angle

        # visualize the trajectory forward or back of the current wheel
        # given the spin center
        radius = math.sqrt(dx * dx + dy * dy)
        self.marker.points = []
        for pt_angle in numpy.arange(abs(angle) - 1.0, abs(angle) + 1.0 + 0.025, 0.05):
            pt = Point()
            pt.x = spin_center.point.x + radius * math.sin(pt_angle)
            if steer_angle < 0:
                pt.y = spin_center.point.y - radius * math.cos(pt_angle)
            else:
                pt.y = spin_center.point.y + radius * math.cos(pt_angle)
            self.marker.ns = link
            self.marker.header.stamp = stamp
            self.marker.points.append(pt)
        self.marker_pub.publish(self.marker)
        return angle

    def steer_callback(self, msg):
        if self.steer['joint'] not in msg.name:
            rospy.logwarn("no %s joint in joint state msg" % (self.steer['joint']))
            rospy.logwarn(msg)
            return
        if self.steer['wheel_joint'] not in msg.name:
            rospy.logwarn("no %s joint in joint state msg" % (self.steer['joint']))
            rospy.logwarn(msg)
            return

        dt = (msg.header.stamp - self.wheel_joint_states.header.stamp).to_sec()
        if self.wheel_joint_states.header.stamp.to_sec() == 0.0:
            dt = 0

        steer_ind = msg.name.index(self.steer['joint'])
        steer_angle = msg.position[steer_ind]
        # TODO(lucasw) use the wheel_lead angle as a velocity right now
        angular_velocity = msg.position[msg.name.index(self.steer['wheel_joint'])]
        # steer_velocity = msg.velocity[steer_ind]
        # steer_effort = msg.effort[steer_ind]

        joint_states = JointState()
        joint_states.header = msg.header

        if steer_angle == 0.0:
            # TODO(lucasw) assume no rotation of link holding this joint
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
        # TODO(lucasw)
        for i in range(len(self.steered)):
            joint = self.steered[i]['joint']
            link = self.steered[i]['link']

            angle = self.get_angle(link, spin_center, steer_angle, msg.header.stamp)

            # print link, angle, dx, dy
            joint_states.name.append(joint)
            joint_states.position.append(angle)

            wheel_joint = self.steered[i]['wheel_joint']
            ind = self.wheel_joint_states.name.index(wheel_joint)
            # TODO(lucasw) angular_velocity for this joint needs
            # to be scaled by different in distance to spin center
            self.wheel_joint_states.position[ind] += angular_velocity * dt
            self.wheel_joint_states.velocity[ind] = angular_velocity

        for i in range(len(self.unsteered)):
            link = self.unsteered[i]['link']
            self.get_angle(link, spin_center, steer_angle, msg.header.stamp)

        self.get_angle(self.steer['link'], spin_center, steer_angle, msg.header.stamp)

        self.joint_pub.publish(joint_states)
        self.wheel_joint_states.header = joint_states.header
        self.joint_pub.publish(self.wheel_joint_states)

if __name__ == '__main__':
    acker = Acker()
    rospy.spin()
