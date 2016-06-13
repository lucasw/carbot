#!/usr/bin/env python
# Copyright 2016 Lucas Walter

import math
import numpy
import rospy
import tf2_py as tf2
import tf2_ros

from geometry_msgs.msg import Point, PointStamped, TransformStamped
from sensor_msgs.msg import JointState
from tf import transformations
from visualization_msgs.msg import Marker


class Acker():
    def __init__(self):
        rospy.init_node("acker")
        self.rate = rospy.get_param("~rate", 20.0)
        self.period = 1.0 / self.rate

        self.tf_buffer = tf2_ros.Buffer()
        self.tf = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf2_ros.TransformBroadcaster()

        # get a dict of joints and their link locations
        # TODO(lucasw) need to know wheel radius to compute velocity
        # correctly, for now assume all wheels are same radius
        self.joints = rospy.get_param("~steered_joints",
                                       [{'link': 'front_left_steer',
                                         'steer_joint': 'front_left_steer_joint',
                                         'wheel_joint': 'wheel_front_left_axle'},
                                        {'link': 'front_right_steer',
                                         'steer_joint': 'front_right_steer_joint',
                                         'wheel_joint': 'wheel_front_right_axle'},
                                        {'link': 'back_left',
                                         'steer_joint': None,
                                         'wheel_joint': 'wheel_back_left_axle'},
                                        {'link': 'back_right',
                                         'steer_joint': None,
                                         'wheel_joint': 'wheel_back_right_axle'}])

        self.wheel_radius = rospy.get_param("~wheel_radius", 0.15)
        # use this to store all the positions persistently
        self.wheel_joint_states = JointState()
        for wheel in self.joints:
            # TODO(lucasw) read the current angle to initialize
            self.wheel_joint_states.name.append(wheel['wheel_joint'])
            self.wheel_joint_states.position.append(0.0)
            self.wheel_joint_states.velocity.append(0.0)

        # TODO(lwalter) initialize the current position from
        # another source
        self.ts = TransformStamped()
        self.ts.header.frame_id = "map"
        self.ts.child_frame_id = "base_link"
        self.ts.transform.rotation.w = 1.0
        # the angle of the base_link
        self.angle = 0

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

        self.marker_pub = rospy.Publisher("marker", Marker, queue_size=len(self.joints) * 2)
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

    # get the angle to to the wheel from the spin center
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
        return angle, radius

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
            for i in range(len(self.joints)):
                joint = self.joints[i]['steer_joint']
                if joint is None:
                    continue
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

        angle, lead_radius = self.get_angle(self.steer['link'], spin_center,
                                       steer_angle, msg.header.stamp)
        # TODO(lucasw) assume no rotation for now
        # TODO(lucasw)
        for i in range(len(self.joints)):
            joint = self.joints[i]['steer_joint']
            link = self.joints[i]['link']

            angle, radius = self.get_angle(link, spin_center, steer_angle, msg.header.stamp)
            fr = radius / lead_radius

            # print link, angle, dx, dy
            if joint is not None:
                joint_states.name.append(joint)
                joint_states.position.append(angle)

            wheel_joint = self.joints[i]['wheel_joint']
            ind = self.wheel_joint_states.name.index(wheel_joint)
            # TODO(lucasw) angular_velocity for this joint needs
            # to be scaled by different in distance to spin center
            self.wheel_joint_states.position[ind] += angular_velocity * fr * dt
            self.wheel_joint_states.velocity[ind] = angular_velocity * fr

        # update odometry
        # There may be another odometric frame in the future, base off
        # actual encoder values rather than desired
        angle, radius = self.get_angle("base_link", spin_center,
                                       steer_angle, msg.header.stamp)
        fr = radius / lead_radius
        # distance travelled along the radial path
        distance = angular_velocity * fr * dt
        angle_traveled = distance / radius
        if steer_angle > 0:
            self.angle += angle_traveled
        else:
            self.angle -= angle_traveled
        # the distance travelled in the current frame:
        dx_in_ts = radius * math.sin(angle_traveled)
        dy_in_ts = radius * (1.0 - math.cos(angle_traveled))
        ca = math.cos(self.angle)
        sa = math.sin(self.angle)
        dx_in_parent = ca * dx_in_ts + sa * dy_in_ts
        dy_in_parent = -sa * dx_in_ts + ca * dy_in_ts
        # then need to rotate x and y by self.angle
        # (use a 2d rotation matrix)
        # then can add the rotated x and y to self.ts.transform.translation
        self.ts.transform.translation.x += dx_in_parent
        self.ts.transform.translation.y += dy_in_parent

        # set the quaternion to current angle
        quat = transformations.quaternion_from_euler(0, 0, -self.angle)
        self.ts.transform.rotation.x = quat[0]
        self.ts.transform.rotation.y = quat[1]
        self.ts.transform.rotation.z = quat[2]
        self.ts.transform.rotation.w = quat[3]

        self.ts.header.stamp = msg.header.stamp
        # convert self.angle to quaternion
        self.br.sendTransform(self.ts)

        self.joint_pub.publish(joint_states)
        self.wheel_joint_states.header = joint_states.header
        self.joint_pub.publish(self.wheel_joint_states)

if __name__ == '__main__':
    acker = Acker()
    rospy.spin()
