#!/usr/bin/env python
# Copyright 2016 Lucas Walter

import math
import numpy
import rospy
import tf2_py as tf2
import tf2_ros

from geometry_msgs.msg import Point, PointStamped, TransformStamped, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
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
        # TODO(lucasw) need to know per wheel radius to compute velocity
        # correctly, for now assume all wheels are same radius
        self.joints = rospy.get_param("~steered_joints",
                                       [{'link': 'front_left_steer',
                                         'steer_joint': 'front_left_steer_joint',
                                         'steer_topic': '/carbot/front_left_steer_position_controller/command',
                                         'wheel_joint': 'wheel_front_left_axle',
                                         'wheel_topic': '/carbot/wheel_front_left_position_controller/command'},
                                        {'link': 'front_right_steer',
                                         'steer_joint': 'front_right_steer_joint',
                                         'steer_topic': '/carbot/front_right_steer_position_controller/command',
                                         'wheel_joint': 'wheel_front_right_axle',
                                         'wheel_topic': '/carbot/wheel_front_right_position_controller/command'},
                                        {'link': 'back_left',
                                         'steer_joint': None,
                                         'steer_topic': None,
                                         'wheel_joint': 'wheel_back_left_axle',
                                         'wheel_topic': '/carbot/wheel_back_left_position_controller/command'},
                                        {'link': 'back_right',
                                         'steer_joint': None,
                                         'steer_topic': None,
                                         'wheel_joint': 'wheel_back_right_axle',
                                         'wheel_topic': '/carbot/wheel_back_right_position_controller/command'}])

        self.wheel_radius = rospy.get_param("~wheel_radius", 0.15)
        # gazebo joint controller commands
        self.command_pub = {}
        # use this to store all the positions persistently
        self.wheel_joint_states = JointState()
        for wheel in self.joints:
            steer_topic = wheel['steer_topic']
            if steer_topic is not None:
                self.command_pub[wheel['steer_joint']] = rospy.Publisher(steer_topic,
                                                                         Float64, queue_size=4)
            wheel_topic = wheel['wheel_topic']
            if wheel_topic is not None:
                self.command_pub[wheel['wheel_joint']] = rospy.Publisher(wheel_topic,
                                                                         Float64, queue_size=4)

            # TODO(lucasw) read the current angle to initialize
            self.wheel_joint_states.name.append(wheel['wheel_joint'])
            self.wheel_joint_states.position.append(0.0)
            self.wheel_joint_states.velocity.append(0.0)

        # TODO(lucasw) initialize the current position from
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
        # TODO(lucasw) would like there to be a gazebo plugin that takes
        # desired joint position and velocity, but I believe there is only
        # the simple command inputs that pid to a position or velocity, but not both.
        self.joint_pub = rospy.Publisher("steered_joint_states", JointState, queue_size=3)
        self.steer = rospy.get_param("~steer", {'link': 'lead_steer',
                                                'joint': 'lead_steer_joint',
                                                'wheel_joint': 'wheel_lead_axle'})
        self.twist_pub = rospy.Publisher("odom_cmd_vel", Twist, queue_size=3)
        # TODO(lucasw) circular publisher here- the steer command is on
        # joint_states, then published onto steered_joint_states, which updates
        # joint_states- but the joints are different.
        self.joint_sub = rospy.Subscriber("joint_states", JointState,
                                          self.steer_callback, queue_size=4)

        self.timer = rospy.Timer(rospy.Duration(self.period), self.update)

    def update(self, event):
        pass

    # get the angle to to the wheel from the spin center
    def get_angle(self, link, spin_center, steer_angle, stamp):
        # lookup the position of each link in the back axle frame
        ts = self.tf_buffer.lookup_transform(spin_center.header.frame_id, link,
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
        wheel_ind = msg.name.index(self.steer['wheel_joint'])
        if len(msg.velocity) < wheel_ind:
            rospy.logwarn("no velocity for wheel_ind %d", wheel_ind)
            return

        steer_angle = msg.position[steer_ind]
        lead_wheel_angular_velocity = msg.velocity[msg.name.index(self.steer['wheel_joint'])]
        # steer_velocity = msg.velocity[steer_ind]
        # steer_effort = msg.effort[steer_ind]

        joint_states = JointState()
        joint_states.header = msg.header
        self.wheel_joint_states.header = msg.header
        odom_cmd_vel = Twist()

        # TODO(lucaw) possibly eliminate this pathway and have
        # special case code below to mix in this special case.
        if steer_angle == 0.0:
            # TODO(lucasw) assume no rotation of link holding this joint
            # with respect to the back axle link
            for i in range(len(self.joints)):
                steer_joint = self.joints[i]['steer_joint']
                if steer_joint is not None:
                    joint_states.name.append(steer_joint)
                    joint_states.position.append(steer_angle)
                    # joint_states.velocity.append(steer_velocity)
                    # joint_states.effort.append(steer_effort)
                if steer_joint in self.command_pub.keys():
                    self.command_pub[steer_joint].publish(steer_angle)

                wheel_joint = self.joints[i]['wheel_joint']
                ind = self.wheel_joint_states.name.index(wheel_joint)
                # TODO(lucasw) this assumes all wheels have the same radius
                self.wheel_joint_states.position[ind] += lead_wheel_angular_velocity * dt
                self.wheel_joint_states.velocity[ind] = lead_wheel_angular_velocity
                if wheel_joint in self.command_pub.keys():
                    self.command_pub[wheel_joint].publish(self.wheel_joint_states.position[ind])

            # TODO(lucasw) combine these into one message
            self.joint_pub.publish(joint_states)
            self.joint_pub.publish(self.wheel_joint_states)

            # upate odometry
            # TODO(lucasw) this doesn't model skid steering at all, it assume an
            # off-axis wheel driving forward will drive the robot forward because
            # all the wheels balance.
            distance = self.wheel_radius * lead_wheel_angular_velocity * dt
            if dt > 0:
                odom_cmd_vel.linear.x = distance / dt
            self.twist_pub.publish(odom_cmd_vel)

            self.ts.transform.translation.x += distance * math.cos(self.angle)
            self.ts.transform.translation.y += distance * math.sin(-self.angle)

            # set the quaternion to current angle
            quat = transformations.quaternion_from_euler(0, 0, -self.angle)
            self.ts.transform.rotation.x = quat[0]
            self.ts.transform.rotation.y = quat[1]
            self.ts.transform.rotation.z = quat[2]
            self.ts.transform.rotation.w = quat[3]

            self.ts.header.stamp = msg.header.stamp
            # convert self.angle to quaternion
            self.br.sendTransform(self.ts)
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
        # TODO(lucasw) assume no rotation for now- zero steer angle
        # means the steer joint is aligned with x axis of the robot
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

            if joint in self.command_pub.keys():
                self.command_pub[joint].publish(angle)

            wheel_joint = self.joints[i]['wheel_joint']
            ind = self.wheel_joint_states.name.index(wheel_joint)
            # TODO(lucasw) lead_wheel_angular_velocity for this joint needs
            # to be scaled by different in distance to spin center
            self.wheel_joint_states.position[ind] += lead_wheel_angular_velocity * fr * dt
            self.wheel_joint_states.velocity[ind] = lead_wheel_angular_velocity * fr
            if wheel_joint in self.command_pub.keys():
                self.command_pub[wheel_joint].publish(self.wheel_joint_states.position[ind])

        # update odometry
        # There may be another odometric frame in the future, based off
        # actual encoder values rather than desired
        # TODO(lucasw) need a steer joint at the base_link and set it
        # to this angle
        steer_angle, radius = self.get_angle("base_link", spin_center,
                                       steer_angle, msg.header.stamp)
        fr = radius / lead_radius
        # distance traveled along the radial path in base_link
        distance = self.wheel_radius * lead_wheel_angular_velocity * fr * dt
        angle_traveled = distance / radius
        # the distance traveled in the base_link frame:
        dx_in_ts = distance * math.cos(steer_angle)
        dy_in_ts = -distance * math.sin(steer_angle)
        if dt > 0:
            odom_cmd_vel.linear.x = dx_in_ts / dt
            odom_cmd_vel.linear.y = dy_in_ts / dt
            # print math.degrees(steer_angle), distance, odom_cmd_vel.linear.x, \
            #         odom_cmd_vel.linear.y, radius, math.degrees(angle_traveled)
            self.twist_pub.publish(odom_cmd_vel)

        # then need to rotate x and y by self.angle
        dx_in_parent = distance * math.cos(self.angle + steer_angle)
        dy_in_parent = -distance * math.sin(self.angle + steer_angle)
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
        self.joint_pub.publish(self.wheel_joint_states)

        if steer_angle > 0:
            self.angle += angle_traveled
        else:
            self.angle -= angle_traveled

if __name__ == '__main__':
    acker = Acker()
    rospy.spin()
