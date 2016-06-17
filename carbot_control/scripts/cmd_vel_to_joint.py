#!/usr/bin/env python
# Lucas Walter
# June 2016
#
# Subscribe to a cmd_vel Twist message and interpret the linear
# and angular components into a joint state for the virtual steer joint.

import math
import rospy
import tf2_py as tf2
import tf2_ros

from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import JointState
# from tf import transformations


class CmdVelToJoint():
    def __init__(self):
        self.rate = rospy.get_param("~rate", 20.0)
        self.period = 1.0 / self.rate

        self.tf_buffer = tf2_ros.Buffer()
        self.tf = tf2_ros.TransformListener(self.tf_buffer)

        self.steer_link = rospy.get_param("~steer_link", "lead_steer")
        self.steer_joint = rospy.get_param("~steer_joint", "lead_steer_joint")
        # +/- this angle
        self.min_steer_angle = rospy.get_param("~min_steer_angle", -0.7)
        self.max_steer_angle = rospy.get_param("~max_steer_angle", 0.7)

        self.wheel_joint = rospy.get_param("~wheel_joint", "wheel_lead_axle")
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.15)
        # the spin center is always on the fixed axle y axis of the fixed axle,
        # it is assume zero rotation on the steer_joint puts the steering
        # at zero rotation with respect to fixed axle x axis (or xz plane)
        self.fixed_axle_link = rospy.get_param("~fixed_axle_link", "back_axle")

        self.point_pub = rospy.Publisher("cmd_vel_spin_center", PointStamped, queue_size=1)
        self.steer_pub = rospy.Publisher("steer_joint_states", JointState, queue_size=1)
        # TODO(lucasw) is there a way to get TwistStamped out of standard
        # move_base publishers?
        self.joint_state = JointState()
        self.joint_state.name.append(self.steer_joint)
        self.joint_state.position.append(0.0)
        self.joint_state.velocity.append(0.0)
        self.joint_state.name.append(self.wheel_joint)
        self.joint_state.position.append(0.0)
        self.joint_state.velocity.append(0.0)
        self.cmd_vel = Twist()
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback, queue_size=2)
        self.timer = rospy.Timer(rospy.Duration(self.period), self.update)

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg

    def update(self, event):
        # if self.cmd_vel is None:
        #     return

        steer_transform = self.tf_buffer.lookup_transform(self.fixed_axle_link,
                                                          self.steer_link,
                                                          rospy.Time(),
                                                          rospy.Duration(4.0))
        self.joint_state.header.stamp = steer_transform.header.stamp
        # if the cmd_vel is pure linear x, then the joint state is at zero
        # steer angle (no skid steering modelled).
        wheel_angular_velocity = 0
        if self.cmd_vel.linear.y == 0.0:
            self.joint_state.position[0] = 0.0
            self.joint_state.velocity[0] = 0.0
            wheel_angular_velocity = self.cmd_vel.linear.x / self.wheel_radius
        elif self.cmd_vel.linear.x == 0.0:
            # TODO(lucasw) what to do here?
            # Could do nothing, but that doesn't reflect the intent of the cmd_vel
            # source - next best thing may be to set the steer angle to
            # the maximum (which needs to be given to this node).

            # if the robot was near an obstacle this strategy doesn't work at all
            # so need to be able to select it with a mode.
            wheel_angular_velocity = self.cmd_vel.linear.y / self.wheel_radius
            if self.cmd_vel.linear.y > 0:
                self.joint_state.position[0] = self.min_steer_angle
            else:
                self.joint_state.position[0] = self.max_steer_angle
            self.joint_state.velocity[0] = 0.0
        else:
            # need to calculate the steer angle
            fixed_to_base = self.tf_buffer.lookup_transform(self.fixed_axle_link, "base_link",
                                                 rospy.Time(), rospy.Duration(4.0))
            # the angle traveled around the spin center
            lin_y = self.cmd_vel.linear.y
            lin_x = self.cmd_vel.linear.x
            lin_mag = math.sqrt(lin_x * lin_x + lin_y * lin_y)
            base_y = fixed_to_base.transform.translation.y
            base_x = fixed_to_base.transform.translation.x
            ty = lin_y + base_y
            tx = lin_x + base_x
            # this includes the angle base_link traveled and an offset angle
            # from fixed back link to base link
            lin_angle = math.atan2(lin_y, lin_x)
            # this connects the spin center to base_y + lin_y/2, base_x + lin_x/2
            radius_2 = (lin_x / 2.0 + base_x) / math.sin(lin_angle)
            spin_angle_traveled = 2.0 * math.atan2(lin_mag / 2.0, radius_2)
            # need to account for angle between back link and base link
            base_offset_angle = lin_angle - spin_angle_traveled / 2.0
            base_spin_radius = radius_2 / math.cos(spin_angle_traveled / 2.0)

            # this angle applies to all links on the vehicle, but each link
            # will have a different spin radius depending on where they are relative
            # to base link.
            back_spin_radius = base_spin_radius * math.cos(base_offset_angle) + base_y
            spin_center = PointStamped()
            spin_center.point.y = back_spin_radius
            spin_center.header.stamp = rospy.Time.now()
            spin_center.header.frame_id = self.fixed_axle_link
            self.point_pub.publish(spin_center)

            # get the transform from the back axle to the steer link
            fixed_to_steer = self.tf_buffer.lookup_transform(self.fixed_axle_link,
                                                             self.steer_link,
                                                             rospy.Time(),
                                                             rospy.Duration(4.0))
            steer_angle = math.atan2(fixed_to_steer.transform.translation.x,
                                     back_spin_radius - fixed_to_steer.transform.translation.y)
            # TODO(lucasw) need to handle steer angle > max steer angle
            distance_traveled = back_spin_radius * spin_angle_traveled
            wheel_angular_velocity = distance_traveled / self.wheel_radius
            # print distance_traveled, self.cmd_vel.linear.x, self.cmd_vel.linear.y
            self.joint_state.position[0] = -steer_angle
            self.joint_state.velocity[0] = 0.0

            print math.degrees(lin_angle), math.degrees(spin_angle_traveled), \
                    math.degrees(steer_angle), back_spin_radius, base_x
            # print base_offset_angle, base_spin_radius, back_spin_radius, base_x, base_y
            # print spin_angle_traveled, math.degrees(spin_angle_traveled), spin_radius

        # TODO(lucasw) assuming fixed period for now, could
        # measure actual dt with event parameter.
        self.joint_state.position[1] += wheel_angular_velocity * self.period
        self.joint_state.velocity[1] = wheel_angular_velocity

        self.steer_pub.publish(self.joint_state)

if __name__ == '__main__':
    rospy.init_node("cmd_vel_to_joint")
    cmd_vel_to_joint = CmdVelToJoint()
    rospy.spin()
