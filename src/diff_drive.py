#!/usr/bin/env python
import math
import time
import rospy
from geometry_msgs.msg import Twist
from roboclaw_driver.msg import SpeedCommand

encoder_resolution = 1920
angular_velocity_multiplier = 1.4
wheel_separation = 0.85
wheel_radius = 0.15
wheel_accel = 6000
wheel_stop = 1


def callback_cmd(data):
    msg = data
    x=msg.linear.x
    z=msg.angular.z

    z *= angular_velocity_multiplier

    wheel_L_lin_vel = x - (z * wheel_separation / 2)
    wheel_R_lin_vel = x + (z * wheel_separation / 2)

    wheel_L_ang_vel = wheel_L_lin_vel / wheel_radius
    wheel_R_ang_vel = wheel_R_lin_vel / wheel_radius

    enc_L_speed = encoder_resolution * wheel_L_ang_vel / (2 * math.pi)
    enc_R_speed = encoder_resolution * wheel_R_ang_vel / (2 * math.pi)

    pub_motor(enc_L_speed, enc_R_speed)


def pub_motor(motor_L, motor_R):
    msg = SpeedCommand()

    msg.m1_qpps = motor_L
    msg.m2_qpps = motor_L
    msg.accel = wheel_accel
    msg.max_secs = wheel_stop

    pub_speed_l.publish(msg)
    # print msg

    msg.m1_qpps = motor_R
    msg.m2_qpps = motor_R

    pub_speed_r.publish(msg)
    # print msg


if __name__ == "__main__":
    try:
        rospy.init_node('diff_control')

        pub_speed_r = rospy.Publisher("roboclaw_R/speed_command", SpeedCommand, queue_size=10)

        pub_speed_l = rospy.Publisher("roboclaw_L/speed_command", SpeedCommand, queue_size=10)

        cmd = rospy.Subscriber('cmd_vel', Twist, callback_cmd)

    except rospy.ROSInterruptException as e:
        rospy.logerr(e)

    rospy.spin()
