#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import numpy as np
from sympy import *

world_target_position = [(1, 1)]

x = Symbol("x")
y = Symbol("y")
a = Symbol("a")
fx = x**2 + (y-a)**2 - a**2

def callback(msg):
    world_x = msg.data[0]
    world_y = msg.data[1]
    world_theta = msg.data[2]

    pr_x = world_target_position[0][0] - world_x
    pr_y = world_target_position[0][1] - world_y
    pr = np.array([pr_x, pr_y])

    cos = np.cos(world_theta)
    sin = np.sin(world_theta)

    reverse_rotate = np.array([[cos, sin], [-sin, cos]])
    rob_target_position = np.dot(reverse_rotate, pr)
    x = rob_target_position[0]
    y = rob_target_position[1]

    a = (x**2 + y**2)/ 2 * y
    move_curve = a
    print(move_curve)
    pub_curve.publish(move_curve)

    print("x value is " + str(x))
    print("y value is " + str(y))
    print("theta value is " + str(theta))




rospy.init_node("auto_control")
pub_speed = rospy.Publisher('move_speed', Float32, queue_size=1000)
pub_curve = rospy.Publisher('move_curve', Float32, queue_size=1000)

rate = rospy.Rate(10)
move_speed = 0.5
# move_curve =  0

sub = rospy.Subscriber("robot_status", Float32MultiArray, callback)

while not rospy.is_shutdown():
    print(move_speed)
    pub_speed.publish(move_speed)

    rate.sleep()
