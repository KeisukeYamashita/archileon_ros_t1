#!/usr/bin/env python
#coding: UTF-8
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import numpy as np
from sympy import *
import math



world_target_pos_list = [(0.5, 0.5), (0, 1.0), (-0.5, 1.5), (0, 2.0), (0.5, 2.5)]
move_speed = 0.05


class world_target_goal_point:
    def set_point(self, x, y):
        self.x = x
        self.y = y


class world_rob_start_point:
    def set_point(self, x, y):
        self.x =x
        self.y =y

class cal_counter:
    def __init__(self):
        self.num = 0
    def set_count(self, num):
        self.num =num

class now_move_curve:
    def set_move_curve(self, move_curve):
        self.move_curve = move_curve

def judge_rob_is_goal(temporal_world_rob_x, temporal_world_rob_y):
    result = False
    if temporal_world_rob_x is world_target_goal_point.x and temporal_world_rob_y is world_target_goal_point.y:
        result = True
    return result

def cal_move_curve(world_rob_x, world_rob_y, world_rob_theta):
    print("world_rob_x " + str(world_rob_x))
    print("world_rob_y " + str(world_rob_y))
    print("world_rob_theta " + str(world_rob_theta))
    print(" ")
    print("world_target_goal_point.x " + str(world_target_goal_point.x))
    print("world_target_goal_point.y " + str(world_target_goal_point.y))
    print(" ")

    pr_x = world_target_goal_point.x - world_rob_x
    pr_y = world_target_goal_point.y - world_rob_y

    print("pr_x is " + str(pr_x))
    print("pr_y is " + str(pr_y))
    print(" ")

    print(world_target_pos_list)
    print(" ")

    pr = np.array([pr_x, pr_y])

    if world_rob_theta < 0:
        print("world_rob_theta is MINUS")
        print(world_rob_theta)
        print(" ")
        cos = np.cos(world_rob_theta)
        sin = np.sin(world_rob_theta)
        rotate = np.array([[cos, -sin], [sin, cos]])

    if world_rob_theta >= 0:
        print("world_rob_theta is PLUS")
        print(world_rob_theta)
        print(" ")
        cos = np.cos(world_rob_theta)
        sin = np.sin(world_rob_theta)
        rotate = np.array([[cos, sin], [-sin, cos]])


    rob_target_position = np.dot(rotate, pr)
    rob_target_x = rob_target_position[0]
    rob_target_y = rob_target_position[1]

    print("rob_target_x  is " + str(rob_target_x))
    print("rob_target_y  is " + str(rob_target_y))
    print(" ")

    radius = ((rob_target_x**2) + (rob_target_y**2) ) / (2 * rob_target_y)
    print("a is " + str(radius))
    print(" ")

    if rob_target_y == 0:
        rad == 0
    elif 0 < rob_target_y:
        rad = math.atan2(rob_target_x, (radius - rob_target_y))
    elif rob_target_y < 0:
        rad = math.atan2(rob_target_x, (rob_target_y - radius))

    move_curve = radius
    print("曲率" + str(1.0 / move_curve))

    return move_curve

def callback(msg):
    world_rob_x = msg.data[0]
    world_rob_y = msg.data[1]
    world_rob_theta = msg.data[2]

    print("world_rob_x " + str(world_rob_x))
    print("world_rob_y " + str(world_rob_y))
    print(" ")

    # if cal_counter.num == 0:
    #     print("First move curve")
    #     print(" ")
    #     move_curve = cal_move_curve(world_rob_x, world_rob_y, world_rob_theta)
    #     now_move_curve.set_move_curve(move_curve)
    # else:
    #     print("Else")
    #     print(" ")
    #     if judge_rob_is_goal(world_rob_x, world_rob_y):
    #         print("Calculate the next goal point")
    #         print(" ")
    #         move_curve = cal_move_curve(world_rob_x, world_rob_y, world_rob_theta)
    #         now_move_curve.set_move_curve(move_curve)
    #         world_target_goal_point.set_point(world_target_pos_list[0][0],world_target_pos_list[0][1])
    #         world_target_pos_list.pop(0)
    # cal_counter.set_count(1)
    #
    # count = 0
    # rate = rospy.Rate(5)
    # while not rospy.is_shutdown():
    #     pub_curve.publish(1.0 / now_move_curve.move_curve)
    #     if count >= 4:
    #         print("break")
    #         break
    #     count +=1
    #     rate.sleep()

cal_counter = cal_counter()
now_move_curve = now_move_curve()
now_move_curve.set_move_curve(1.0)

world_target_goal_point = world_target_goal_point()
world_target_goal_point.set_point(world_target_pos_list[0][0],world_target_pos_list[0][1])
world_target_pos_list.pop(0)

rospy.init_node("auto_control")
pub_speed = rospy.Publisher('move_speed', Float32, queue_size=1000)
pub_curve = rospy.Publisher('move_curve', Float32, queue_size=1000)
sub = rospy.Subscriber("robot_status", Float32MultiArray, callback)
rate = rospy.Rate(1)
while not rospy.is_shutdown():
    pub_speed.publish(move_speed)
    pub_curve.publish(1.0 / now_move_curve.move_curve)
    rate.sleep()
