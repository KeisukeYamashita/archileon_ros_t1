#!/usr/bin/env python
#coding: UTF-8
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import numpy as np
from sympy import *
import math
import time


# world_target_pos_list = [(0.5, 0.5), (0, 1.0), (-0.5, 1.5), (0, 2.0), (0.5, 2.5), (0, 3.0), (-0.5, 3.5), (0, 4.0)]
world_target_pos_list = [(0.5, 0.5), (0.0, 1.0), (-1.0, 1.5), (0, 2.0), (0.5, 1.5), (0, 1.0), (-0.5, -0.5), (0, 0)]

move_speed = 0.08

class world_target_goal_point:
    def set_point(self, x, y):
        self.x = x
        self.y = y

class cal_counter:
    def __init__(self):
        self.num = 0
    def add_count(self, kazu):
        self.num = self.num + kazu

class now_move_curve:
    def set_move_curve(self, move_curve):
        self.m_curve = move_curve

class diff_threshold:
    def set_diff_xy(self, x,y):
        self.diff_xy = [x,y]
    # def compare(self):

# class time_checker:
#     # def __init__(self):
#     #     self.s_time = None
#     def __init__(self):
#         self.s_switch = False
#     def set_start_time(self, time):
#         self.s_time = time
#         self.s_switch = True
#     def set_move_time(self, time):
#         self.m_time = time
#
#     def reset_start_time(self):
#         self.s_time = False

def judge_rob_goal(temporal_world_rob_x, temporal_world_rob_y, world_goal_x, world_goal_y):
    result = False
    diff_x = temporal_world_rob_x - world_goal_x
    diff_y = temporal_world_rob_y - world_goal_y

    if cal_counter.num == 1:
        diff_thr.set_diff_xy(diff_x, diff_y)
        print("diff_thr x " + str(diff_thr.diff_xy[0]))
        print("diff_thr y " + str(diff_thr.diff_xy[1]))
    else:
        if abs(diff_x) + abs(diff_y) <= diff_thr.diff_xy[0] + diff_thr.diff_xy[1]:
            diff_thr.set_diff_xy(diff_x, diff_y)
        if diff_thr.diff_xy[0] > abs(diff_x) and diff_thr.diff_xy[1] > abs(diff_y):
            print("/////////////////////////////////////////////////")
            print("diff_x is " + str(diff_x))
            print("diff_y is " + str(diff_y))
            result = True
    # if math.sqrt(diff_x**2 + diff_y**2) <= math.sqrt(diff_thr.diff_xy[0]**2 + diff_thr.diff_xy[1]**2):
    #     diff_thr.set_diff_xy(diff_x, diff_y)

    # if abs(diff_x) + abs(diff_y) <= (0.005)*2:

    return result

def cal_move_curve(world_rob_x, world_rob_y, world_rob_theta):
    print(world_target_pos_list)
    print(" ")
    print("world_rob_x " + str(world_rob_x))
    print("world_rob_y " + str(world_rob_y))
    print("world_rob_theta " + str(world_rob_theta))
    print(" ")

    goal_x,goal_y = world_target_pos_list[0][0],world_target_pos_list[0][1]
    world_target_goal_point.set_point(goal_x, goal_y)
    print("world_target_goal_point.x " + str(world_target_goal_point.x))
    print("world_target_goal_point.y " + str(world_target_goal_point.y))
    print(" ")
    world_target_pos_list.pop(0)

    pr_x = world_target_goal_point.x - world_rob_x
    pr_y = world_target_goal_point.y - world_rob_y

    print("pr_x is " + str(pr_x))
    print("pr_y is " + str(pr_y))
    print(" ")

    pr = np.array([pr_x, pr_y])

    if world_rob_theta < 0:
        print("world_rob_theta is MINUS " + str(world_rob_theta))
        print(" ")
        cos = np.cos(world_rob_theta)
        sin = np.sin(world_rob_theta)
        rotate = np.array([[cos, -sin], [sin, cos]])

    if world_rob_theta >= 0:
        print("world_rob_theta is PLUS " + str(world_rob_theta))
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

    if rob_target_y == 0:
        rad == 0
    elif 0 < rob_target_y:
        rad = math.atan2(rob_target_x, (radius - rob_target_y))
    elif rob_target_y < 0:
        rad = math.atan2(rob_target_x, (rob_target_y - radius))

    arc = abs(2 * radius * math.pi * (math.degrees(rad)/360))
    move_time = arc/move_speed
    move_curve = radius
    print("動く時間 " + str(move_time))
    print("曲率" + str(1.0 / move_curve))

    return move_curve, move_time

def callback(msg):
    world_rob_x = msg.data[0]
    world_rob_y = msg.data[1]
    world_rob_theta = msg.data[2]

    # print("world_rob_x " + str(world_rob_x))
    # print("world_rob_y " + str(world_rob_y))
    # print("world_rob_theta " + str(world_rob_theta))
    # print(" ")

    if cal_counter.num == 0:
        print("/////////////////////////////////////////////////")
        print("First move curve")
        print(" ")

        move_curve, move_time = cal_move_curve(world_rob_x, world_rob_y, world_rob_theta)
        now_move_curve.set_move_curve(move_curve)
        #
        # time_checker.set_move_time(move_time)

    else:
        # print("Else")
        # print(" ")
        rob_is_goal = judge_rob_goal(world_rob_x, world_rob_y, world_target_goal_point.x, world_target_goal_point.y)
        if rob_is_goal is True:
            print("/////////////////////////////////////////////////")
            print("Calculate the next goal point")
            print(" ")
            print("diff_xy is " + str(diff_xy))
            print(" ")
            move_curve, move_time= cal_move_curve(world_rob_x, world_rob_y, world_rob_theta)
            print("now_move_curve is " + str(now_move_curve.m_curve))
            now_move_curve.set_move_curve(move_curve)
            # print("now_move_curve is " + str(now_move_curve.m_curve))
            # time_checker.set_move_time(move_time)
            # time_checker.set_start_time(time.time())

        # if (time_checker.m_time + 2.5) <= float(time.time()) - float(time_checker.s_time):
        #     print("/////////////////////////////////////////////////")
        #     print("Time is over")
        #     print(" ")
        #     print("diff_xy is " + str(diff_xy))
        #     print(" ")
        #
        #     move_curve, move_time = cal_move_curve(world_rob_x, world_rob_y, world_rob_theta)
        #     now_move_curve.set_move_curve(move_curve)
        #     time_checker.set_move_time(move_time)
        #     time_checker.set_start_time(time.time())

    # print(time_checker.m_time - (float(time.time()) - float(time_checker.s_time)))
    cal_counter.add_count(1)
    pub_speed.publish(move_speed)
    pub_curve.publish(1.0 / now_move_curve.m_curve)

    # if time_checker.s_switch is not True:
    #     time_checker.set_start_time(time.time())
    # if time_checker.s_time is not None:
    #     time_checker.set_start_time(time.time())

# time_checker = time_checker()
diff_thr = diff_threshold()
cal_counter = cal_counter()
now_move_curve = now_move_curve()
world_target_goal_point = world_target_goal_point()


rospy.init_node("auto_control")
pub_speed = rospy.Publisher('move_speed', Float32, queue_size=1000)
pub_curve = rospy.Publisher('move_curve', Float32, queue_size=1000)
sub = rospy.Subscriber("robot_status", Float32MultiArray, callback)
rospy.spin()
