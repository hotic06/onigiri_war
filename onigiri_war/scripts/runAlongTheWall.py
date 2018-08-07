#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2
import sensor_msgs.msg

class WallBot():
    

    def __init__(self, bot_name):
        # bot name 
        self.name = bot_name
        
        # internal step number
        self.step_num=0

        #
        self.distance=0
        self.robot_angle=0
        self.angle_mode="left"
        self.distance_mode="top"
        self.twist=Twist()
        self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
        self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0
        
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.twist_state_pub = rospy.Publisher('twist/state', Float64 , queue_size=1)
        self.wall_state_pub = rospy.Publisher('wall/state', Float64,queue_size=1)
        self.twist_enable = rospy.Publisher('twist/pid_enable',Bool,queue_size=1)
        self.wall_enable = rospy.Publisher('wall/pid_enable',Bool,queue_size=1)
        self.wall_setpoint = rospy.Publisher('wall/setpoint',Float64,queue_size=1)
        self.twist_setpoint = rospy.Publisher('twist/setpoint',Float64,queue_size=1)

    def twist_wall_run(self,twist_enable,twist_value,wall_enable,wall_value,tol,next_step):
        self.twist_enable.publish(False)
        self.wall_enable.publish(False)
        self.twist_setpoint.publish(twist_value)
        self.wall_setpoint.publish(wall_value)
        self.twist_enable.publish(twist_enable)
        self.wall_enable.publish(wall_enable)
        if wall_enable and abs(self.distance - wall_value)<tol and \
           twist_enable and abs(self.robot_angle - twist_value)<tol                       or \
           wall_enable and abs(self.distance - wall_value)<tol and not(twist_enable)  or \
           twist_enable and abs(self.robot_angle - twist_value)<tol and not(wall_enable):
            self.robotStop()
            self.step_num=next_step
            rospy.logout("Step goto " + str(next_step))


    def strategy(self):
        r = rospy.Rate(1) # change speed 1fps

        while not rospy.is_shutdown():
            if self.step_num < 10: #前に進む
                self.distance_mode="top"
                self.twist_wall_run(False,0,True,3,0.1,20)
            elif self.step_num<= 20: #後ろに戻る
                self.twist_wall_run(False,0,True,15,0.01,30)
            elif self.step_num<=30: #左に向く
                self.angle_mode="left"
                self.distance_mode="back"
                self.twist_wall_run(True,90,False,0,0.01,40)
                temp_distance=self.distance
            elif self.step_num<=40: #左の的を取りに行く
                self.twist_wall_run(True,90,True,14.5,0.1,50)
            elif self.step_num<=50: #一旦戻る
                self.twist_wall_run(True,90,True,temp_distance,0.01,60)
            elif self.step_num<=60: #向き直す
                self.twist_wall_run(True,45,False,0,1,70)
            elif self.step_num<=70: #右を向く
                self.angle_mode="right"
                self.twist_wall_run(True,90,False,0,0.01,80)
            elif self.step_num<=80: #右の的を取りに行く
                self.twist_wall_run(True,90,True,14.5,0.1,90)
            elif self.step_num<=90: #右の壁を向く
                self.distance_mode="top"
                self.twist_wall_run(True,135,False,0,0.01,100)
            elif self.step_num<=100: #壁に向かう
                self.twist_wall_run(False,0,True,4,0.1,110)
            elif self.step_num<=110: #壁に沿うよう向きを治す
                self.twist_wall_run(True,90,False,0,0.01,120)
            elif self.step_num<=120: #壁に向かう
                self.twist_wall_run(True,90,True,3,0.1,130)
            elif self.step_num < 130: #中央の的を向く
                self.twist_wall_run(True,45,False,0,0.01,140)
            elif self.step_num<= 140: #前に行く
                self.twist_wall_run(False,0,True,3,0.1,150)
                self.angle_mode="left"
            elif self.step_num<=150: #途中まで戻る
                self.twist_wall_run(False,0,True,15,0.1,160)
            elif self.step_num<=160: #左を向く
                self.twist_wall_run(True,90,False,0,0.01,170)
            r.sleep()

    def robotStop(self):
#        twist=Twist()
        self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
        self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0
        self.vel_pub.publish(self.twist)


    def twistCallback(self,data):
        if self.angle_mode=="left":
            self.twist.angular.z = -data.data
        else:
            self.twist.angular.z = data.data
        print(self.twist)
        self.vel_pub.publish(self.twist)

    def wallCallback(self,data):
        if self.distance_mode=="top":
            self.twist.linear.x = -data.data
        else:
            self.twist.linear.x = data.data
        print(self.twist)
        self.vel_pub.publish(self.twist)


    def robot_angle_left_Callback(self,data):
        if self.angle_mode=="left":
            self.robot_angle=data.data
            self.twist_state_pub.publish(data.data)

    def robot_angle_right_Callback(self,data):
        if self.angle_mode=="right":
            self.robot_angle=data.data
            self.twist_state_pub.publish(data.data)

    def distance_top_Callback(self,data):
        if self.distance_mode=="top":
            self.distance=data.data
            self.wall_state_pub.publish(data.data)

    def distance_back_Callback(self,data):
        if self.distance_mode=="back":
            self.distance=data.data
            self.wall_state_pub.publish(data.data)


if __name__ == '__main__':
    rospy.init_node('wall_run')
    bot = WallBot('WallRun')

    rospy.Subscriber("twist/control_effort",Float64,bot.twistCallback)
    rospy.Subscriber("wall/control_effort",Float64,bot.wallCallback)
    rospy.Subscriber("robot_angle_left",Float64,bot.robot_angle_left_Callback)
    rospy.Subscriber("robot_angle_right",Float64,bot.robot_angle_right_Callback)
    rospy.Subscriber("distance_top",Float64,bot.distance_top_Callback)
    rospy.Subscriber("distance_back",Float64,bot.distance_back_Callback)


    bot.strategy()

