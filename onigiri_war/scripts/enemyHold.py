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
import tf
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import math
import numpy as np




class EnemyHold():
    

    def __init__(self, bot_name):
        # bot name 
        self.name = bot_name        

        #
        self.enemy_angle=-1
        self.twist=Twist()
        self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
        self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0
        self.twist_enable=True #動いているかFlag
        self.rush_mode=True #Green発見したら突進する
        self.walk_mode=True #Wood発見したら突進する
        self.green_found=False #緑マーカー発見Flag
        self.collision=False #衝突が発生したFlag
        self.rotate_mode=False #一定角度回転モードFlag
        self.rotate_mode_last=0 # 前回の回転角
        self.rotate_mode_count=0 #何回連続回転モードを使ったか

        self.odom=Odometry()
        self.odom.pose.pose.orientation.z=-9999

        #self.mode="camera" #"lidar" # or "camera"

        # publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.enemyhold_state_pub = rospy.Publisher('enemyhold/state', Float64 , queue_size=1)
        self.enemyhold_enable_pub = rospy.Publisher('enemyhold/pid_enable', Bool , queue_size=1)
        self.enemyhold_setpoint_pub = rospy.Publisher('enemyhold/setpoint', Float64 , queue_size=1)


    def strategy(self):
        r = rospy.Rate(5) # change speed 1fps

        while not rospy.is_shutdown():
            self.enemyhold_setpoint_pub.publish(0)
            #self.rotate_robot_loop()
            r.sleep()

    """
    def angleCallback(self,data):
        if self.mode=="lidar":
            if np.isnan(data.data):
                self.enemyhold_enable_pub.publish(False)
                self.robotStop()
            else:
                self.enemyhold_state_pub.publish(data.data)
                self.enemyhold_enable_pub.publish(True)
                self.twist_enable=True
    """

    def greenCameraCallback(self,data):
        #if self.mode=="camera":
        if self.collision==False:
            if np.isnan(data.data):
                self.green_found=False
                #self.twist.linear.x=0
            else:
                rospy.logout("green found " + str(data.data))
                self.green_found=True
                self.rotate_mode=False
                self.rotate_mode_count=0
                self.enemyhold_state_pub.publish(data.data)
                self.enemyhold_enable_pub.publish(True)
                self.twist_enable=True
                if self.rush_mode:
                    if abs(data.data) < 5:
                        self.twist.linear.x=0.3
                    else:
                        self.twist.linear.x=0

    def angleCameraCallback(self,data):
        if self.green_found==False:
            if np.isnan(data.data):
                #self.enemyhold_enable_pub.publish(False)
                #self.robotStop()
                #self.twist.linear.x=0
                #首をふる

                self.twist_enable=False               
                self.twist.linear.x=0
                self.twist.angular.z=1
                self.vel_pub.publish(self.twist)

            else:
                if self.collision==False:
                    rospy.logout("wood found")
                    self.rotate_mode=False
                    self.rotate_mode_count=0
                    self.enemyhold_state_pub.publish(data.data)
                    self.enemyhold_enable_pub.publish(True)
                    self.twist_enable=True
                    if self.walk_mode:
                        rospy.logout("walk " + str(data.data))
                        if abs(data.data) < 5:
                            self.twist.linear.x=0.1
                            self.vel_pub.publish(self.twist)
                        else:
                            self.twist.linear.x=0
                            


    def enemyholdCallback(self,data):
        if self.twist_enable:
            self.twist.angular.z = data.data
            self.vel_pub.publish(self.twist)
            #rospy.logout(self.twist)

    def collisionCallback(self,data):
        if not(np.isnan(data.data)):
            self.collision=True

            linearx=-0.2
            twistz=0.8
            angl=0.7

            twistz= -twistz * (abs(data.data)/data.data)
            #angl=(65-abs(data.data))/30

            self.robotStop()
            self.twist.linear.x=linearx
            self.vel_pub.publish(self.twist)
            rospy.sleep(0.3)
            self.robotStop()


            self.twist.angular.z = twistz
            self.vel_pub.publish(self.twist)
            rospy.sleep(angl)

            self.robotStop()
            self.twist.linear.x=-linearx
            self.vel_pub.publish(self.twist)
            rospy.sleep(0.3)
            self.robotStop()

            self.twist.angular.z = -twistz
            self.vel_pub.publish(self.twist)
            rospy.sleep(angl)

            self.robotStop()

        else:
            self.collision=False




    def robotStop(self):
        self.twist_enable=False
        self.enemyhold_enable_pub.publish(False)
        self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
        self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0
        self.vel_pub.publish(self.twist)
        rospy.logout("robot stop")

    def rotateRobot(self,angle):
        if self.odom.pose.pose.orientation.z<>-9999 and not(self.rotate_mode):
            self.rotate_angle=angle
            rospy.logout("rotate mode " + str(angle))
            self.rotate_mode=True
            self.rotate_mode_last=angle
            euler=self.quaternion_to_euler(self.odom.pose.pose.orientation)
            self.start_rotate_angle=euler

    def rotate_robot_loop(self):
        if self.rotate_mode==False:
            return

        angle=self.rotate_angle

        

        angle=angle/180.0*math.pi
        if angle>0:
            z=0.4
        else:
            z=-0.4
        self.twist.angular.z = z
        self.vel_pub.publish(self.twist)
        
        euler=self.quaternion_to_euler(self.odom.pose.pose.orientation)
        temp=euler.z-self.start_rotate_angle.z



        if angle > 0:
            if temp<0 :
                temp=2*math.pi+temp
        else:
            if temp>0 :
                temp=-2*math.pi+temp

        if abs(temp)<350/180*math.pi:
            if abs(temp)>=abs(angle):
                self.rotate_mode=False

        rospy.logout("rotate loop " + str(angle/math.pi*180)+" "+str(temp/math.pi*180))
        if self.rotate_mode==False:
            self.twist.angular.z=0
            self.vel_pub.publish(self.twist)
            rospy.logout("rotate finish")
            
    def odom_Callback(self,data):
        self.odom=data

    def quaternion_to_euler(self,quaternion):
        """Convert Quaternion to Euler Angles

        quarternion: geometry_msgs/Quaternion
        euler: geometry_msgs/Vector3
        """
        e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        return Vector3(x=e[0], y=e[1], z=e[2])

if __name__ == '__main__':
    rospy.init_node('enemy_hold')
    bot = EnemyHold('EnemyHold')


    #rospy.Subscriber("enemy_angle",Float64,bot.angleCallback)
    rospy.Subscriber("enemy_angle_camera",Float64,bot.angleCameraCallback,queue_size = 1)
    rospy.Subscriber("enemy_green_angle_camera",Float64,bot.greenCameraCallback,queue_size = 1)

    rospy.Subscriber("enemyhold/control_effort",Float64,bot.enemyholdCallback,queue_size = 1)

    rospy.Subscriber("collision",Float64,bot.collisionCallback,queue_size = 1)

    rospy.Subscriber("odom",Odometry,bot.odom_Callback,queue_size=1)

    bot.strategy()
