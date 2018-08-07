#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import math

from std_msgs.msg import Float64
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
        # velocity publisher
        self.distance_left = rospy.Publisher('distance_left', Float64 ,queue_size=1)
        self.distance_right = rospy.Publisher('distance_right', Float64,queue_size=1)
        self.distance_top = rospy.Publisher('distance_top',Float64,queue_size=1)
        self.distance_back = rospy.Publisher('distance_back',Float64,queue_size=1)
        self.robot_angle_left = rospy.Publisher('robot_angle_left',Float64,queue_size=1)
        self.robot_angle_right = rospy.Publisher('robot_angle_right',Float64,queue_size=1)
        self.wall_distance_left = rospy.Publisher('wall_distance_left',Float64,queue_size=1)

    def strategy(self):
        r = rospy.Rate(1) # change speed 1fps
        while not rospy.is_shutdown():
            r.sleep()

    def scanCallback(self,data):
        range_left=0
        range_right=0
        range_top=0
        range_back=0
        for k in range(-5,6): 
            range_left=range_left+data.ranges[90+k]
            range_right=range_right+data.ranges[270+k]
            range_back=range_back+data.ranges[180+k]
            if k>=0:
                range_top=range_top+data.ranges[0+k]
            else:
                range_top=range_top+data.ranges[360+k]
        range_left = range_left/11
        range_right = range_right/11
        
        #robot angle calculation Left
        L1=data.ranges[90-10]
        L2=data.ranges[90+10]
        rad=math.radians(10)
        x1=-L1*math.cos(rad)
        y1=L1*math.sin(rad)
        x2=-L2*math.cos(rad)
        y2=-L2*math.sin(rad)
        a=(y2-y1)/(x2-x1)

        if L1 > L2 :
            robot_angle_left = - math.degrees( math.atan(a) )
        elif L1 < L2 :
            robot_angle_left = 180 - math.degrees( math.atan(a) )
        else:
            robot_angle_left = 90


        #robot angle calculation Right
        L2=data.ranges[270-10]
        L1=data.ranges[270+10]
        rad=math.radians(10)
        x1=-L1*math.cos(rad)
        y1=L1*math.sin(rad)
        x2=-L2*math.cos(rad)
        y2=-L2*math.sin(rad)
        a=(y2-y1)/(x2-x1)

        if L1 > L2 :
            robot_angle_right = - math.degrees( math.atan(a) )
        elif L1 < L2 :
            robot_angle_right = 180 - math.degrees( math.atan(a) )
        else:
            robot_angle_right = 90


        #robot distance calculation
        b=y1-a*x1
        a2=a
        b2=-1
        c2=b

        Ld=abs(c2)/math.sqrt(a2**2+b2**2)
        
        self.distance_left.publish(range_left)
        self.distance_right.publish(range_right)
        self.distance_top.publish(range_top)
        self.distance_back.publish(range_back)

        self.robot_angle_left.publish(robot_angle_left)        
        self.robot_angle_right.publish(robot_angle_right)        
        self.wall_distance_left.publish(Ld)

if __name__ == '__main__':
    rospy.init_node('wall_distance')
    bot = WallBot('WallDistance')

    rospy.Subscriber("scan",sensor_msgs.msg.LaserScan,bot.scanCallback)

    bot.strategy()

