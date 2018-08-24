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
import numpy as np


class CollisionNode():
    def __init__(self, bot_name):
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.collision_pub = rospy.Publisher('collision', Float64 ,queue_size=1)

    def strategy(self):
        r = rospy.Rate(1) # change speed 1fps
        while not rospy.is_shutdown():
            r.sleep()

    def scanCallback(self,data):
        ang=np.nan
        min_rang=99999
        k=0
        for rang in data.ranges:
            hanni=50
            if k<hanni or k>=360-hanni:
                if rang < 0.275 and rang < min_rang:
                    ang=k
                    min_rang=rang
                    if ang>180:
                        ang=ang+1-360
                    else:
                        ang=ang-2

            k=k+1
        
        self.collision_pub.publish(ang)


if __name__ == '__main__':
    rospy.init_node('collisionNode')
    bot = CollisionNode('collisionNode')

    rospy.Subscriber("scan",sensor_msgs.msg.LaserScan,bot.scanCallback,queue_size = 1)

    bot.strategy()

