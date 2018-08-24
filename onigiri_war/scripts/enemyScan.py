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





class EnemyScan():
    

    def __init__(self, bot_name):
        # bot name 
        self.name = bot_name        

        #
        self.enemy_angle=np.nan
        
        # publisher
        self.enemy_angle_pub = rospy.Publisher('enemy_angle', Float64 , queue_size=1)




    def strategy(self):
        r = rospy.Rate(1) # change speed 1fps

        while not rospy.is_shutdown():
            r.sleep()

    def scanCallback(self,data):
        flag=False
        temp=0
        k=0
        for intensity in (data.intensities):
#            if intensity > 1*10**27 and intensity < 3.5*10**27 :
            if ( (intensity) > 1*10**10 ) or intensity < 0:
                flag=True
                temp=k
                if k>180:
                    self.enemy_angle=k+1-360
                else:
                    self.enemy_angle=k-2
                break
            k=k+1
        if not(flag):
            self.enemy_angle=np.nan
        self.enemy_angle_pub.publish(self.enemy_angle)
        rospy.logout(str(temp) + " " + str(self.enemy_angle))


if __name__ == '__main__':
    rospy.init_node('enemy_scan')
    bot = EnemyScan('EnemyScan')

    rospy.Subscriber("scan",sensor_msgs.msg.LaserScan,bot.scanCallback)

    bot.strategy()
