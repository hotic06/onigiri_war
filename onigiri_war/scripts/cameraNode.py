#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
#from abc import ABCMeta, abstractmethod
#from geometry_msgs.msg import Twist
#from ccr_msgs.msg import Bumper
from sensor_msgs.msg import Image
#from sensor_msgs.msg import LaserScan
#from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_msgs.msg import Float64


class CameraNode():
    def __init__(self):
        # for convert image topic to opencv obj
        self.img = None
        self.camera_preview = False
        self.mask_preview = False
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('camera/image_raw', Image, self.imageCallback)
        # publisher
        self.enemy_angle_camera_pub = rospy.Publisher('enemy_angle_camera', Float64 , queue_size=1)
        self.enemy_green_angle_camera_pub = rospy.Publisher('enemy_green_angle_camera', Float64 , queue_size=1)

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.camera_preview:
          cv2.imshow("Image window", self.img)
          cv2.waitKey(1)

    def woodChoice(self):
        # image size
        height , weight , col = self.img.shape

        # convert hsv color space
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

        # HSV空間で茶色の範囲を定義
        lower_wood = np.array([5,50,50])
        upper_wood = np.array([25,255,255])

        mask_wood = cv2.inRange(hsv, lower_wood, upper_wood)

        # wood 重心
        mom = cv2.moments(mask_wood)
        
        if self.mask_preview:
            cv2.imshow("wood masked view", mask_wood)
            cv2.waitKey(1)

        if "m00" in mom and "m10" in mom and "m01" in mom and mom["m00"]<>0:
            cx = int(mom['m10']/mom['m00'])
            cy = int(mom['m01']/mom['m00'])
            return -(cx - weight/2)*(32.0/176.0)
        else:
            return np.nan





    def blueMarkerChoice(self):
        # image size
        height , weight , col = self.img.shape

        # convert hsv color space
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

        # HSV空間で青色の範囲を定義
        lower_blue = np.array([110,50,50])
        upper_blue = np.array([130,255,255])

        # HSV イメージから青い物体だけを取り出すための閾値
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        if self.mask_preview:
            cv2.imshow("blue masked view", mask_blue)
            cv2.waitKey(1)


    def greenMarkerChoice(self):
        # image size
        height , weight , col = self.img.shape

        # convert hsv color space
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)


        # HSV空間で緑色の範囲を定義
        lower_green = np.array([50,120,120])
        upper_green = np.array([70,255,255])


        # HSV イメージから緑の物体だけを取り出すための閾値
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        
        # Greenノイズ除去
        kernel = np.ones((2,2),np.uint8)
        mask_green_denoise = cv2.erode(mask_green,kernel,iterations = 1)
        mask_green_denoise = cv2.dilate(mask_green_denoise,kernel,iterations = 1)

        # Green Blob
        labelnum, labelimg, contours, GoCs  = cv2.connectedComponentsWithStats(mask_green_denoise)

        # Green 一番大きい丸を選ぶ
        sort_idx =np.argsort(contours[:,4])[::-1]
        contours_sort=contours[sort_idx]
        GoCs_sort=GoCs[sort_idx]
        """
        print "label num ", labelnum 
        print "contours ", contours_sort
        print "Gravity of Centers ", GoCs_sort
        """

        img=mask_green_denoise

        """
        for label in xrange(1,labelnum):
            x,y = GoCs[label]
            img = cv2.circle(img, (int(x),int(y)), 1, (0,0,255), -1)    
            
            x,y,w,h,size = contours[label]
            img = cv2.rectangle(img, (x,y), (x+w,y+h), (255,255,0), 1)    
        """

        if labelnum > 1:
            x,y,w,h,size = contours_sort[1]
            img = cv2.rectangle(img, (x,y), (x+w,y+h), (255,255,0), 1)    
            x,y = GoCs_sort[1]

        if self.mask_preview:
            cv2.imshow("green masked view", img)
            cv2.waitKey(1)


        if labelnum > 1:
            return -(x - weight/2)*(32.0/176.0)
        else:
            return np.nan

    def strategy(self):
        r = rospy.Rate(5) # change speed 1fps
        while not rospy.is_shutdown():
            if self.img is None:
                continue
            angl=self.woodChoice()
            if not(np.isnan(angl)):
                self.enemy_angle_camera_pub.publish(angl)
                rospy.logout("cam_wood "+str(angl))
            else:
                self.enemy_angle_camera_pub.publish(np.nan)
                rospy.logout("cam_wood missing")
            angl_green=self.greenMarkerChoice()
            if not(np.isnan(angl_green)):
                self.enemy_green_angle_camera_pub.publish(angl_green)
                rospy.logout("cam_green "+str(angl_green))
            else:
                self.enemy_green_angle_camera_pub.publish(np.nan)
                rospy.logout("cam_green missing")
            r.sleep()



if __name__ == '__main__':
    rospy.init_node('camera_node')
    bot = CameraNode()
#    bot.camera_preview=True
    bot.mask_preview=True

    bot.strategy()

