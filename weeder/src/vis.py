#! /usr/bin/env python
import rospy
import sys
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


class Looker():
    
    def __init__(self):
        self.camSub = rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect", Image, self.camCallback)
        self.camPub = rospy.Publisher("/thorvald_001/kinect2_camera/hd/image_color_rect_filtered", Image, queue_size=0)
        self.bridge = CvBridge() # Bridge used for converting msgs to cv2 image class

    def camCallback(self, msg):
        # format reply
        reply = Image()
        reply.header = msg.header

        ogimage = self.bridge.imgmsg_to_cv2(msg, "bgr8") 
        HSVimage = cv2.cvtColor(ogimage, cv2.COLOR_BGR2HSV) 

        # mask out weed
        lowerGreen = np.array([035, 000, 000]) 
        upperGreen = np.array([180, 129, 254]) 
        weedmask = cv2.inRange(HSVimage, lowerGreen, upperGreen)
        kernelSize = np.ones((8, 8))
        weedmask = cv2.morphologyEx(weedmask, cv2.MORPH_OPEN, kernelSize)

        # mask out crop
        lowerGreen1 = np.array([040, 130, 000]) 
        upperGreen1 = np.array([141, 206, 254]) 
        cropmask = cv2.inRange(HSVimage, lowerGreen1, upperGreen1)

        cropimage = cv2.bitwise_and(HSVimage, HSVimage, mask=cropmask) 
        weedimage = cv2.bitwise_and(HSVimage, HSVimage, mask=weedmask) 
        # image[np.all(image != (0, 0, 0), axis=-1)] = (179, 255, 255)
        
        
        image = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)
        
        
        # https://answers.opencv.org/question/100731/i-want-to-change-the-black-pixels-in-the-image-to-red/
        indices = np.where(weedmask==255)
        ogimage[indices[0], indices[1], :] = [0, 0, 255]
        
        indices1 = np.where(cropmask==255)
        ogimage[indices1[0], indices1[1], :] = [0, 255, 0]


        self.camPub.publish(self.bridge.cv2_to_imgmsg(ogimage, 'bgr8')) # Convert image back to bgr8 image msg, then publish

        
rospy.init_node("looker_")
mover = Looker()

print("Loaded!")

rospy.spin()

cv2.destroyAllWindows()