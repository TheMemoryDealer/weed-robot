#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class Looker():
    
    def __init__(self):
        self.sub = rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect", Image, self.callback)
        self.difSub = rospy.Subscriber('/crop_difficulty', String, self.difcallback)
        self.difSwitch = ''
        self.pub = rospy.Publisher("/thorvald_001/kinect2_camera/hd/image_color_rect_filtered", Image, queue_size=1)
        self.bridge = CvBridge()

    def callback(self, msg): # callback should decide which crop bot is looking at
        try:
            ogimage = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as err:
            print(err) 
            
        if (self.difSwitch == 'Easy'):
            self.pub.publish(self.bridge.cv2_to_imgmsg(self.easyalgo(ogimage), 'bgr8'))
        elif (self.difSwitch == 'Medium'):
            self.pub.publish(self.bridge.cv2_to_imgmsg(self.mediumalgo(ogimage), 'bgr8'))
        elif (self.difSwitch == 'Hard'):
            self.pub.publish(self.bridge.cv2_to_imgmsg(self.hardalgo(ogimage), 'bgr8'))
        else:
            self.pub.publish(self.bridge.cv2_to_imgmsg(ogimage, 'bgr8'))
            
        
        
        
    def difcallback(self, data): # sets difficulty switch from subscribed data
        self.difSwitch = data.data
        
        
        
    def mediumalgo(self, image):
        # BLURogimage = cv2.GaussianBlur(image,(35,35),0)
        HSVimage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
        # mask out weed
        lowerGreen = np.array([033, 106, 165]) 
        upperGreen = np.array([126, 253, 255]) 
        mask = cv2.inRange(HSVimage, lowerGreen, upperGreen)
        mask = cv2.dilate(mask, np.ones((29, 29)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((39, 39)))
        # weedmask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernelSize)
        weedimage = cv2.bitwise_and(HSVimage, HSVimage, mask=mask) 
        weedimage = cv2.cvtColor(weedimage, cv2.COLOR_HSV2BGR)
        # cv2.imshow("filtered", weedimage)
        # cv2.waitKey(1)
        return weedimage
        
    def easyalgo(self, image):
        BLURogimage = cv2.GaussianBlur(image,(35,35),0)
        HSVimage = cv2.cvtColor(BLURogimage, cv2.COLOR_BGR2HSV) 
        # mask out weed
        lowerGreen = np.array([035, 000, 000]) 
        upperGreen = np.array([180, 129, 254]) 
        weedmask = cv2.inRange(HSVimage, lowerGreen, upperGreen)
        kernelSize = np.ones((8, 8))
        # weedmask = cv2.erode(weedmask, kernelSize)
        weedmask = cv2.morphologyEx(weedmask, cv2.MORPH_OPEN, kernelSize)
        weedmask=cv2.bitwise_not(weedmask)
        # mask out crop
        lowerGreen1 = np.array([040, 130, 000]) 
        upperGreen1 = np.array([141, 206, 254]) 
        cropmask = cv2.inRange(HSVimage, lowerGreen1, upperGreen1)
        cropimage = cv2.bitwise_and(HSVimage, HSVimage, mask=cropmask) 
        weedimage = cv2.bitwise_and(HSVimage, HSVimage, mask=weedmask) 
        # image[np.all(image != (0, 0, 0), axis=-1)] = (179, 255, 255)
        # https://www.learnopencv.com/blob-detection-using-opencv-python-c/
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = False # false yet still need to be here otherwise default set to True
        params.minArea = 100
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = -10
        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.87
        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.01
        # Create a detector with the parameters
        detector = cv2.SimpleBlobDetector_create(params)
        # Detect blobs.
        keypoints = detector.detect(weedmask)
        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
        # the size of the circle corresponds to the size of blob        
        weedmask=cv2.bitwise_not(weedmask)
        # https://answers.opencv.org/question/100731/i-want-to-change-the-black-pixels-in-the-image-to-red/
        indices = np.where(weedmask==255)
        image[indices[0], indices[1], :] = [0, 0, 255] # weed turns red
        indicesgreen = np.where(cropmask==255)
        image[indicesgreen[0], indicesgreen[1], :] = [0, 255, 0] # crop turns green
        im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        # cv2.imshow("filtered", im_with_keypoints)
        # cv2.waitKey(1)
        return im_with_keypoints

        
rospy.init_node("vis")
mover = Looker()


rospy.spin()

cv2.destroyAllWindows()