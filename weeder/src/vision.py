#! /usr/bin/env python
import rospy, image_geometry, cv2, sys, tf
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, CameraInfo, PointCloud
from geometry_msgs.msg import PoseStamped, Point32
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Empty
import numpy as np
from colorama import Fore, Back, Style, init # this allows for color print to console
init(autoreset=True) # dont save color to memory
import itertools


class Vision:
    
    def __init__(self):
        self.sub = rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect", Image, self.callback) # subscribe to get camera image
        self.difSub = rospy.Subscriber('/thorvald_001/crop_difficulty', String, self.difCallback) # subscribe to crop type from Navigation ndoe
        self.difSwitch = '' # switch to store dificulty
        self.pub = rospy.Publisher("/thorvald_001/kinect2_camera/hd/image_color_rect_filtered", Image, queue_size=1) # publish segmented image for visualization
        self.bridge = CvBridge() # to connect OpenCV with ROS
        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_camera/hd/camera_info', CameraInfo, self.cameraInfoCallback) # camera info for iamge to world projection
        self.camera_model = None # we'll use this later for img geometry
        self.tf_listener = tf.TransformListener() # create listener for transforms
        self.pointcloud_publisher = rospy.Publisher("/thorvald_001/weed_PCL", PointCloud, queue_size=10, latch='true') # PCL to publish sprayed weeds
        self.spray_method = rospy.ServiceProxy('/thorvald_001/spray', Empty) # get Marc's sprayer

    def callback(self, msg): # callback should decide which crop bot is looking at
        try:
            ogimage = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as err:
            print(err)
            
        if (self.difSwitch == 'Easy'): # 1st crop
            print('Looking at: '+ Fore.GREEN + 'Young lettuce')
            imgTopub, keypoints = self.easyAlgo(ogimage) # apply vision algo
            self.pub.publish(self.bridge.cv2_to_imgmsg(imgTopub, 'bgr8')) # publish img results
            self.processPoints(keypoints) # convert to world and send PCL
            
        elif (self.difSwitch == 'Medium'): # 2nd crop
            print('Looking at: '+ Fore.YELLOW + 'Lettuce')
            imgTopub, keypoints = self.mediumAlgo(ogimage) # apply vision algo
            self.pub.publish(self.bridge.cv2_to_imgmsg(imgTopub, 'bgr8')) # publish img results
            self.processPoints(keypoints) # convert to world and send PCL
            
        elif (self.difSwitch == 'Hard'): # 3rd crop
            print('Looking at: '+ Fore.RED + 'Onion')
            imgTopub, keypoints = self.hardAlgo(ogimage) # apply vision algo
            self.pub.publish(self.bridge.cv2_to_imgmsg(imgTopub, 'bgr8')) # publish img results
            self.processPoints(keypoints) # convert to world and send PCL
            
        elif (self.difSwitch == 'N/A'):# End of rows
            print(Fore.CYAN + 'NO CROP VISIBLE') 
            imgTopub = self.removeGround(ogimage) # just remove ground
            self.pub.publish(self.bridge.cv2_to_imgmsg(imgTopub, 'bgr8')) # publish img results
            
        else:
            self.pub.publish(self.bridge.cv2_to_imgmsg(ogimage, 'bgr8')) # publish og image
            
            
    def difCallback(self, data): # sets difficulty switch from subscribed data
        self.difSwitch = data.data
        
        
    def removeGround(self, image):
        HSVimage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # convert color space
        # between values for thresholding
        min = np.array([43, 000, 000]) 
        max = np.array([180, 253, 255]) 
        mask = cv2.inRange(HSVimage, min, max) # threshold
        cropimage = cv2.bitwise_and(HSVimage, HSVimage, mask=mask) # obtain threshold result
        image = cv2.cvtColor(cropimage, cv2.COLOR_HSV2BGR) # reconvert color space for publishing
        return image
    
    def detectWeeds(self, image, mask, cropdif):
        weedmask=cv2.bitwise_not(mask) # invert as blob detector will look for black pixels, ours is white
        #  FROM -> https://www.learnopencv.com/blob-detection-using-opencv-python-c/
        params = cv2.SimpleBlobDetector_Params() # initialize detection parameters
        # decide which area looking at
        if cropdif == 'hard': 
            params.filterByArea = True 
            params.minArea = 7000
        elif cropdif == 'easy':
            params.filterByArea = True 
            params.minArea = 5000
        elif cropdif == 'medium':
            params.filterByArea = True
            params.minArea = 500
        else:
            params.filterByArea = False
        params.maxArea = sys.maxint
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = -10
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0
        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0
        # Create a detector with the parameters
        detector = cv2.SimpleBlobDetector_create(params)
        # Detect blobs.
        keypoints = detector.detect(weedmask)
        # Draw detected blobs as red circles. cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        # if(len(keypoints) != 0): # super sophisticated spray method!!!
        #     self.spray_method()
        return im_with_keypoints, keypoints
    
    def processPoints(self, keypoints):
        for keyPoint in keypoints: #  FROM -> https://stackoverflow.com/a/37332802
            x = int(keyPoint.pt[0])
            y = int(keyPoint.pt[1])
            d = int(keyPoint.size) # diameter not area, might be useful later
            realpoints = self.pxToWorldFrame(int(x), int(y)) # apply point transform to world coordiantes
            self.pointToPointcloud(realpoints.pose.position.x, realpoints.pose.position.y, realpoints.pose.position.z) # create PCL

            
    def pointToPointcloud(self, x, y, z): #  FROM -> https://answers.ros.org/question/207071/how-to-fill-up-a-pointcloud-message-with-data-in-python/?answer=218951#post-id-218951
            my_awesome_pointcloud = PointCloud() # declaring pointcloud
            #filling pointcloud header
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'map'
            my_awesome_pointcloud.header = header # assign header to pointcloud
            my_awesome_pointcloud.points.append(Point32(x, y, z)) #filling some points
            self.pointcloud_publisher.publish(my_awesome_pointcloud) # publish
    
    def pxToWorldFrame(self, pxX, pxY):
        #  FROM _> https://answers.ros.org/question/241624/converting-pixel-location-in-camera-frame-to-world-frame/?answer=242060#post-id-242060 + Greg's lab on img geometry
        cam_model_point = self.camera_model.projectPixelTo3dRay(self.camera_model.rectifyPoint((pxX, pxY))) # project a rectified pixel to a 3d ray. 
        bot_pose = PoseStamped()
        # fill pose stamp with data
        bot_pose.pose.position.x = cam_model_point[0] * 0.5 # normalize by +- focal length
        bot_pose.pose.position.y = cam_model_point[1] * 0.5
        bot_pose.pose.position.z = 0.5
        bot_pose.pose.orientation.x = 0
        bot_pose.pose.orientation.y = 0
        bot_pose.pose.orientation.z = 0
        bot_pose.pose.orientation.w = 1.0
        bot_pose.header.stamp = rospy.Time.now()
        bot_pose.header.frame_id = self.camera_model.tfFrame()
        self.tf_listener.waitForTransform(self.camera_model.tfFrame(), '/map', rospy.Time.now(), rospy.Duration(1.0)) # block until a transform is possible or times out.
        tf_point = self.tf_listener.transformPose('/map', bot_pose) # get transform when/if available
        return tf_point
    
    def cameraInfoCallback(self, data): # func gets camera info. Copied straight off Greg's labs
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() #Only subscribe once
        
    def easyAlgo(self, image):
        imagenoground = self.removeGround(image) # remove ground on start
        BLURogimage = cv2.GaussianBlur(imagenoground,(35,35),0) # blur to remove details
        HSVimage = cv2.cvtColor(BLURogimage, cv2.COLOR_BGR2HSV)   # convert color space for thresholding
        # mask out weed 
        min = np.array([035, 000, 000]) 
        max = np.array([180, 129, 254]) 
        weedmask = cv2.inRange(HSVimage, min, max) # threshold
        weedmask = cv2.morphologyEx(weedmask, cv2.MORPH_OPEN, np.ones((8, 8))) # remove small white dots
        # mask out crop
        min1 = np.array([040, 130, 000]) 
        max1 = np.array([141, 206, 254]) 
        cropmask = cv2.inRange(HSVimage, min1, max1) # threshold
        # obtain threshold result
        cropimage = cv2.bitwise_and(HSVimage, HSVimage, mask=cropmask) 
        weedimage = cv2.bitwise_and(HSVimage, HSVimage, mask=weedmask) 
        # image[np.all(image != (0, 0, 0), axis=-1)] = (255, 255, 255) # turn all non black pixels white
        #  FROM -> https://stackoverflow.com/a/60019059
        indices = np.where(weedmask==255) # find pixels
        image[indices[0], indices[1], :] = [0, 0, 255] # weed turns red for those pixels
        indicesgreen = np.where(cropmask==255) # find pixels
        image[indicesgreen[0], indicesgreen[1], :] = [0, 255, 0] # crop turns green for those pixels
        
        im_with_keypoints, keypoints = self.detectWeeds(image, weedmask, 'easy') # continue vision pipeline
        return im_with_keypoints, keypoints
        
    def mediumAlgo(self, image):
        imagenoground = self.removeGround(image) # remove ground on start
        HSVimage = cv2.cvtColor(imagenoground, cv2.COLOR_BGR2HSV) 
        # mask out crop
        min = np.array([033, 106, 165]) 
        max = np.array([126, 253, 255]) 
        mask = cv2.inRange(HSVimage, min, max) # threshold
        mask = cv2.dilate(mask, np.ones((50, 50)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((50, 50))) # smoothen out mask
        mask = cv2.dilate(mask, np.ones((100, 100))) # expand mask
        cropimage = cv2.bitwise_and(HSVimage, HSVimage, mask=mask) # obtain threshold results
        cropimage = cv2.cvtColor(cropimage, cv2.COLOR_HSV2BGR) # reconvert colorspace for presentation
        img_without_crop = cv2.subtract(image, cropimage) # remove crops
        indices = np.where(mask==255) # find pixels
        img_without_crop[indices[0], indices[1], :] = [0, 255, 0] # crop turns green
        
        # now mask out whatever is left (weeds)
        minWeed = np.array([031, 037, 000]) 
        maxWeed = np.array([180, 255, 250])
        HSVweedimage = cv2.cvtColor(img_without_crop, cv2.COLOR_BGR2HSV) # convert color space for thresholding
        mask2 = cv2.inRange(HSVweedimage, minWeed, maxWeed) # threshold
        mask2 = cv2.erode(mask2, np.ones((4, 4))) # make smaller
        mask2 = cv2.morphologyEx(mask2, cv2.MORPH_CLOSE, np.ones((7, 7))) # closing small holes
        weedimage = cv2.bitwise_and(HSVweedimage, HSVweedimage, mask=mask2) # obtain threshold results
        weedimage = cv2.cvtColor(weedimage, cv2.COLOR_HSV2BGR) # reconvert for presentation (not used)
        indicesred = np.where(mask2==255) # find pixels
        img_without_crop[indicesred[0], indicesred[1], :] = [0, 0, 255] # weed turns red for those pixels
        
        im_with_keypoints = self.detectWeeds(img_without_crop, mask2, 'medium') # continue vision pipeline
        return im_with_keypoints
        
    def hardAlgo(self, image):
        imagenoground = self.removeGround(image) # remove ground on start
        HSVimage = cv2.cvtColor(imagenoground, cv2.COLOR_BGR2HSV)  # convert color space for thresholding
        # between values for thresholding
        min = np.array([78, 000, 000]) 
        max = np.array([170, 255, 255])
        mask = cv2.inRange(HSVimage, min, max) # threshold
        mask = cv2.dilate(mask, np.ones((19, 19))) # expand mask
        #  FROM -> https://stackoverflow.com/a/42812226
        dummy_image=mask.astype(np.uint8) # reconvert to uint8
        #find all your connected components (white blobs in your image)
        nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(dummy_image, connectivity=8)
        #connectedComponentswithStats yields every seperated component with information on each of them, such as size
        #the following part is just taking out the background which is also considered a component, but most of the time we don't want that.
        sizes = stats[1:, -1]; nb_components = nb_components - 1
        # minimum size of particles we want to keep (number of pixels)
        #here, it's a fixed value, but you can set it as you want, eg the mean of the sizes or whatever
        min_size = 4000  
        #answer image
        mask_updated = np.zeros((output.shape))
        #for every component in the image, you keep it only if it's above min_size
        for i in range(0, nb_components):
            if sizes[i] >= min_size:
                mask_updated[output == i + 1] = 255

        mask_updated=mask_updated.astype(np.uint8) # reconvert to uint8
        
        cropimage = cv2.bitwise_and(HSVimage, HSVimage, mask=mask_updated) # obtain threshold results
        cropimage = cv2.cvtColor(cropimage, cv2.COLOR_HSV2BGR) # reconvert for presentation
        img_without_crop = cv2.subtract(image, cropimage) # remove crops
        indices = np.where(mask_updated==255) # find pixels
        image[indices[0], indices[1], :] = [0, 255, 0] # crop turns green
        
        # now mask out whatever is left (weeds)
        minWeed = np.array([026, 002, 000]) 
        maxWeed = np.array([80, 255, 250])
        HSVweedimage = cv2.cvtColor(img_without_crop, cv2.COLOR_BGR2HSV) # convert color space for thresholding
        mask2 = cv2.inRange(HSVweedimage, minWeed, maxWeed) # threshold
        mask2 = cv2.erode(mask2, np.ones((8, 8))) # make mask smaller
        mask2 = cv2.morphologyEx(mask2, cv2.MORPH_OPEN, np.ones((10, 10))) # remove noise
        weedimage = cv2.bitwise_and(HSVweedimage, HSVweedimage, mask=mask2) # obtain threshold
        weedimage = cv2.cvtColor(weedimage, cv2.COLOR_HSV2BGR) # reconvert for presentation
        indicesred = np.where(mask2==255) # fnd pixels
        image[indicesred[0], indicesred[1], :] = [0, 0, 255] # weed turns red for those pixels
        
        im_with_keypoints = self.detectWeeds(image, mask2, 'hard') # continue vision pipeline
        return im_with_keypoints
        


if __name__ == '__main__':
    rospy.init_node("vision") # initialize this class as ROS node
    mover = Vision() # initialize the actual class
    print('-----Vision started-----')
    try:
        rospy.spin() # run vision until stopped 
    except KeyboardInterrupt:
        print "Shutting down"
    print('-----Vision exiting-----')
    cv2.destroyAllWindows() # destroy all opencv windows when killing node