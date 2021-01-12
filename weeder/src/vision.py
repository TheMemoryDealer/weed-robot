#! /usr/bin/env python
import rospy, image_geometry, cv2, sys, tf
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, CameraInfo, PointCloud
from geometry_msgs.msg import PoseStamped, Point32
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Empty
import numpy as np
from colorama import Fore, Back, Style, init # this allows for color print to console
init(autoreset=True) # dont remember the color
import itertools


class Vision:
    camera_model = None
    
    def __init__(self):
        self.sub = rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect", Image, self.callback)
        self.difSub = rospy.Subscriber('/crop_difficulty', String, self.difCallback)
        self.difSwitch = 'Easy'
        self.pub = rospy.Publisher("/thorvald_001/kinect2_camera/hd/image_color_rect_filtered", Image, queue_size=1)
        self.bridge = CvBridge()
        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_camera/hd/camera_info', CameraInfo, self.cameraInfoCallback)
        self.tf_listener = tf.TransformListener()
        self.pointcloud_publisher = rospy.Publisher("/my_pointcloud_topic", PointCloud, queue_size=1)
        self.spray_method = rospy.ServiceProxy('/thorvald_001/spray', Empty)

    def callback(self, msg): # callback should decide which crop bot is looking at
        try:
            ogimage = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as err:
            print(err)
            
        if (self.difSwitch == 'Easy'):
            print('Looking at: '+ Fore.GREEN + 'Young lettuce')
            self.pub.publish(self.bridge.cv2_to_imgmsg(self.easyAlgo(ogimage), 'bgr8'))
        elif (self.difSwitch == 'Medium'):
            print('Looking at: '+ Fore.YELLOW + 'Lettuce')
            self.pub.publish(self.bridge.cv2_to_imgmsg(self.mediumAlgo(ogimage), 'bgr8'))
        elif (self.difSwitch == 'Hard'):
            print('Looking at: '+ Fore.RED + 'Onion')
            self.pub.publish(self.bridge.cv2_to_imgmsg(self.hardAlgo(ogimage), 'bgr8'))
        else:
            print(Fore.CYAN + 'NO CROP VISIBLE')
            self.pub.publish(self.bridge.cv2_to_imgmsg(self.removeGround(ogimage), 'bgr8'))
            
            
    def difCallback(self, data): # sets difficulty switch from subscribed data
        self.difSwitch = data.data
        
        
    def removeGround(self, image):
        HSVimage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        min = np.array([43, 000, 000]) 
        max = np.array([180, 253, 255])
        mask = cv2.inRange(HSVimage, min, max)
        # mask = cv2.dilate(mask, np.ones((19, 19)))
        cropimage = cv2.bitwise_and(HSVimage, HSVimage, mask=mask)
        image = cv2.cvtColor(cropimage, cv2.COLOR_HSV2BGR)
        return image
    
    def detectWeeds(self, image, mask):
        weedmask=cv2.bitwise_not(mask) # invert as blob detector will look for black pixels, ours is white
        # STOLEN FROM -> https://www.learnopencv.com/blob-detection-using-opencv-python-c/
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True 
        params.minArea = 250
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
        # pointarray = []
        if(len(keypoints) != 0):
            self.spray_method()

        for keyPoint in keypoints: # STOLEN FROM -> https://stackoverflow.com/a/37332802
            x = int(keyPoint.pt[0])
            y = int(keyPoint.pt[1])
            d = int(keyPoint.size) # diameter not area, might be useful later
            '''
            Ok this is where we need to stop. If we keep calling local funcs 
            from here it'll affect the FPS on the segmented image, that's no good.
            We need to get whatever data we have out via publishing and have another 
            node deal with it elsewhere.
            '''
            print([x,y])
            #declaring pointcloud
            my_awesome_pointcloud = PointCloud() # STOLEN FROM -> https://answers.ros.org/question/207071/how-to-fill-up-a-pointcloud-message-with-data-in-python/?answer=218951#post-id-218951
            #filling pointcloud header
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = str(b)
            my_awesome_pointcloud.header = header
            #filling some points
            my_awesome_pointcloud.points.append(Point32(x, y, d))
            #publish
            self.pointcloud_publisher.publish(my_awesome_pointcloud)
            # self.pixelPoints.append([int(x),int(y)])
            # self.pixelPoints.sort()
            # list(self.pixelPoints for self.pixelPoints,_ in itertools.groupby(self.pixelPoints))
            # a = self.pxToWorldFrame(int(x), int(y))
            # print(a)
            # print(a.pose.position.x)
            # print(a.pose.position.y)
            # print(a.pose.position.z)
            # pointarray.append([x,y])
            # print(s)
        # print(pointarray)
        return im_with_keypoints
    
    def pxToWorldFrame(self, pxX, pxY):
        # STOLEN FROM _> https://answers.ros.org/question/241624/converting-pixel-location-in-camera-frame-to-world-frame/?answer=242060#post-id-242060 + Greg's lab on img geometry
        cam_model_point = self.camera_model.projectPixelTo3dRay(self.camera_model.rectifyPoint((pxX, pxY)))
        bot_pose = PoseStamped()
        bot_pose.pose.position.x = cam_model_point[0]
        bot_pose.pose.position.y = cam_model_point[1]
        bot_pose.pose.position.z = cam_model_point[2]
        bot_pose.pose.orientation.x = 0
        bot_pose.pose.orientation.y = 0
        bot_pose.pose.orientation.z = 0
        bot_pose.pose.orientation.w = 1.0
        bot_pose.header.stamp = rospy.Time.now()
        bot_pose.header.frame_id = self.camera_model.tfFrame()
        self.tf_listener.waitForTransform(self.camera_model.tfFrame(), '/map', rospy.Time.now(), rospy.Duration(1.0))
        tf_point = self.tf_listener.transformPose('/map', bot_pose)
        return tf_point
    
    def cameraInfoCallback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() #Only subscribe once
        
    def easyAlgo(self, image):
        imagenoground = self.removeGround(image)
        BLURogimage = cv2.GaussianBlur(imagenoground,(35,35),0)
        HSVimage = cv2.cvtColor(BLURogimage, cv2.COLOR_BGR2HSV) 
        # mask out weed
        min = np.array([035, 000, 000]) 
        max = np.array([180, 129, 254]) 
        weedmask = cv2.inRange(HSVimage, min, max)
        # weedmask = cv2.erode(weedmask, kernelSize)
        weedmask = cv2.morphologyEx(weedmask, cv2.MORPH_OPEN, np.ones((8, 8)))
        # mask out crop
        min1 = np.array([040, 130, 000]) 
        max1 = np.array([141, 206, 254]) 
        cropmask = cv2.inRange(HSVimage, min1, max1)
        cropimage = cv2.bitwise_and(HSVimage, HSVimage, mask=cropmask) 
        weedimage = cv2.bitwise_and(HSVimage, HSVimage, mask=weedmask) 
        # image[np.all(image != (0, 0, 0), axis=-1)] = (255, 255, 255) # turn all non black pixels white
        # STOLEN FROM -> https://stackoverflow.com/a/60019059
        indices = np.where(weedmask==255)
        image[indices[0], indices[1], :] = [0, 0, 255] # weed turns red
        indicesgreen = np.where(cropmask==255)
        image[indicesgreen[0], indicesgreen[1], :] = [0, 255, 0] # crop turns green
        # cv2.imshow("filtered", im_with_keypoints)
        # cv2.waitKey(1)
        im_with_keypoints = self.detectWeeds(image, weedmask)
        return im_with_keypoints
        
    def mediumAlgo(self, image):
        imagenoground = self.removeGround(image)
        # BLURogimage = cv2.GaussianBlur(image,(35,35),0)
        HSVimage = cv2.cvtColor(imagenoground, cv2.COLOR_BGR2HSV) 
        # mask out crop
        min = np.array([033, 106, 165]) 
        max = np.array([126, 253, 255]) 
        mask = cv2.inRange(HSVimage, min, max)
        mask = cv2.dilate(mask, np.ones((50, 50)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((39, 39)))
        mask = cv2.dilate(mask, np.ones((50, 50)))
        cropimage = cv2.bitwise_and(HSVimage, HSVimage, mask=mask) 
        cropimage = cv2.cvtColor(cropimage, cv2.COLOR_HSV2BGR)
        img_without_crop = cv2.subtract(image, cropimage) # remove crops
        indices = np.where(mask==255)
        img_without_crop[indices[0], indices[1], :] = [0, 255, 0] # crop turns green
        
        # now mask out whatever is left (weeds)
        minWeed = np.array([031, 037, 000]) 
        maxWeed = np.array([180, 255, 250])
        HSVweedimage = cv2.cvtColor(img_without_crop, cv2.COLOR_BGR2HSV)
        mask2 = cv2.inRange(HSVweedimage, minWeed, maxWeed)
        mask2 = cv2.erode(mask2, np.ones((4, 4)))
        mask2 = cv2.morphologyEx(mask2, cv2.MORPH_CLOSE, np.ones((7, 7)))
        weedimage = cv2.bitwise_and(HSVweedimage, HSVweedimage, mask=mask2)
        weedimage = cv2.cvtColor(weedimage, cv2.COLOR_HSV2BGR)
        indicesred = np.where(mask2==255)
        img_without_crop[indicesred[0], indicesred[1], :] = [0, 0, 255]
        im_with_keypoints = self.detectWeeds(img_without_crop, mask2)
        return im_with_keypoints
        
    def hardAlgo(self, image):
        # STOLEN FROM -> https://www.tutorialspoint.com/line-detection-in-python-with-opencv
        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # edges = cv2.Canny(gray, 75, 150)
        # lines = cv2.HoughLinesP(edges, 1, np.pi/10, 30, maxLineGap=250)
        # for line in lines:
        #     x1, y1, x2, y2 = line[0]
        #     # cv2.line(image, (x1, y1), (x2, y2), (0, 0, 200), 1)
        # # cv2.imshow("linesEdges", edges)
        # # cv2.imshow("linesDetected", image)
        # edges = cv2.dilate(edges, np.ones((6, 6)))
        # cropimage = cv2.bitwise_and(image, image, mask=edges)
        # img_without_crop = cv2.subtract(image, cropimage)
        # BLURogimage = cv2.GaussianBlur(image,(35,35),0)
        imagenoground = self.removeGround(image)
        HSVimage = cv2.cvtColor(imagenoground, cv2.COLOR_BGR2HSV) 
        min = np.array([78, 000, 000]) 
        max = np.array([170, 255, 255])
        mask = cv2.inRange(HSVimage, min, max)
        mask = cv2.dilate(mask, np.ones((19, 19)))
        
        # STOLEN FROM -> https://stackoverflow.com/a/42812226
        dummy_image=mask.astype(np.uint8)
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

        mask_updated=mask_updated.astype(np.uint8)
        
        cropimage = cv2.bitwise_and(HSVimage, HSVimage, mask=mask_updated)
        cropimage = cv2.cvtColor(cropimage, cv2.COLOR_HSV2BGR)
        img_without_crop = cv2.subtract(image, cropimage) # remove crops
        indices = np.where(mask_updated==255)
        image[indices[0], indices[1], :] = [0, 255, 0] # crop turns green
        
        # now mask out whatever is left (weeds)
        minWeed = np.array([026, 002, 000]) 
        maxWeed = np.array([80, 255, 250])
        HSVweedimage = cv2.cvtColor(img_without_crop, cv2.COLOR_BGR2HSV)
        mask2 = cv2.inRange(HSVweedimage, minWeed, maxWeed)
        mask2 = cv2.erode(mask2, np.ones((6, 6)))
        mask2 = cv2.morphologyEx(mask2, cv2.MORPH_OPEN, np.ones((10, 10)))
        weedimage = cv2.bitwise_and(HSVweedimage, HSVweedimage, mask=mask2)
        weedimage = cv2.cvtColor(weedimage, cv2.COLOR_HSV2BGR)
        indicesred = np.where(mask2==255)
        image[indicesred[0], indicesred[1], :] = [0, 0, 255]
        
        im_with_keypoints = self.detectWeeds(image, mask2)
        return im_with_keypoints
        


if __name__ == '__main__':
    rospy.init_node("vision")
    mover = Vision()
    print('-----Vision started-----')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    print('-----Vision exiting-----')
    cv2.destroyAllWindows()