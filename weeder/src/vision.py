#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from colorama import Fore, Back, Style, init
init(autoreset=True)

class Vision:
    
    def __init__(self):
        self.sub = rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect", Image, self.callback)
        self.difSub = rospy.Subscriber('/crop_difficulty', String, self.difCallback)
        self.difSwitch = 'Hard'
        self.pub = rospy.Publisher("/thorvald_001/kinect2_camera/hd/image_color_rect_filtered", Image, queue_size=1)
        self.bridge = CvBridge()

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
        weedmask=cv2.bitwise_not(weedmask) # invert as blob detector will look for black, ours is white
        # mask out crop
        min1 = np.array([040, 130, 000]) 
        max1 = np.array([141, 206, 254]) 
        cropmask = cv2.inRange(HSVimage, min1, max1)
        cropimage = cv2.bitwise_and(HSVimage, HSVimage, mask=cropmask) 
        weedimage = cv2.bitwise_and(HSVimage, HSVimage, mask=weedmask) 
        # image[np.all(image != (0, 0, 0), axis=-1)] = (179, 255, 255)
        # STOLEN FROM -> https://www.learnopencv.com/blob-detection-using-opencv-python-c/
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
        weedmask=cv2.bitwise_not(weedmask) # invert again to apply color 
        # STOLEN FROM -> https://answers.opencv.org/question/100731/i-want-to-change-the-black-pixels-in-the-image-to-red/
        indices = np.where(weedmask==255)
        image[indices[0], indices[1], :] = [0, 0, 255] # weed turns red
        indicesgreen = np.where(cropmask==255)
        image[indicesgreen[0], indicesgreen[1], :] = [0, 255, 0] # crop turns green
        im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        # cv2.imshow("filtered", im_with_keypoints)
        # cv2.waitKey(1)
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
        #  STOLEN FROM -> https://www.learnopencv.com/blob-detection-using-opencv-python-c/
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True 
        params.minArea = 100
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = -1
        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.87
        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = -1
        # Create a detector with the parameters
        detector = cv2.SimpleBlobDetector_create(params)
        # Detect blobs.
        mask2=cv2.bitwise_not(mask2)
        keypoints = detector.detect(mask2)
        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
        # the size of the circle corresponds to the size of blob        
        im_with_keypoints = cv2.drawKeypoints(img_without_crop, keypoints, np.array([]), (255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
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
        
        # STOLEN FROM -> https://stackoverflow.com/questions/42798659/how-to-remove-small-connected-objects-using-opencv
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
        
        mask2=cv2.bitwise_not(mask2)
        # STOLEN FROM -> https://www.learnopencv.com/blob-detection-using-opencv-python-c/
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = False # false yet still need to be here otherwise default set to True
        params.minArea = 100
        # Filter by Circularity
        params.filterByCircularity = False
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
        keypoints = detector.detect(mask2)
        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
        # the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
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
