import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class BlobColorTracker:

    def __init__(self):
        self.ns = rospy.get_namespace()
        self.point_blob_topic = self.ns + "blob/point_blob"
        self.blurred_blob_topic = self.ns + "blob/blurred_blob/compressed"
        self.eroded_blob_topic = self.ns + "blob/eroded_blob/compressed"
        self.dilated_blob_topic = self.ns + "blob/dilated_blob/compressed"
        # This publisher  uses Point message to publish
        # x,y: x,y relative poses of the center of the blob detected relative to the center of teh image
        # z: size of the blob detected
        self.pub_blob = rospy.Publisher(
            self.point_blob_topic, Point, queue_size=1)
        self.pub_blob_1 = rospy.Publisher(
            self.blurred_blob_topic, CompressedImage, queue_size=1)
        self.pub_blob_2 = rospy.Publisher(
            self.eroded_blob_topic, CompressedImage, queue_size=1)
        self.pub_blob_3 = rospy.Publisher(
            self.dilated_blob_topic, CompressedImage, queue_size=1)

    def blob_detect(self, image, lowerb, upperb, blur=0, sigma=0, kernel=(5, 5), color_space=cv2.COLOR_BGR2HSV, blob_params=None, search_window=None, publish_blob=False):
        """
        :param image: The frame
        :param lowerb: minimum threshold of the filter []
        :param upperb: maximum threshold of the filter []
        :param blur: blur value (default 0)
        :param sigma: sigma value (default 0)
        :param kernel: kernel value for erode & dilate operators
        :param blob_params: blob parameters (default None)
        :param search_window: window where to search as [x_min, y_min, x_max, y_max] adimensional (0.0 to 1.0) starting from top left corner
        :param publish_blob: if each stage should be published in a topic
        """
        # Create an UMat image for proccesing in the GPU
        uimg = cv2.UMat(image)

        # - Blur image to remove noise
        if blur > 0 and sigma > 0:
            ublurred = cv2.GaussianBlur(uimg, (blur, blur), sigma)
        else:
            ublurred = cv2.blur(uimg, (5, 5))

        if publish_blob:
            self.publish_to_topic(cv2.UMat.get(ublurred), self.pub_blob_1)

        # Erode image
        ueroded = cv2.erode(ublurred, kernel, iterations=2)

        if publish_blob:
            self.publish_to_topic(cv2.UMat.get(ueroded), self.pub_blob_2)

        # Dilate the image
        udilatated = cv2.dilate(ueroded, kernel, iterations=2)

        if publish_blob:
            self.publish_to_topic(cv2.UMat.get(udilatated), self.pub_blob_3)

        # - Convert image
        uimgconv = cv2.cvtColor(udilatated, color_space)

        # - Apply HSV threshold
        umask = cv2.inRange(uimgconv, lowerb, upperb)

        # converts back image for using cpu only
        mask = cv2.UMat.get(umask)

        # - Search window
        if search_window is None:
            search_window = [0.1, 0.1, 0.9, 0.9]

        # - Cut the image using the search mask
        mask = self.apply_search_window(mask, search_window)

        # - build default blob detection parameters, if none have been provided
        if blob_params is None:
            params = self.get_default_blob_params()
        else:
            params = blob_params

        # - Apply blob detection
        detector = cv2.SimpleBlobDetector_create(params)

        keypoints = detector.detect(mask)

        return keypoints, mask

    def get_default_blob_params(self):
        # Set up the SimpleBlobdetector with default parameters.
        params = cv2.SimpleBlobDetector_Params()

        # Detect light blobs, don't need to reverse the mask
        params.blobColor = 255

        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 100

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 45
        params.maxArea = 20000

        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.1

        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.87

        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.125

        return params

    def apply_search_window(self, image, window_adim=[0.1, 0.1, 0.9, 0.9]):
        """
        Apply search window
        :param image: standar opencv frame
        :param window_adim: box for searching
        :return:
        """
        rows = image.shape[0]
        cols = image.shape[1]
        x_min_px = int(cols * window_adim[0])
        y_min_px = int(rows * window_adim[1])
        x_max_px = int(cols * window_adim[2])
        y_max_px = int(rows * window_adim[3])

        # --- Initialize the mask as a black image
        mask = np.zeros(image.shape, np.uint8)

        # --- Copy the pixels from the original image corresponding to the window
        mask[y_min_px:y_max_px,
             x_min_px:x_max_px] = image[y_min_px:y_max_px, x_min_px:x_max_px]

        # --- return the mask
        return mask

    def publish_to_topic(self, image, pub):
        try:
            pub.publish(bridge.cv2_to_compressed_imgmsg(
                image, dst_format='jpeg'))
        except CvBridgeError as e:
            rospy.logerror("%s", e)
