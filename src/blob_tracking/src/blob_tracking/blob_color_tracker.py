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
        # This publisher  uses Point message to publish
        # x,y: x,y relative poses of the center of the blob detected relative to the center of teh image
        # z: size of the blob detected
        self.pub_blob = rospy.Publisher(
            self.point_blob_topic, Point, queue_size=1)

    def blob_detect(self, image, lowerb, upperb, blur=0, sigma=0, kernel=(5, 5), color_space=cv2.COLOR_BGR2HSV, blob_params=None, search_window=None, show_image=False):
        """
        :param image: The frame
        :param lowerb: minimum threshold of the filter []
        :param upperb: maximum threshold of the filter []
        :param blur: blur value (default 0)
        :param sigma: sigma value (default 0)
        :param kernel: kernel value for erode & dilate operators
        :param blob_params: blob parameters (default None)
        :param search_window: window where to search as [x_min, y_min, x_max, y_max] adimensional (0.0 to 1.0) starting from top left corner
        :param show_image: if each stage should be published in a topic
        """
        # Create an UMat image for proccesing in the GPU
        uimg = cv2.UMat(image)

        # - Blur image to remove noise
        if blur > 0 and sigma > 0:
            ublurred = cv2.GaussianBlur(uimg, (blur, blur), sigma)
        else:
            ublurred = cv2.blur(uimg, (5, 5))

        if show_image:
            cv2.imshow('Blob Blurred', ublurred)

        # Erode image
        ueroded = cv2.erode(ublurred, kernel, iterations=2)

        if show_image:
           cv2.imshow('Blob Erode', ueroded)

        # Dilate the image
        udilatated = cv2.dilate(ueroded, kernel, iterations=2)

        if show_image:
            cv2.imshow('Blob Dilated', udilatated)

        # - Convert image
        uimgconv = cv2.cvtColor(udilatated, color_space)

        # - Apply HSV threshold
        umask = cv2.inRange(uimgconv, lowerb, upperb)

        # converts back image for using cpu only
        mask = cv2.UMat.get(umask)

        # - Search window
        if search_window is None:
            search_window = [0.1, 0.1, 0.9, 0.9]

        if show_image:
            cv2.imshow('Blob Mask', mask)

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

    def draw_frame(self, image, dimension=0.3, line=2):
        """
        Draw X Y frame
        :param image:
        :param dimension:
        :param line:
        :return: image
        """

        rows = image.shape[0]
        cols = image.shape[1]
        size = min([rows, cols])
        center_x = int(cols / 2)
        center_y = int(rows / 2)

        line_length = int(size)

        # -- X
        image = cv2.line(image, (center_x, center_y),
                         (center_x + line_length, center_y), (0, 0, 255), line)
        # -- Y
        image = cv2.line(image, (center_x, center_y),
                         (center_x, center_y + line_length), (0, 255, 0), line)

        return image