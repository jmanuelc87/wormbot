#! /usr/bin/env python
import rospy
from blob_tracking import BlobTracker, detect_publish
from blob_tracking.cfg import HSVLimitsConfig
from camera import CameraSensor
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image, CompressedImage

# HSV limits default values
hsv_min = [0, 255, 255]
hsv_max = [23, 255, 255]

# We define the detection area [x_min, y_min, x_max, y_max] adimensional (0.0 to 1.0) starting from top left corner
window = [0.05, 0.05, 0.95, 0.95]

ns = rospy.get_namespace()


def reconfigure_callback(config, level):
    rospy.loginfo("Reconfigure Request")
    # Assign configuration values
    hsv_max[0] = config.h_max
    hsv_max[1] = config.s_max
    hsv_max[2] = config.v_max
    hsv_min[0] = config.h_min
    hsv_min[1] = config.s_min
    hsv_min[2] = config.v_min
    rospy.loginfo("hsv_max: %s, hsv_min: %s", hsv_max, hsv_min)
    return config


bridge = CvBridge()

pub_image_raw = rospy.Publisher(ns + "camera/keypoints/raw", Image, queue_size=1)
pub_image_compressed = rospy.Publisher(ns + "camera/keypoints/compressed", CompressedImage, queue_size=1)

rospy.init_node("blob_tracking_node", log_level=rospy.DEBUG)

srv = Server(HSVLimitsConfig, reconfigure_callback)

blob_tracker = BlobTracker()
serviceImage = CameraSensor()

# node namespace
ns = rospy.get_namespace()

# define a loop rate
loop_rate = rospy.Rate(rospy.get_param(ns + "loop_rate"))

while not rospy.is_shutdown():
    # Get most recent image
    cv_image = serviceImage.get_image()

    # Detect blobs
    image_with_keypoints = detect_publish(cv_image, hsv_min, hsv_max, blob_tracker)

    try:
        pub_image_raw.publish(bridge.cv2_to_imgmsg(image_with_keypoints, encoding='bgr8'))
        pub_image_compressed.publish(bridge.cv2_to_compressed_imgmsg(image_with_keypoints, dst_format='jpeg'))
    except CvBridgeError as e:
        rospy.signal_shutdown("Error in CvBridge")

    loop_rate.sleep()

rospy.logwarn("Shutting down")
