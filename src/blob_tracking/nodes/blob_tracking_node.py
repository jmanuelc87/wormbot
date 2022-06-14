#! /usr/bin/env python
import math
import rospy
from blob_tracking import BlobColorTracker, detect_publish
from blob_tracking.cfg import LimitsConfig
from camera import CameraSensor
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image, CompressedImage

# HSV limits default values
lowerb = [0, 0, 0]
upperb = [255, 255, 255]
params = [5, 2.7, (5, 5)]
show = False

# We define the detection area [x_min, y_min, x_max, y_max] adimensional (0.0 to 1.0) starting from top left corner
window = [0.1, 0.1, 0.9, 0.9]

ns = rospy.get_namespace()


colors = {
    1: [0, 255, 255],
    2: [120, 255, 255],
    3: [60, 255, 255],
    4: [90, 255, 255],
    5: [101, 255, 255]
}


def reconfigure_color_callback(config, level):
    pass


def reconfigure_callback(config, level):
    rospy.loginfo("Selecting: %s", config.color_s)
    if config.color_s == 0:
        upperb[0] = config.h_max
        upperb[1] = config.s_max
        upperb[2] = config.v_max
        lowerb[0] = config.h_min
        lowerb[1] = config.s_min
        lowerb[2] = config.v_min
    else:
        s_color = colors[config.color_s]
        upperb[0] = min(s_color[0] + 10, 255)
        upperb[1] = s_color[1]
        upperb[2] = s_color[2]
        lowerb[0] = max(s_color[0] - 10, 0)
        lowerb[1] = s_color[1]
        lowerb[2] = s_color[2]

    params[0] = config.blur
    params[1] = config.sigma
    params[2] = (config.kernel, config.kernel)
    show = config.show

    rospy.loginfo("upperb: %s, lowerb: %s, publish: %s", upperb, lowerb, show)
    return config


bridge = CvBridge()

pub_image_raw = rospy.Publisher(
    ns + "camera/keypoints/raw", Image, queue_size=1)
pub_image_compressed = rospy.Publisher(
    ns + "camera/keypoints/compressed", CompressedImage, queue_size=1)

rospy.init_node("blob_tracking_node", log_level=rospy.DEBUG)

srv = Server(LimitsConfig, reconfigure_callback)

blob_tracker = BlobColorTracker()
serviceImage = CameraSensor()

# node namespace
ns = rospy.get_namespace()

# define a loop rate
loop_rate = rospy.Rate(rospy.get_param(ns + "loop_rate"))

while not rospy.is_shutdown():
    # Get most recent image
    cv_image = serviceImage.get_image()

    # Detect blobs
    image_with_keypoints = detect_publish(cv_image, tuple(lowerb), tuple(
        upperb), blob_tracker, show_image=show, params=params)

    try:
        pub_image_raw.publish(bridge.cv2_to_imgmsg(
            image_with_keypoints, encoding='bgr8'))
        pub_image_compressed.publish(bridge.cv2_to_compressed_imgmsg(
            image_with_keypoints, dst_format='jpeg'))
    except CvBridgeError as e:
        rospy.signal_shutdown("Error in CvBridge")

    loop_rate.sleep()

rospy.logwarn("Shutting down")
