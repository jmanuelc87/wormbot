#! /usr/bin/env python
import cv2
import numpy as np
import os
import rospkg
import rospy

from camera import rescaleFrame
from camera.srv import ServiceImage, ServiceImageResponse
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from pip._internal.utils.misc import get_installed_distributions

installed_packages = [package.project_name for package in get_installed_distributions()]

if 'jetcam' in installed_packages:
    from jetcam import CSICamera
    from jetcam import USBCamera


def main():
    ns = rospy.get_namespace()

    use_csi_camera = rospy.get_param(ns + "use_csi_camera")
    use_cv2_camera = rospy.get_param(ns + "use_cv2_camera")

    width = rospy.get_param(ns + "width")

    height = rospy.get_param(ns + "height")

    rate = rospy.get_param(ns + "rate")

    npzfile = rospy.get_param(ns + "npzfilepath")

    usecalibration = rospy.get_param(ns + 'usecalibration')

    rospack = rospkg.RosPack()

    image = None

    def handle_camera_picture(req):
        if image is not None:
            frame = rescaleFrame(image, scale=req.rescale)
            srv = ServiceImageResponse()
            srv.image = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            srv.retval = 1
            return srv
        else:
            srv = ServiceImageResponse()
            srv.retval = 0
            return srv

    rospy.init_node("camera_node", log_level=rospy.DEBUG)

    rospy.Service(ns + 'get_camera_picture', ServiceImage, handle_camera_picture)

    if use_csi_camera:
        rospy.loginfo("Using CSICamera %s", use_csi_camera)
        camera = CSICamera(width=width, height=height)
    else:
        if not use_cv2_camera:
            rospy.loginfo("Using USBCamera")
            camera = USBCamera(width=width, height=height)
        else:
            rospy.loginfo("Using CV2 Camera")
            camera = cv2.VideoCapture(0)

    loop = rospy.Rate(rate)

    image_pub = rospy.Publisher(ns + "camera/raw", Image, queue_size=1)

    image_compressed_pub = rospy.Publisher(
        ns + "camera/compressed", CompressedImage, queue_size=1)

    bridge = CvBridge()

    path = rospack.get_path('camera') + '/' + npzfile

    if usecalibration and os.path.exists(path):
        calibrationvalues = np.load(path)
        newcameramatrix, roi = cv2.getOptimalNewCameraMatrix(
            calibrationvalues['mtx'], calibrationvalues['dist'], (width, height), 1, (width, height))

    while not rospy.is_shutdown():
        if use_csi_camera:
            try:
                image = camera.read()
                retval = True
            except RuntimeError as err:
                retval = False
        else:
            if not use_cv2_camera:
                retval, image = camera.read()
            else:
                retval, image = camera.read()

                if not retval:
                    raise RuntimeError('Could not read Image from camera')

        if usecalibration and newcameramatrix and calibrationvalues and roi:
            dst = cv2.undistort(
                image, calibrationvalues['mtx'], calibrationvalues['dist'], None, newcameramatrix)
            x, y, w, h = roi
            image = dst[y:y + h, x:x + w]

        if not retval:
            loop.sleep()
            continue

        try:
            image_pub.publish(bridge.cv2_to_imgmsg(image, encoding='bgr8'))
            image_compressed_pub.publish(bridge.cv2_to_compressed_imgmsg(image, dst_format='jpeg'))
        except CvBridgeError as e:
            print(e)
            rospy.signal_shutdown("Error in CvBridge")

        loop.sleep()


if __name__ == "__main__":
    main()
