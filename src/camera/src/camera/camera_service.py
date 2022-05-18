import rospy

from camera.srv import ServiceImage, ServiceImageRequest


class CameraService:

    def __init__(self):
        self.ns = rospy.get_namespace()
        self.service_name = self.ns + 'get_camera_picture'
        self.get_camera_picture = rospy.ServiceProxy(self.service_name, ServiceImage)
        rospy.wait_for_service(self.service_name)

    def get_frame(self, rescale=1.0):
        frame = None
        try:
            srv = ServiceImageRequest()
            srv.rescale = rescale
            frame = self.get_camera_picture(srv)
        except rospy.ServiceException as err:
            rospy.logerr("Error requesting service %s with %s", self.service_name, err)

        rospy.logdebug('%s', frame)

        return frame.image
