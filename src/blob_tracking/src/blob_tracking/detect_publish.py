import cv2
import rospy
import numpy as np

from geometry_msgs.msg import Point


def draw_keypoints(image,  # -- Input image
                    keypoints,  # -- CV keypoints
                    line_color=(0, 0, 255),  # -- line's color (b,g,r)
                    ):
    """
    Draw detected blobs: returns the image
    return(im_with_keypoints)
    """
    # -- Draw detected blobs as red circles.
    # -- cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), line_color,
                                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    return im_with_keypoints


def publish_blob(x, y, size, pub_blob):
    blob_point = Point()
    blob_point.x = x
    blob_point.y = y
    blob_point.z = size
    pub_blob.publish(blob_point)


def get_blob_relative_position(image, keyPoint):
        """
        Obtain the camera relative frame coordinate of one single keypoint
        :param image:
        :param keyPoint:
        :return: x, y
        """
        rows = float(image.shape[0])
        cols = float(image.shape[1])

        center_x = 0.5 * cols
        center_y = 0.5 * rows

        x = (keyPoint.pt[0] - center_x) / center_x
        y = (keyPoint.pt[1] - center_y) / center_y

        return x, y


def detect_publish(image, lower, upper, blob_tracker, show_image=False, params=[5, 2.7, (5,5)]):
    image_with_keypoints = image
    keypoints = blob_tracker.blob_detect(image, lower, upper, blur=params[0], sigma=params[1], kernel=params[2], show_image=show_image)

    sorted_keypoints = sorted(keypoints[0], reverse=True, key=lambda e: e.size)

    if len(sorted_keypoints) > 0:
        keypoint = sorted_keypoints[0]

        x, y = get_blob_relative_position(image, keypoint)
        blob_size = keypoint.size
        publish_blob(x, y, blob_size, blob_tracker.pub_blob)

        image_with_keypoints = draw_keypoints(image, [keypoint])

        if show_image:
            cv2.imshow("Blob with keypoints", image_with_keypoints)

    return image_with_keypoints

    # for keypoint in keypoints:
    #    x, y = blob_tracker.get_blob_relative_position(cv_image, keypoint)
    #    blob_size = keypoint.size
    #    blob_tracker.publish_blob(x, y, blob_size)
    #
    # image_with_keypoints = blob_tracker.draw_keypoints(cv_image, keypoints)
    # cv2.imshow("Blob with keypoints", image_with_keypoints)
