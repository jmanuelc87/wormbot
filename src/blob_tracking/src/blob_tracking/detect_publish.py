import cv2
import rospy


def detect_publish(image, lower, upper, blob_tracker, show_image=False):
    image_with_keypoints = image
    keypoints = blob_tracker.blob_detect(image, lower, upper, blur=5, sigma=2.7, kernel=(5, 5), show_image=show_image)

    sorted_keypoints = sorted(keypoints[0], reverse=True, key=lambda e: e.size)

    if len(sorted_keypoints) > 0:
        keypoint = sorted_keypoints[0]

        x, y = blob_tracker.get_blob_relative_position(image, keypoint)
        blob_size = keypoint.size
        blob_tracker.publish_blob(x, y, blob_size)

        image_with_keypoints = blob_tracker.draw_keypoints(image, [keypoint])

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
