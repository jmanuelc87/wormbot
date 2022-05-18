import cv2


def rescaleFrame(frame, scale=0.75):
    """
    Resize video or image frame
    :param frame: the video or image frame
    :param scale: the value to rescale the image
    :return: rescaled frame
    """
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dimensions = (width, height)

    return cv2.resize(frame, dimensions, interpolation=cv2.INTER_AREA)
