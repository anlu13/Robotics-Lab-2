import cv2
import numpy as np
import time

def filter_image(img, hsv_lower, hsv_upper):
    img_filt = cv2.medianBlur(img, 5)
    hsv = cv2.cvtColor(img_filt, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
    return mask

    ###############################################################################
    ### You might need to change the parameter values to get better results
    ###############################################################################
def detect_blob(mask):
    # copy mask to img (originally was medianBlue() step)
    img = mask

    # Set up the SimpleBlobdetector with default parameters with specific values.
    params = cv2.SimpleBlobDetector_Params()

    # ADD (parameter) CODE HERE
    # blob is filtered by color
    params.filterByColor = True
    # color is dark
    params.blobColor = 255

    # blob is filtered by area
    params.filterByArea = True
    # minimum of 350 pixel area
    params.minArea = 350
    # max 100,000 pixel area (unsure if even need this?)
    params.maxArea = 100000

    # blob is not circular
    params.filterByCircularity = False

    # blob is not convex - cubes does not bulge outwards
    params.filterByConvexity = False

    # blob is inertia - meaning how stretched out a shape is
    params.filterByInertia = True
    # blobs longest side is no more than 1.5x longer than the shorter side. Needed for "angled" cubes
    params.maxInertiaRatio = 1.5

    # builds a blob detector with the given parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # use the detector to detect blobs.
    keypoints = detector.detect(img)

    # use the detector to detect blobs.
    # keypointsImage = cv2.drawKeypoints(
    #     img, keypoints,
    #     np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # cv2.imshow("Blobs Detected", keypointsImage)
    # cv2.moveWindow("Blobs Detected", 640, -75)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    return keypoints


def find_cube(img, hsv_lower, hsv_upper):
    """Find the cube in an image.
        Arguments:
        img -- the image
        hsv_lower -- the h, s, and v lower bounds
        hsv_upper -- the h, s, and v upper bounds
        Returns [x, y, radius] of the target blob, and [0,0,0] or None if no blob is found.
    """
    mask = filter_image(img, hsv_lower, hsv_upper)
    keypoints = detect_blob(mask)

    if keypoints == []:
        return None
    
    largest_keypoint = [0, 0, 0]

    for kp in keypoints:
        radius = kp.size / 2
        if radius > largest_keypoint[2]:
            largest_keypoint = [kp.pt[0], kp.pt[1], radius]


    return largest_keypoint

