#!/usr/bin/env python

import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration


# Function that converts image coord to world coord
# Note: input c corresponds to columns in the image, input r is rows in the image
def IMG2W(c,r):

    theta = 0.85681/180*np.pi #radians
    beta = 760.938 #pix/m
    # tx = 0.185252 #meters
    # ty = 0.2692 #meters
    tx = 0.26329846 - 0.0075
    ty = 0.04802061 - 0.0075 + 0.03
    T = np.array([[tx], [ty]])
    O_r = 240
    O_c = 320

    xc = (r - O_r)/beta
    yc = (c - O_c)/beta

    # print(xc, yc)

    R_cw = np.array([[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta),  np.cos(theta)]])
    world = np.matmul(R_cw, np.array([[xc], [yc]])) +T
    # print(world)

    # print(world + T, '\n')
            #x          y
    return world[0][0], world[1][0]

# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 150
    # params.maxArea = 500

    # Filter by Circularity
    params.filterByCircularity = False

    params.maxCircularity = 0.8

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False
    params.minInertiaRatio = 0.9

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================

    # if  color == 'orange':
    #     lower = (20,90,90)     # orange lower
    #     upper = (38,150,150)   # orange upper
    if color == 'green':
        lower = (52, 90, 60) #green lower
        upper = (82, 255, 255)# green upper
    elif color == 'red':
        lower = (0, 90, 90)   #red lower
        upper = (20, 255, 255)   # red upper
    elif color == 'blue':
        lower = (90,90,90)
        upper = (130, 255, 255)
    else:
        print('pick green or red or blue')



    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # Draw the keypoints on the detected block
    # print(keypoints)
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, None)

    # print(blob_image_center)
    xw_yw = []
    # if(num_blobs == 0):
    #     print("No block found!")
    # else:
    #     # Convert image coordinates to global world coordinate using IM2W() function
    #     # print(color + str(num_blobs))
    #     for i in range(num_blobs):
    #         # print(blob_image_center[i][1], blob_image_center[i][0])
    #         xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    if(num_blobs != 0):
        # Convert image coordinates to global world coordinate using IM2W() function
        # print(color + str(num_blobs))
        for i in range(num_blobs):
            # print(blob_image_center[i][1], blob_image_center[i][0])
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    # cv2.namedWindow("Camera View")
    # cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
