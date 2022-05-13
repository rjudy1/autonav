#!/usr/bin/env python

# ********************************************* #
# Cedarville University                         #
# AutoNav Senior Design Team 2020-2021          #
# Top-Level AutoNav Class                       #
# ********************************************* #

import cv2
import numpy as np

class AutoNavClass:

    def __init__(self, name):
        pass

    def hsv_filter(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower = np.array([0, 65, 100])
        upper = np.array([179, 255, 255])
        return cv2.inRange(hsv, lower, upper)

# functions that need to be called outside of a class
# Function used to display images in a way that allows them to be closed when not used.
def closeWindows(handleArr):
    if len(handleArr) > 0:
        for handle in handleArr:
            cv2.destroyWindow(handle)
            handleArr.remove(handle)
    
    # Closes unwanted windows
def cvDisplay(image, handle, handleArr):
    cv2.imshow(handle, image)
    cv2.waitKey(2)
    if handle not in handleArr:
        handleArr.append(handle)
