"""
This file was created by Daniel using the Camera Test file developed 2/10.

It is intended to test different filters and settings on the Baxter Robot to increase
knowledge in preparation for image processing.
"""


import argparse

from cv_bridge import CvBridge, CvBridgeError

from baxter_core_msgs.srv import (
    ListCameras,
)

import cv2
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import rospy


class webcam_image:

    def __init__(self):

        self.bridge = CvBridge()
        # baxter camera Subscriber
        self.image_sub = rospy.Subscriber("/cameras/left_hand_camera/image", Image, self.callback)
        # Subscriber to determine whether or not to use vision
        self.is_moving_sub = rospy.Subscriber("is_moving", Bool, self.check_moving)
        self.is_moving = False

    def check_moving(self, data):
        self.is_moving = data.data

    def callback(self, data):
        if not self.is_moving:
            try:
                frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print("==[CAMERA MANAGER]==", e)

            scale = 0.75
            frame = (frame * scale).astype(np.uint8)

            # Frame Processing goes here
            #Hopefully
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (3, 3), 0)
            get_edge = cv2.Canny(blurred, 10, 100)
            frame = np.hstack([get_edge])

            # Display the resulting frame
            cv2.imshow('f', frame)

            cv2.waitKey(1)


def main():
    rospy.init_node('webcam_image', anonymous=True)
    ic = webcam_image()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


main()
