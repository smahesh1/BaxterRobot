"""
This file is used to create a box around a face detected in the camera, based on OpenCV's default face data
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
        self.image_sub = rospy.Subscriber("/cameras/head_camera/image", Image, self.callback)
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
            face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
            eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, 1.3, 5)
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                roi_gray = gray[y:y + h, x:x + w]
                roi_color = frame[y:y + h, x:x + w]
                eyes = eye_cascade.detectMultiScale(roi_gray)
                for (ex, ey, ew, eh) in eyes:
                    cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0), 2)

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
