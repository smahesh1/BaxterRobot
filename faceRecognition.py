"""
This file is used to create a box around a face detected in the camera, based on OpenCV's default face data
"""

import argparse
import baxter_interface

from cv_bridge import CvBridge, CvBridgeError

from baxter_core_msgs.srv import (
    ListCameras,
)

import cv2
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import random

import rospy

from baxter_interface import CHECK_VERSION


class webcam_image:

    def __init__(self):

        self.limb = baxter_interface.Limb('right')
        self.bridge = CvBridge()
        # baxter camera Subscriber
        self.image_sub = rospy.Subscriber("/cameras/head_camera/image", Image, self.callback)
        # Subscriber to determine whether or not to use vision
        self.is_moving_sub = rospy.Subscriber("is_moving", Bool, self.check_moving)
        self.is_moving = False
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()


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
            path = "/home/ricerobotics/Downloads/"
            face_cascade = cv2.CascadeClassifier(path + 'haarcascade_frontalface_default.xml')
            #eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(30, 30),
                flags=cv2.CASCADE_SCALE_IMAGE
            )
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                # roi_gray = gray[y:y + h, x:x + w]
                # roi_color = frame[y:y + h, x:x + w]
                # eyes = eye_cascade.detectMultiScale(roi_gray)
                # for (ex, ey, ew, eh) in eyes:
                #     cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0), 2)


            # Display on Robot Face:
            msg = CvBridge().cv2_to_imgmsg(frame, encoding="bgr8")

            pub = rospy.Publisher('/robot/xdisplay', Image, latch = True)
            pub.publish(msg)

            # Display the resulting frame
            cv2.imshow('f', frame)
            guess = random.randrange(1,25)
            print(guess)

            cv2.waitKey(1)
            if len(faces) > 0 and guess == 7:
                wave_1 = {'right_s0': -0.459, 'right_s1': -0.202, 'right_e0': 1.807, 'right_e1': 1.714, 'right_w0': -0.906, 'right_w1': -1.545, 'right_w2': -0.276}
                wave_2 = {'right_s0': -0.395, 'right_s1': -0.202, 'right_e0': 1.831, 'right_e1': 1.981, 'right_w0': -1.979, 'right_w1': -1.100, 'right_w2': -0.448}

                #for _move in range(1):
                self.limb.move_to_joint_positions(wave_1)
                self.limb.move_to_joint_positions(wave_2)

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose.
        """
        print("Moving to neutral pose...")
        self.limb.move_to_neutral()

    def clean_shutdown(self):
        print("\nExiting example...")
        #return to normal
        self.set_neutral()
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True


def main():
    rospy.init_node('webcam_image', anonymous=True)
    limb = baxter_interface.Limb('right')
    limb_left = baxter_interface.Limb('left')
    angles = limb.joint_angles()
    print angles

    # angles['right_s0'] = 0.0
    # angles['right_s1'] = 0.0
    # angles['right_e0'] = 0.0
    # angles['right_e1'] = 0.0
    # angles['right_w0'] = 0.0
    # angles['right_w1'] = 0.0
    # angles['right_w2'] = 0.0

    # print angles 
    # limb.move_to_joint_positions(angles)

    # wave_1 = {'right_s0': -0.459, 'right_s1': -0.202, 'right_e0': 1.807, 'right_e1': 1.714, 'right_w0': -0.906, 'right_w1': -1.545, 'right_w2': -0.276}
    # limb.move_to_joint_positions(wave_1)

    limb.move_to_neutral()
    limb_left.move_to_neutral()
    ic = webcam_image()
    rospy.on_shutdown(ic.clean_shutdown)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()



main()
