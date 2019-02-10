import argparse

import numpy as np


import cv2

from cv_bridge import CvBridge, CvBridgeError

from baxter_core_msgs.srv import (
    ListCameras,
)

import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#from sklearn.externals import joblib
# from scipy.cluster.vq import vq, kmeans, whiten
# from object_recognition.msg import ObjectInfo


import rospy

import baxter_interface

import argparse
import socket
import sys

import rospy
import rosgraph

import std_srvs.srv

from baxter_core_msgs.srv import (
    ListCameras,
)
from baxter_interface.camera import CameraController


# def main():

#     """Camera Display Example

#     """

#     rp = baxter_interface.RobotParams()

#     # valid_cameras = rp.get_camera_names()

#     valid_cameras = list_cameras()

#     if not valid_cameras:

#         rp.log_message(("Cannot detect any camera_config"

#             " parameters on this robot. Exiting."), "ERROR")

#         return

#     arg_fmt = argparse.RawDescriptionHelpFormatter

#     parser = argparse.ArgumentParser(formatter_class=arg_fmt,

#                                      description=main.__doc__)

#     parser.add_argument(

#         '-c', '--camera', type=str, default="head_camera",

#         choices=valid_cameras, help='Setup Camera Name for Camera Display')

#     parser.add_argument(

#         '-r', '--raw', action='store_true', 

#         help='Specify use of the raw image (unrectified) topic')

#     parser.add_argument(

#         '-e', '--edge', action='store_true',

#         help='Streaming the Canny edge detection image')

#     args = parser.parse_args()

#     print("Initializing node... ")

#     rospy.init_node('camera_display', anonymous=True)

#     #camera = intera_interface.Cameras()
#     camera = CameraController(valid_cameras[1])
#     print "VALDI CAMERA 0", valid_cameras[1]
#     #camera = valid_cameras[0]

#     # if not camera.verify_camera_exists(args.camera):

#     #     rospy.logerr("Invalid camera name, exiting the example.")

#     #     return
#     camera.open()
#     #camera.start_streaming(args.camera)

#     rectify_image = not args.raw

#     use_canny_edge = args.edge


#     show_image_callback(rectify_image, (use_canny_edge, args.camera))

#     # camera.set_callback(args.camera, show_image_callback,

#     #     rectify_image=rectify_image, callback_args=(use_canny_edge, args.camera))


#     def clean_shutdown():

#         print("Shutting down camera_display node.")

#         cv2.destroyAllWindows()


#     rospy.on_shutdown(clean_shutdown)

#     rospy.loginfo("Camera_display node running. Ctrl-c to quit")

#     rospy.spin()\
class webcam_image:

	def __init__(self):

	    self.bridge = CvBridge()
	    # baxter camera Subscriber
	    self.image_sub = rospy.Subscriber("/cameras/left_hand_camera/image",Image,self.callback)
	    # Subscriber to determine whether or not to use vision
	    self.is_moving_sub = rospy.Subscriber("is_moving",Bool,self.check_moving)
	    # Publisher
	   #self.object_location_pub = rospy.Publisher("object_location",ObjectInfo,queue_size=10)

	    # self.object_info = ObjectInfo()
	    # self.object_info.names = ['','','']
	    # self.object_info.x = [0,0,0]
	    # self.object_info.y = [0,0,0]
	    # self.object_info.theta = [0,0,0]
	    self.is_moving = False
	    print("asdfasdf")

	def check_moving(self,data):
	    self.is_moving = data.data

	def callback(self,data):
	    if not self.is_moving:
	        try:
	            frame = self.bridge.imgmsg_to_cv2(data,"bgr8")
	        except CvBridgeError as e:
	            print("==[CAMERA MANAGER]==",e)

	        scale = 0.75
	        frame = (frame*scale).astype(np.uint8)

	        # Split frame into left and right halves
	        left_frame = frame[0:frame.shape[0], 0:frame.shape[1]/3]
	        middle_frame = frame[0:frame.shape[0], frame.shape[1]/3:2*frame.shape[1]/3]
	        right_frame = frame[0:frame.shape[0], 2*frame.shape[1]/3:frame.shape[1]]

	        # Create list of left and right images
	        frames = [left_frame, middle_frame, right_frame]

	        # Combine smaller frames into one
	        frame[0:frame.shape[0], 0:frame.shape[1]/3] = frames[0]
	        frame[0:frame.shape[0], frame.shape[1]/3:2*frame.shape[1]/3] = frames[1]
	        frame[0:frame.shape[0], 2*frame.shape[1]/3:frame.shape[1]] = frames[2]

	        # Add a dividing line down the middle of the frame
	        cv2.line(frame, (frame.shape[1]/3,0), (frame.shape[1]/3,frame.shape[0]), (255,255,255), 1)
	        cv2.line(frame, (2*frame.shape[1]/3,0), (2*frame.shape[1]/3,frame.shape[0]), (255,255,255), 1)

	        # Display the resulting frame
	        cv2.imshow('f', frame)

	        #self.object_location_pub.publish(self.object_info)

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