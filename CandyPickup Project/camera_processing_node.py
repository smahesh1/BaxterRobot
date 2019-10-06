"""
This node gets camera data and (eventually) publishes
whether or not there is a candy bar in the grippers
"""

import sys
import rospy
import cv2
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# TODO: implement camera prediction based off baxter_object_prediction.py 
# ... in apollack11's project


class webcam_image:
	def __init__(self):
		self.bridge = CvBridge()
		# baxter camera Subscriber
		self.image_sub = rospy.Subscriber("/cameras/left_hand_camera/image",Image,self.callback)
		# Subscriber to determine whether or not to use vision
		self.is_moving_sub = rospy.Subscriber("is_moving",Bool,self.check_moving)
		# Publisher: may need to be rephrased but essentially what we want
		self.has_candy_pub = rospy.Publisher("has_candy",Bool,queue_size=10)
		has_candy_pub.publish(False)

	### TODO: fill in other methods we need



def main(args):
	rospy.init_node('webcam_image', anonymous=True)
	ic = webcam_image()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()


if __name__ == '__main__':
	print "OpenCV Version:",cv2.__version__

	### TODO do stuff here we need