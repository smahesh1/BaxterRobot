"""
Executes arm motion, based on pickup_object.py
"""

from grip_node import GripperClient

import rospy
from std_msgs.msg import Bool, String
import motion_planning_node as pnode
import baxter_interface


def pickup():
	pass


def arm_setup():
	### TODO: figure out what values we want for default arm position
    # Get desired joint values from parameter server
    left_w0 = rospy.get_param('left_w0',default =0)
    left_w1 = rospy.get_param('left_w1',default =0)
    left_w2 = rospy.get_param('left_w2',default =0)
    left_e0 = rospy.get_param('left_e0',default =0)
    left_e1 = rospy.get_param('left_e1',default =0)
    left_s0 = rospy.get_param('left_s0',default =0)
    left_s1 = rospy.get_param('left_s1',default =0)

    # Send the left arm to the desired position
    home = {'left_w0': left_w0, 'left_w1': left_w1, 'left_w2': left_w2, 'left_e0': left_e0, 'left_e1': left_e1, 'left_s0': left_s0, 'left_s1': left_s1}
    limb = baxter_interface.Limb('left')
    limb.move_to_joint_positions(home)


# Left out some stuff, we can add in if we need it - Daniel
if __name__ == '__main__':
	rospy.init_node('pickup_object', log_level=rospy.INFO)

	print "Moving arm to correct location"
	arm_setup()

	rate = rospy.Rate(50)

	is_moving_pub = rospy.Publisher("is_moving",Bool,queue_size=10)
	is_moving_pub.publish(False)
	print "Ready to go!"
	rospy.spin()
