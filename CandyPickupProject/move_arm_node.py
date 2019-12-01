"""
Executes arm motion, based on pickup_object.py
"""

from grip_node import GripperClient

import rospy
from std_msgs.msg import Bool, String
import motion_planning_node as pnode
import baxter_interface


has_candy = None
is_moving = False

pickup={'left_w0': -0.12540292940963257, 'left_w1': -0.004985437560627594, 'left_w2': -0.13268933815208828, 'left_e0': 0.05253884198507542, 'left_e1': 2.0436459046603423, 'left_s0': 0.2017184736069319, 'left_s1': -0.5572185211993765}
lift={'left_w0': -0.17717478100076528, 'left_w1': 0.45904375077471005, 'left_w2': 0.06902913545484361, 'left_e0': 0.1457281748491143, 'left_e1': 2.154476016585064, 'left_s0': 0.0782330201821561, 'left_s1': -1.1217234511412089}
handout={'left_w0': -1.9750002644024702, 'left_w1': 0.5982525072753113, 'left_w2': 1.2973642513540886, 'left_e0': -0.566422405926689, 'left_e1': 0.769291365124535, 'left_s0': -0.36393694192581444, 'left_s1': 0.010737865515197896}


def get_has_candy(data):
    if not is_moving:
        global has_candy
        has_candy = data
        if has_candy:
            pickup()

def pickup():
    is_moving = True
    is_moving_pub.publish(is_moving)
    # Move to intermediate position
    gc = GripperClient()
    gc.command(position=100.0, effort=50.0)
    limb = baxter_interface.Limb('left')
    limb.move_to_joint_position(lift)
    # Move to Pickup position
    limb.move_to_joint_position(pickup)
    # Close grippers
    gc.command(position=10.0, effort=50.0)
    gc.wait()
    # Move up to intermediate position
    limb.move_to_joint_position(lift)
    # Move to handout position
    limb.move_to_joint_position(handout)
    is_moving = False
    is_moving_pub.publish(is_moving)


def arm_setup():
    pickup()


# Left out some stuff, we can add in if we need it - Daniel
if __name__ == '__main__':
	rospy.init_node('pickup_object', log_level=rospy.INFO)

	print "Moving arm to initial location"
	arm_setup()
	rate = rospy.Rate(50)
    is_moving_pub = rospy.Publisher("is_moving",Bool,queue_size=10)
	is_moving_pub.publish(False)

    has_candy_sub = rospy.Subscriber("/has_candy", Bool, get_has_candy)

	print "Ready to go!"

	rospy.spin()
