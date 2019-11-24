
import rospy
import baxter_interface
rospy.init_node('Hello_Baxter')



limb = baxter_interface.Limb('left')
angles = limb.joint_angles()
print angles


pickup={'left_w0': -0.12540292940963257, 'left_w1': -0.004985437560627594, 'left_w2': -0.13268933815208828, 'left_e0': 0.05253884198507542, 'left_e1': 2.0436459046603423, 'left_s0': 0.2017184736069319, 'left_s1': -0.5572185211993765}
lift={'left_w0': -0.17717478100076528, 'left_w1': 0.45904375077471005, 'left_w2': 0.06902913545484361, 'left_e0': 0.1457281748491143, 'left_e1': 2.154476016585064, 'left_s0': 0.0782330201821561, 'left_s1': -1.1217234511412089}
handout={'left_w0': -1.9750002644024702, 'left_w1': 0.5982525072753113, 'left_w2': 1.2973642513540886, 'left_e0': -0.566422405926689, 'left_e1': 0.769291365124535, 'left_s0': -0.36393694192581444, 'left_s1': 0.010737865515197896}
limb.move_to_joint_positions(angles)


for _move in range(3):
 	limb.move_to_joint_positions(pickup)
 	limb.move_to_joint_positions(lift)
 	limb.move_to_joint_positions(handout)
 	limb.move_to_joint_positions(lift)


