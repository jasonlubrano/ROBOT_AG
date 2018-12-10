#!/usr/bin/env python
# coding: utf-8

# In[1]:
import rospy
import intera_interface
import copy	


# In[2]:
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

"""
# In[3]:
rospy.init_node("sawyer")


# In[4]:
limb = intera_interface.Limb('right')


# In[5]:
robot_state = intera_interface.RobotEnable()


# In[6]:
robot_state.enable()


# In[7]:

"""
# Joint Position Example

def get_current_position(limb):
    current_position = limb.joint_angles()
    return current_position
#print(current_position)

# In[8]:
#current_position = limb.joint_angles()
#print(current_position)


# In[9]:
#new_position = copy.copy(current_position)
    
joint_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
##### DELET HEAD PAN, TORSO!
#[farm]	[head_pan, right_j0, right_j1, right_j2, right_j3, right_j4, right_j5, right_j6, torso_t0]
#[home] [0.526962890625, -0.4500732421875, -1.3575419921875, 0.298607421875, 2.448572265625, -0.066544921875, 0.5279326171875, 3.0836171875, 0.0]

# starting positon
home =              [-0.0479130859375,-0.956372070313,-1.55625390625,1.59204199219,0.622408203125,1.55823828125,-0.030720703125]

# hover positions
hover_seeds =       [-0.43649609375,-0.772165039063,-0.244763671875,1.48357128906,0.201188476562,0.982875,4.11783691406]
hover_plot =        [0.040599609375,-0.827349609375,-0.038009765625,1.81553417969,0.164611328125,0.55252734375,4.68306445312]
hover_dump =        [-0.731625,-1.13401757812,-0.303334960938,2.09787695313,0.103762695313,0.694225585937,3.82856640625]

# hover seeds -> seperated based on color
hover_seed_1 =      [-0.566900390625,-0.601936523438,0.113004882812,1.30038671875,-0.176081054687,0.903537109375,4.42868066406]
hover_seed_2 =      [-0.49059375,-0.649911132813,-0.13023828125,1.52984375,0.206120117188,0.7341328125,4.19802636719]
hover_seed_3 =      [-0.458711914063,-0.65115234375,-0.207451171875,1.59506445313,0.206740234375,0.755823242187,4.19802636719]
hover_seed_4 =      [-0.528341796875,-0.7457578125,-0.149034179687,1.73819433594,0.104989257813,0.678874023438,4.19802636719]
# 4 pos


# seeds -> seperated based on color
down_seed_1 =       [-0.540987304688,-0.50721484375,0.0320830078125,1.29290625,-0.043705078125,0.802290039062,4.42868066406]
down_seed_2 =       [-0.392676757812,-0.46336328125,-0.2749921875,1.42248339844,0.374944335937,0.737206054688,4.19802636719]
down_seed_3 =       [-0.431041992188,-0.537862304687,-0.243608398438,1.54004003906,0.265485351563,0.706709960938,4.19802636719]
down_seed_4 =       [-0.503657226563,-0.630004882812,-0.176357421875,1.70575488281,0.179821289063,0.614784179687,4.19802636719]
# 4 pos

#hover over plots
hover_plot_1 =      [0.121499023438,-0.809302734375,-0.0940908203125,1.99093164062,0.291978515625,0.4340703125,4.713328125]
hover_plot_2 =      [-0.106770507812,-0.794239257813,-0.185759765625,1.96913964844,0.301635742188,0.5328515625,4.33520507812]
hover_plot_3 =      [-0.11922265625,-0.540956054687,-0.116649414062,1.46575,0.165231445313,0.711009765625,4.51324707031]
hover_polt_4 =      [0.258836914062,-0.440556640625,-0.350942382813,1.37503808594,0.519624023438,0.79369140625,4.58851757812]

# plots
down_plot_1 =       [0.0658193359375,-0.603280273438,-0.0192138671875,1.9125,0.259120117187,0.332318359375,4.71312109375]
down_plot_2 =       [-0.28651953125,-0.584396484375,0.052755859375,2.0208984375,0.011216796875,0.141133789062,4.51304003906]
down_plot_3 =       [-0.0410185546875,-0.415176757813,-0.189088867188,1.53343457031,0.388293945312,0.513188476562,4.34784472656]
down_plot_4 =       [0.269545898437,-0.202284179688,-0.37003515625,1.23909082031,0.588981445312,0.780186523437,4.58831152344]
# 4 plots


def create_new_position(position, limb):
    new_position = {}
    for i in range(len(joint_names)):
        new_position[joint_names[i]] = position[i]
    move_to_position(new_position, limb)


"""
new_position = {}
for i in range(len(joint_names)):
	new_position[joint_names[i]] = home[i]
"""


def move_to_position(position, limb):
    limb.move_to_joint_positions(position)

"""
# In[11]:
for key in new_position:
    new_position[key] = 0.0


# In[12]:
limb.move_to_joint_positions(new_position)
"""

# In[ ]:


# Gripper Example

'''
# In[13]:
gripper = intera_interface.Gripper('right_gripper')


# In[14]:
gripper.get_position()


# In[15]:
gripper.set_position(100)


# In[16]:
gripper.set_position(0)
'''

# In[ ]:





# In[ ]:





# In[51]:


# Inverse Kinematics Example


# In[17]:
'''ik_service_name = "ExternalTools/right/PositionKinematicsNode/IKService"


# In[18]:
iksvc = rospy.ServiceProxy(ik_service_name, SolvePositionIK)


# In[19]:
ikreq = SolvePositionIKRequest()


# In[20]:
hdr = Header(stamp=rospy.Time.now(), frame_id='base')


# In[21]:
desired_pose = PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=-0.5,
                    y=-0.1,
                    z=-0.2,
                ),
                orientation=Quaternion(
                    x=0.7,
                    y=0.7,
                    z=0.1,
                    w=0.1,                )
            )
        )


# In[22]:
ikreq.pose_stamp.append(desired_pose)


# In[23]:
ikreq.tip_names.append('right_gripper_tip')


# In[24]:
rospy.wait_for_service(ik_service_name,5)


# In[25]:
resp = iksvc(ikreq)


# In[26]:
print(resp)


# In[27]:
#Home position
#joints = []
#joints[0] = [0.526962890625, -0.4500732421875, -1.3575419921875, 0.298607421875, 2.448572265625, -0.066544921875, 0.5279326171875, 3.0836171875]

target_pose = dict(zip(resp.joints[0].name, resp.joints[0].position))

# In[28]:
limb.set_joint_positions(target_pose)


# In[ ]:



'''

# In[29]:


# Euler Angle to Quaternion Example


# In[32]:
import tf


# In[33]:
zero_angle_quaternion= tf.transformations.quaternion_from_euler(0,0,0)


# In[34]:
print(zero_angle_quaternion)


# In[ ]:

