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


# In[3]:
rospy.init_node("sawyer")


# In[4]:
limb = intera_interface.Limb('right')


# In[5]:
robot_state = intera_interface.RobotEnable()


# In[6]:
robot_state.enable()


# In[7]:


# Joint Position Example


# In[8]:
current_position = limb.joint_angles()
print(current_position)


# In[9]:
new_position = copy.copy(current_position)

joint_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
##### DELET HEAD PAN, TORSO!
#[farm]	[head_pan, right_j0, right_j1, right_j2, right_j3, right_j4, right_j5, right_j6, torso_t0]
#[home] [0.526962890625, -0.4500732421875, -1.3575419921875, 0.298607421875, 2.448572265625, -0.066544921875, 0.5279326171875, 3.0836171875, 0.0]
home = [-0.4500732421875, -1.3575419921875, 0.298607421875, 2.448572265625, -0.066544921875, 0.5279326171875, 3.0836171875]
seed = [0.6252626953125, -0.503607421875, 0.148115234375, 1.6371318359375, -0.344052734375, 0.4533330078125, 2.8494384765625]
dump = [-1.7319931640625, -0.4028916015625, 0.7388515625, 1.524953125, -0.9330498046875, 0.8798076171875, 1.1140146484375]


new_position = {}
for i in range(len(joint_names)):
	new_position[joint_names[i]] = dump[i]


# In[10]:
limb.move_to_joint_positions(new_position)


# In[11]:
for key in new_position:
    new_position[key] = 0.0


# In[12]:
limb.move_to_joint_positions(new_position)


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

