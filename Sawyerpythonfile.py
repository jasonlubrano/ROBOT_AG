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

rospy.init_node("sawyer_demo_node")


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
new_position['right_j0'] -=1


# In[10]:


limb.move_to_joint_positions(new_position)


# In[11]:


for key in new_position:
    new_position[key] = 0.0


# In[12]:


limb.move_to_joint_positions(new_position)


# In[ ]:


# Gripper Example


# In[13]:


gripper = intera_interface.Gripper('right_gripper')


# In[14]:


gripper.get_position()


# In[15]:


gripper.set_position(100)


# In[16]:


gripper.set_position(0)


# In[ ]:





# In[ ]:





# In[51]:


# Inverse Kinematics Example


# In[17]:


ik_service_name = "ExternalTools/right/PositionKinematicsNode/IKService"


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
                    x=0.704020578925,
                    y=0.710172716916,
                    z=0.00244101361829,
                    w=0.00194372088834,
                )
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


target_pose = dict(zip(resp.joints[0].name, resp.joints[0].position))


# In[28]:


limb.set_joint_positions(target_pose)


# In[ ]:





# In[29]:


# Euler Angle to Quaternion Example


# In[32]:


import tf


# In[33]:


zero_angle_quaternion= tf.transformations.quaternion_from_euler(0,0,0)


# In[34]:


print(zero_angle_quaternion)


# In[ ]:




