{
 "cells": [
  {
   "cell_type": "code",
   "execution_count": 1,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "import rospy\n",
    "import intera_interface\n",
    "import copy"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 2,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "from geometry_msgs.msg import (\n",
    "    PoseStamped,\n",
    "    Pose,\n",
    "    Point,\n",
    "    Quaternion,\n",
    ")\n",
    "from std_msgs.msg import Header\n",
    "from sensor_msgs.msg import JointState\n",
    "\n",
    "from intera_core_msgs.srv import (\n",
    "    SolvePositionIK,\n",
    "    SolvePositionIKRequest,\n",
    ")"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 3,
   "metadata": {
    "collapsed": false,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "rospy.init_node(\"sawyer_demo_node\")"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 4,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "limb = intera_interface.Limb('right')"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 5,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "robot_state = intera_interface.RobotEnable()"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 6,
   "metadata": {
    "collapsed": false,
    "deletable": true,
    "editable": true
   },
   "outputs": [
    {
     "name": "stdout",
     "output_type": "stream",
     "text": [
      "[INFO] [1543447425.474195, 177.074000]: Robot Enabled\n"
     ]
    }
   ],
   "source": [
    "robot_state.enable()"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 7,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "# Joint Position Example"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 8,
   "metadata": {
    "collapsed": false,
    "deletable": true,
    "editable": true
   },
   "outputs": [
    {
     "name": "stdout",
     "output_type": "stream",
     "text": [
      "{'right_j6': -0.030876760925905522, 'right_j5': -0.10264515007911701, 'right_j4': -0.01614688408149423, 'right_j3': 0.41061666992453993, 'right_j2': 0.23067483041642323, 'right_j1': 1.6174384398878479, 'right_j0': -0.1909147481041611}\n"
     ]
    }
   ],
   "source": [
    "current_position = limb.joint_angles()\n",
    "print current_position"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 9,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "new_position = copy.copy(current_position)\n",
    "new_position['right_j0'] -=1"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 10,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "limb.move_to_joint_positions(new_position)"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 11,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "for key in new_position:\n",
    "    new_position[key] = 0.0"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 12,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "limb.move_to_joint_positions(new_position)"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": null,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "# Gripper Example"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 13,
   "metadata": {
    "collapsed": false,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "gripper = intera_interface.Gripper('right_gripper')"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 14,
   "metadata": {
    "collapsed": false,
    "deletable": true,
    "editable": true
   },
   "outputs": [
    {
     "data": {
      "text/plain": [
       "0.0"
      ]
     },
     "execution_count": 14,
     "metadata": {},
     "output_type": "execute_result"
    }
   ],
   "source": [
    "gripper.get_position()"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 15,
   "metadata": {
    "collapsed": false,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "gripper.set_position(100)\n"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 16,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "gripper.set_position(0)"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": null,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": []
  },
  {
   "cell_type": "code",
   "execution_count": null,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": []
  },
  {
   "cell_type": "code",
   "execution_count": 51,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "# Inverse Kinematics Example"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 17,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "ik_service_name = \"ExternalTools/right/PositionKinematicsNode/IKService\""
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 18,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "iksvc = rospy.ServiceProxy(ik_service_name, SolvePositionIK)"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 19,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "ikreq = SolvePositionIKRequest()"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 20,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "hdr = Header(stamp=rospy.Time.now(), frame_id='base')"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 21,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "desired_pose = PoseStamped(\n",
    "            header=hdr,\n",
    "            pose=Pose(\n",
    "                position=Point(\n",
    "                    x=0.450628752997,\n",
    "                    y=0.161615832271,\n",
    "                    z=0.217447307078,\n",
    "                ),\n",
    "                orientation=Quaternion(\n",
    "                    x=0.704020578925,\n",
    "                    y=0.710172716916,\n",
    "                    z=0.00244101361829,\n",
    "                    w=0.00194372088834,\n",
    "                )\n",
    "            )\n",
    "        )"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 22,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "ikreq.pose_stamp.append(desired_pose)"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 23,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "ikreq.tip_names.append('right_gripper_tip')"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 24,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "rospy.wait_for_service(ik_service_name,5)"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 25,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "resp = iksvc(ikreq)"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 26,
   "metadata": {
    "collapsed": false,
    "deletable": true,
    "editable": true
   },
   "outputs": [
    {
     "name": "stdout",
     "output_type": "stream",
     "text": [
      "joints: \n",
      "  - \n",
      "    header: \n",
      "      seq: 0\n",
      "      stamp: \n",
      "        secs: 1543447835\n",
      "        nsecs: 523935218\n",
      "      frame_id: ''\n",
      "    name: [right_j0, right_j1, right_j2, right_j3, right_j4, right_j5, right_j6]\n",
      "    position: [0.5998664974825264, 0.33407948970083634, -2.03748629510374, 1.4991680751013094, -1.1649061168362789, -1.9787481268888447, -0.5664763070321123]\n",
      "    velocity: []\n",
      "    effort: []\n",
      "result_type: [2]\n"
     ]
    }
   ],
   "source": [
    "print resp"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 27,
   "metadata": {
    "collapsed": false,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "target_pose = dict(zip(resp.joints[0].name, resp.joints[0].position))"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 28,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": [
    "limb.set_joint_positions(target_pose)"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": null,
   "metadata": {
    "collapsed": true,
    "deletable": true,
    "editable": true
   },
   "outputs": [],
   "source": []
  },
  {
   "cell_type": "code",
   "execution_count": 29,
   "metadata": {
    "collapsed": true
   },
   "outputs": [],
   "source": [
    "# Euler Angle to Quaternion Example"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 32,
   "metadata": {
    "collapsed": false
   },
   "outputs": [],
   "source": [
    "import tf"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 33,
   "metadata": {
    "collapsed": true
   },
   "outputs": [],
   "source": [
    "zero_angle_quaternion= tf.transformations.quaternion_from_euler(0,0,0)"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 34,
   "metadata": {
    "collapsed": false
   },
   "outputs": [
    {
     "name": "stdout",
     "output_type": "stream",
     "text": [
      "[0. 0. 0. 1.]\n"
     ]
    }
   ],
   "source": [
    "print zero_angle_quaternion"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": null,
   "metadata": {
    "collapsed": true
   },
   "outputs": [],
   "source": []
  }
 ],
 "metadata": {
  "kernelspec": {
   "display_name": "Python 2",
   "language": "python",
   "name": "python2"
  },
  "language_info": {
   "codemirror_mode": {
    "name": "ipython",
    "version": 2
   },
   "file_extension": ".py",
   "mimetype": "text/x-python",
   "name": "python",
   "nbconvert_exporter": "python",
   "pygments_lexer": "ipython2",
   "version": "2.7.12"
  }
 },
 "nbformat": 4,
 "nbformat_minor": 2
}
