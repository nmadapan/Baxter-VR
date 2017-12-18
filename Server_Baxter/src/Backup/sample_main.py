#!/usr/bin/env python

## General
import socket, time, struct
import cv2 as cv
import numpy as np
from copy import deepcopy

# ROS
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

# Baxter
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from baxter_core_msgs.msg import EndpointState

## Global Static Variables
TCP_IP = '128.46.125.46'
TCP_PORT = 5000
BUFFER_SIZE = 1024
MAX_CLIENTS = 1
INITIAL_MESSAGE = 'Handshake'
TOUCHPAD_ID = 4294967296
TRIGGER_ID = 8589934592
R_left = np.array([[1,0,0],[0,0,-1],[0,1,0]])
R_right = np.array([[1,0,0],[0,0,-1],[0,1,0]])

class Baxter():
    def __init__(self, limb_name):
        ## Initializing the hand
        self.ikreq = SolvePositionIKRequest()
        self.limb_name = limb_name
        self.limb_inst = baxter_interface.Limb(self.limb_name)
        self.ns = "ExternalTools/"+ self.limb_name + "/PositionKinematicsNode/IKService"
        self.iksvc = rospy.ServiceProxy(self.ns, SolvePositionIK)
        self.curr_bax_pose = {'position':None, 'orientation':None}        
        self.eesub = rospy.Subscriber('/robot/limb/'+self.limb_name+'/endpoint_state', EndpointState, self.update_ee_state)
        self.gripper = baxter_interface.Gripper(self.limb_name, CHECK_VERSION)
        self.gripper.calibrate()
        time.sleep(1)


    def update_ee_state(self, data):
        self.curr_bax_pose['position'] = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        self.curr_bax_pose['orientation'] = [data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z]
        # print self.limb_name, self.curr_bax_pose

    def ik_solver_move(self, position=[], orientation=[0.0118, 0,1,-0.0227]):
        x_pos, y_pos, z_pos = tuple(position)
        w_quat, x_quat, y_quat, z_quat = tuple(orientation)
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        poses = PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=x_pos,
                    y=y_pos,
                    z=z_pos,
                ),
                orientation=Quaternion(
                    x=x_quat,
                    y=y_quat,
                    z=z_quat,
                    w=w_quat,
                ),
            ),
        )

        self.ikreq.pose_stamp.append(poses)

        try:
            rospy.wait_for_service(self.ns, 5.0)
            resp = self.iksvc(self.ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1

        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                                   resp.result_type)
        if (resp_seeds[-1] != resp.RESULT_INVALID):
            seed_str = {
                        self.ikreq.SEED_USER: 'User Provided Seed',
                        self.ikreq.SEED_CURRENT: 'Current Joint Angles',
                        self.ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[-1], 'None')
            print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                  (seed_str,))
            # Format solution into Limb API-compatible dictionary
            print 'resp.joints[-1]: ', resp.joints[-1]

            limb_joints = dict(zip(resp.joints[-1].name, resp.joints[-1].position))

            self.limb_inst.move_to_joint_positions(limb_joints)
            print 'Target reached: %.3f %.3f %.3f' % (x_pos, y_pos, z_pos)
            
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")

        return 0

    def increment_ee_position(self, dposition):
        self.ik_solver_move( (np.array(dposition) + np.array(self.curr_bax_pose['position'])).tolist() )


rospy.init_node('baxter_controller')
bax_left = Baxter('left')
# bax_right = Baxter('right')
# # while not rospy.is_shutdown():
# #     # print 'Left: ', bax_left.curr_bax_pose
# #     print 'Right: ', bax_right.curr_bax_pose
# #     time.sleep(0.5)

# bax_left.increment_ee_position([0.05,-0.05,0.0])

print bax_left.gripper.open()
print bax_left.gripper.parameters()