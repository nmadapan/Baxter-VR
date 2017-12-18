#!/usr/bin/env python

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from baxter_core_msgs.msg import EndpointState
import struct
import time

def ik_solver_move(limb, x_pos, y_pos, z_pos):
    ns = "ExternalTools/"+ limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    limb = baxter_interface.Limb(limb)

    poses_ex = PoseStamped(
        header=hdr,

        )

    poses = PoseStamped(
        header=hdr,
        pose=Pose(
            position=Point(
                x=x_pos,
                y=y_pos,
                z=z_pos,
            ),
            orientation=Quaternion(
                x=0,
                y=1,
                z=-0.02278316070786928,
                w=0.01182456830798754,
            ),
        ),
    )

    ikreq.pose_stamp.append(poses)

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[-1] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[-1], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        print 'resp.joints[-1]: ', resp.joints[-1]

        limb_joints = dict(zip(resp.joints[-1].name, resp.joints[-1].position))

        limb.move_to_joint_positions(limb_joints)
        print 'Target reached: %.3f %.3f %.3f' % (x_pos, y_pos, z_pos)
        
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    return 0

count = 0
PRINT_EVERY = 300

def get_ee_state(data):
    global count
    count += 1
    if count == PRINT_EVERY:
        print 'EE Position: '
        print data.pose.position
        print 'EE Orientation: '
        print data.pose.orientation
        print '----------'
        count = 0
    # time.sleep(5)

rospy.init_node('example_run_baxter')

# ik_solver_move('left', 0.5, 0.0, 0.1)
# ik_solver_move('left', 0.55, 0.0, 0.1)
# ik_solver_move('left', 0.65, 0.0, 0.1)
# ik_solver_move('left', 0.7, 0.0, 0.1)
# ik_solver_move('left', 0.75, 0.0, 0.1)
# ik_solver_move('left', 0.75, 0.1, 0.1)
# ik_solver_move('left', 0.75, 0.2, 0.1)
# ik_solver_move('left', 0.75, 0.3, 0.1)
# ik_solver_move('left', 0.75, 0.4, 0.1)




# ik_solver_move('left', 0.5569924760761431, 0.3775405087920894, 0.5030001395469457)

eesub = rospy.Subscriber('/robot/limb/left/endpoint_state', EndpointState, get_ee_state)
rospy.spin()