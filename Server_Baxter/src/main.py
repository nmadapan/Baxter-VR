#!/usr/bin/env python

## General
import socket, time, struct
import cv2 as cv
import numpy as np
from copy import deepcopy
from threading import Thread
from Queue import Queue

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
# Natalia's computer
# R_left = np.array([[-1,0,0],[0,0,1],[0,1,0]])
# R_right = np.array([[1,0,0],[0,0,-1],[0,1,0]])

# Rahul's computer
R_left = np.array([[1,0,0],[0,0,-1],[0,1,0]])

MAX_QUEUE_SIZE = 1028
QUEUE_PUSH_RATE = 10

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
        self.gripper_state = True # True - Open, False - Close
        time.sleep(1)

    def update_ee_state(self, data):
        self.curr_bax_pose['position'] = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        self.curr_bax_pose['orientation'] = [data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z]
        # print self.limb_name, self.curr_bax_pose

    def ik_solver_move(self, position=[], orientation=[]):
        x_pos, y_pos, z_pos = tuple(position)
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
                    x=0,
                    y=1,
                    z=-0.02278316070786928,
                    w=0.01182456830798754,
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

    def toggle_gripper_status(self):
        if self.gripper_state:
            self.gripper.close()
        else:
            self.gripper.open()
        self.gripper_state = not self.gripper_state

    def increment_ee_position(self, dposition):
        self.ik_solver_move( (np.array(dposition) + np.array(self.curr_bax_pose['position'])).tolist() )

class Server():
    def __init__(self):
        ## Socket Initialization
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # Opening the socket
        self.sock.bind((TCP_IP, TCP_PORT))
        self.sock.listen(MAX_CLIENTS)
        self.connect_status = False # Connection not yet established.

        ## Controller tracking variables
        # The controller pose just before sending move signal to baxter.
        self.baxter_previous_controller_pose = {'left':{'position':None, 'orientation':None}, 'right':{'position':None, 'orientation':None}}
        # reference pose is the controller pose when the trigger is clicked
        self.left_reference_controller_pose = {'left':{'position':None, 'orientation':None}, 'right':{'position':None, 'orientation':None}}
        self.right_reference_controller_pose = {'left':{'position':None, 'orientation':None}, 'right':{'position':None, 'orientation':None}}
        # Current controller pose
        self.controller_pose = {'left':{'position':None, 'orientation':None}, 'right':{'position':None, 'orientation':None}}
        # Current controller status
        self.controller_status = {'left':{'pad_click':False, 'pad_press':False, 'trig_click':False, 'trig_press':False}, \
                                 'right':{'pad_click':False, 'pad_press':False, 'trig_click':False, 'trig_press':False}}
        # Previous controller pose
        self.previous_controller_pose = {'left':{'position':None, 'orientation':None}, 'right':{'position':None, 'orientation':None}}
        # Previous controller status
        self.previous_controller_status = {'left':{'pad_click':False, 'pad_press':False, 'trig_click':False, 'trig_press':False}, \
                                 'right':{'pad_click':False, 'pad_press':False, 'trig_click':False, 'trig_press':False}}

        ## Baxter Initialization: Left and right are separately initialized.
        rospy.init_node('baxter_controller')
        self.bax_left = Baxter('left')
        self.bax_right = Baxter('right')
        # Queue of dpositions for left and right arms separately.
        self.left_queue_dposition = Queue(maxsize=MAX_QUEUE_SIZE)
        self.right_queue_dposition = Queue(maxsize=MAX_QUEUE_SIZE)
        # Current baxter dposition
        self.baxter_dposition = {'left':[0.0,0.0,0.0], 'right':[0.0,0.0,0.0]}

        ## Miscellaneous
        self.count = 0

        ## Waiting for establishing the connection between server and client
        self.wait_for_connection()

    def wait_for_connection(self):
        self.client, self.addr = (self.sock.accept())
        data = self.client.recv(BUFFER_SIZE)
        if data == INITIAL_MESSAGE:
            print 'Received a handshake'
            self.connect_status = True
            self.client.send(str(True)) # Send a success note

    def parse_data(self, data):
        # 0,1,2 - left x,y,z
        # 3,4,5 - left yaw, pitch, roll
        # 6,7,8 - flagPress, flagTouch, serial_no
        # 9, 10, 11 - right x, y, z
        # 12, 13, 14 - right yaw, pitch, roll
        # 15, 16, 17 - flagPress, flagTouch, serial_no
        data = map( float, data.split(' ') )

        # Extracting left controller data
        left_position = data[0:3]; left_orientation = data[3:6]; left_state = data[6:9]
        left_data = {'position':left_position, 'orientation':left_orientation, 'state':left_state}

        # Extracting right controller data
        right_position = data[9:12]; right_orientation = data[12:15]; right_state = data[15:18]
        right_data = {'position':right_position, 'orientation':right_orientation, 'state':right_state}

        return left_data, right_data

    def update_controller_info(self, data):
        left_data, right_data = data

        # Update left controller pose
        self.controller_pose['left']['position'] = left_data['position']
        self.controller_pose['left']['orientation'] = left_data['orientation']

        # Update right controller pose
        self.controller_pose['right']['position'] = right_data['position']
        self.controller_pose['right']['orientation'] = right_data['orientation']

        # 0 for CLICK and 1 for PRESS
        # Update left controller status
        if int(left_data['state'][0]) == TOUCHPAD_ID:   self.controller_status['left']['pad_click'] = True
        else:                                           self.controller_status['left']['pad_click'] = False
        if int(left_data['state'][1]) == TOUCHPAD_ID:   self.controller_status['left']['pad_press'] = True
        else:                                           self.controller_status['left']['pad_press'] = False
        if int(left_data['state'][0]) == TRIGGER_ID:   self.controller_status['left']['trig_click'] = True
        else:                                           self.controller_status['left']['trig_click'] = False
        if int(left_data['state'][1]) == TRIGGER_ID:   self.controller_status['left']['trig_press'] = True
        else:                                           self.controller_status['left']['trig_press'] = False

        # Update both the controller status
        if int(left_data['state'][0]) == TOUCHPAD_ID+TRIGGER_ID:
            self.controller_status['left']['pad_click'] = True
            self.controller_status['left']['trig_click'] = True
        if int(left_data['state'][1]) == TOUCHPAD_ID+TRIGGER_ID:
           self.controller_status['left']['pad_press'] = True
           self.controller_status['left']['trig_press'] = True


        # Update right controller status
        if int(right_data['state'][0]) == TOUCHPAD_ID:   self.controller_status['right']['pad_click'] = True
        else:                                           self.controller_status['right']['pad_click'] = False
        if int(right_data['state'][1]) == TOUCHPAD_ID:   self.controller_status['right']['pad_press'] = True
        else:                                           self.controller_status['right']['pad_press'] = False
        if int(right_data['state'][0]) == TRIGGER_ID:   self.controller_status['right']['trig_click'] = True
        else:                                           self.controller_status['right']['trig_click'] = False
        if int(right_data['state'][1]) == TRIGGER_ID:   self.controller_status['right']['trig_press'] = True
        else:                                           self.controller_status['right']['trig_press'] = False

        # Update both the controller status
        if int(right_data['state'][0]) == TOUCHPAD_ID+TRIGGER_ID:
            self.controller_status['right']['pad_click'] = True
            self.controller_status['right']['trig_click'] = True
        if int(right_data['state'][1]) == TOUCHPAD_ID+TRIGGER_ID:
           self.controller_status['right']['pad_press'] = True
           self.controller_status['right']['trig_press'] = True

    def generate_baxter_data(self):
        if self.controller_status['left']['trig_click']:
            self.baxter_dposition['left'] = np.array(self.controller_pose['left']['position']) - \
                                            np.array(self.previous_controller_pose['left']['position'])
            self.baxter_dposition['left'] = np.dot(R_left, self.baxter_dposition['left'].reshape(3,1)).flatten().copy().tolist()

        if self.controller_status['right']['trig_click']:
            self.baxter_dposition['right'] = np.array(self.controller_pose['right']['position']) - \
                                             np.array(self.previous_controller_pose['right']['position'])
            self.baxter_dposition['right'] = np.dot(R_right, self.baxter_dposition['right'].reshape(3,1)).flatten().tolist()

    def print_data(self):
        print 'Data Received: Yayy'
        print 'Left Controller: '
        print 'Position: ', self.controller_pose['left']['position'], 'Orientation: ', self.controller_pose['left']['orientation'], 'Status: ', self.controller_status['left']
        # print 'Right Controller: '
        # print 'Position: ', self.controller_pose['right']['position'], 'Orientation: ', self.controller_pose['right']['orientation'], 'Status: ', self.controller_status['right']

    def run_left_controller(self):
        START_COUND_IDX = 1
        while self.connect_status and (not rospy.is_shutdown()):
            try:
                # Receive VIVE Controller information
                data = self.client.recv(BUFFER_SIZE)
                self.client.send(str(True)) # Send a success note

                # Parse and update controller information
                self.update_controller_info(self.parse_data(data))
                # self.print_data() # Prints current controller data for both controllers.

                #### For first reading from vive: Update previous controller pose and continue
                if self.count == 0: # Initially previous and current poses are the same
                    self.previous_controller_pose = deepcopy(self.controller_pose)
                    self.previous_controller_status = deepcopy(self.controller_status)
                    self.count = START_COUND_IDX
                    continue

                ### --- From here it is left controller specific.  --- ###
                #### From second VIVE reading onwards
                # Continue if previous and current states of trigger click are False
                if (not self.previous_controller_status['left']['trig_click']) and (not self.controller_status['left']['trig_click']):
                    self.previous_controller_status = deepcopy(self.controller_status)
                    self.previous_controller_pose = deepcopy(self.controller_pose)
                    continue

                print self.previous_controller_status['left']['trig_click'], self.controller_status['left']['trig_click']

                # Transition from False to True is marked as initial point for motion: Left Controller
                # Consider that point as a reference
                if (not self.previous_controller_status['left']['trig_click']) and self.controller_status['left']['trig_click']:
                    self.left_reference_controller_pose = deepcopy(self.controller_pose['left'])
                    self.baxter_previous_controller_pose = deepcopy(self.controller_pose) # It stores for both controllers.
                    self.previous_controller_status = deepcopy(self.controller_status)
                    self.previous_controller_pose = deepcopy(self.controller_pose)
                    self.count = START_COUND_IDX
                    print 'left_reference_controller_pose', self.left_reference_controller_pose
                    continue

                # trigger is clicked for both previous and current states.
                if self.previous_controller_status['left']['trig_click'] and self.controller_status['left']['trig_click']:
                    self.generate_baxter_data()
                    # print 'baxter_dposition[left]: ', self.baxter_dposition['left']
                    if np.mean(np.abs(np.array(self.baxter_dposition['left']))) > 1e-3:
                        print 'baxter_dposition[left]: ', self.baxter_dposition['left']
                        self.left_queue_dposition.put(deepcopy(self.baxter_dposition['left']))
                    if self.previous_controller_status['left']['pad_click'] != self.controller_status['left']['pad_click']:
                        self.bax_left.toggle_gripper_status()
                    self.count += 1

                self.previous_controller_status = deepcopy(self.controller_status)
                self.previous_controller_pose = deepcopy(self.controller_pose)

            except Exception as exp:
                print exp
                print 'Connection Closed'
                self.connect_status = False
                self.client.close()

        self.connect_status = False

    def increment_baxter_left_beta(self): ### Updating prevous position in here.
    # Emptying half of the queue at once. Hence Baxter moves quickly as distance is longer.
        while self.connect_status and (not rospy.is_shutdown()):
            sum_dposition = np.zeros(3)
            try:
                if self.left_queue_dposition.qsize() < 1:
                    print 1/0 # Intentionally creating the exception
                for _ in range(self.left_queue_dposition.qsize()/1):
                    sum_dposition += np.array(self.left_queue_dposition.get(block=False))
                self.bax_left.increment_ee_position(sum_dposition.tolist())
            except Exception as exp:
                pass
                # print 'Queue is empty ... ^^'
            # time.sleep(0.001)

            ### Updating the previous controller pose ###
            self.baxter_previous_controller_pose = deepcopy(self.controller_pose)

    def increment_baxter_left(self): ### Updating prevous position in here.
    # Emptying the queue one at a time. It is slower.
        while self.connect_status and (not rospy.is_shutdown()):
            try:
                print 'queue_size: ', self.left_queue_dposition.qsize()
                self.bax_left.increment_ee_position(self.left_queue_dposition.get(block=False))
            except Exception as exp:
                pass
                # print 'Queue is empty ... ^^'
            # time.sleep(0.001)

            ### Updating the previous controller pose ###
            self.baxter_previous_controller_pose = deepcopy(self.controller_pose)

    def increment_baxter_right(self):
        while self.connect_status and (not rospy.is_shutdown()) and (not self.right_queue_dposition.empty()):
            ### Updating the previous controller pose ###
            self.bax_right.increment_ee_position(self.right_queue_dposition.get())

print '--------- Server ---------'
server = Server()
server_thread = Thread(name='server_thread', target=server.run_left_controller)
bax_left_thread = Thread(name='baxter_left_hand', target=server.increment_baxter_left_beta)

server_thread.start()
bax_left_thread.start()
rospy.spin()

# server_thread.join()
# bax_left_thread.join()

# bax_right_thread = Thread(name='baxter_right_hand', target=server.increment_baxter_right)
# bax_right_thread.start()
# bax_right_thread.join()
