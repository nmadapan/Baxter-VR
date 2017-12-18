#!/usr/bin/env python

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def callback(img_info):
    try:
        cv_image = bridge.imgmsg_to_cv2(img_info, 'bgr8')
    except CvBridgeError as err:
        print(err)
    (height, width, channels) = cv_image.shape
    cv.imshow('Image', cv_image)
    cv.waitKey(3)

rospy.init_node('hello_baxter')

# rs = baxter_interface.RobotEnable(CHECK_VERSION)
# rs.enable()

# limb = baxter_interface.Limb('right')
# print limb.joint_angles()
# print limb.joint_names()
# print limb.joint_angle('right_s0')
# print 'endpoint_pose: ', limb.endpoint_pose()
# print 'endpoint_vel: ', limb.endpoint_velocity()
# print 'endpoint_effort: ', limb.endpoint_effort()

# rs.disable()

## Camera

cam = baxter_interface.CameraController('right_hand_camera')
# cam.close()
cam.open()
rospy.Subscriber('/cameras/right_hand_camera/image', Image, callback)
rospy.spin()