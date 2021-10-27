#!/usr/bin/env python3
import rospy
import array
from mrs_msgs.msg import ControlManagerDiagnostics
from mrs_msgs.msg import Float64Stamped
from mrs_msgs import VelocityReferenceStamped
from mrs_msgs.msg import ReferenceStamped
from std_msgs.msg import Float64
from 
import math
import numpy as np
import time
import os

class flyingPoseClass(object):
    def __init__(self):
        self.landmarkpub = rospy.Publisher('/landmarkCoord' , array)

        self.landmarkcoords = np.empty(shape(3,33), dtype = 'float32')

    def servicestarter(self):
        rospy.wait_for_service("/uav1/control_manager/velocity_reference")

        try:
            service1 = rospy.ServiceProxy("/uav1/control_manager/velocity_reference", VelocityReferenceStamped)
            resp1 = service1(self.landmarkcoords)

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def calculateLandmarks(self):
        


