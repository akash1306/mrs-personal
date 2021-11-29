import rospy
import array 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from mrs_msgs.srv import Vec4
from mrs_msgs.msg import ControlManagerDiagnostics
from mrs_msgs.msg import Float64Stamped
from mrs_msgs.msg import ReferenceStamped
from std_msgs.msg import Float64
import math
import numpy as np
import time
import os

import pandas as pd

def getClosestTimeIdx(time_list, timestamp):

    # array = np.asarray(time_list)
    idx = np.argmin(np.abs(time_list - timestamp), axis=0)

    return idx

def main():
    print ("Entering Main")
    base_uav = pd.read_csv('/home/akash/workspace/UAV5Data.csv')
    compare_uav4 = pd.read_csv('/home/akash/workspace/UAV4Data.csv')
    compare_uav3 = pd.read_csv('/home/akash/workspace/UAV3Data.csv')
    compare_uav2 = pd.read_csv('/home/akash/workspace/UAV2Data.csv')
    compare_uav1 = pd.read_csv('/home/akash/workspace/UAV1Data.csv')
    f = open("PsyOrderGazebo3D.csv", "a")

    for index, timestamp in enumerate(base_uav['time'].values):
        f.write(str(timestamp))
        f.write(",")
        f.write(str(base_uav['heading'].values[index]))
        f.write(",")
        f.write(str(compare_uav4['heading'].values[getClosestTimeIdx(compare_uav4['time'].values, timestamp)]))
        f.write(",")
        f.write(str(compare_uav3['heading'].values[getClosestTimeIdx(compare_uav3['time'].values, timestamp)]))
        f.write(",")
        f.write(str(compare_uav2['heading'].values[getClosestTimeIdx(compare_uav2['time'].values, timestamp)]))
        f.write(",")
        f.write(str(compare_uav1['heading'].values[getClosestTimeIdx(compare_uav1['time'].values, timestamp)]))
        f.write("\n")


    f.close()

    

if __name__ == '__main__':
    main()
