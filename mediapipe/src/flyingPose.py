#!/usr/bin/env python3
import rospy
import array
from mrs_msgs.msg import ControlManagerDiagnostics
from mrs_msgs.msg import Float64Stamped
from mrs_msgs.msg import VelocityReferenceStamped
from mrs_msgs.msg import ReferenceStamped
from std_msgs.msg import Float64
from misc.msg import floatarray
import cv2
import mediapipe as mp
import math
import numpy as np
import time
import os
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose

class flyingPoseClass(object):
    def __init__(self):
        self.landmarkpub = rospy.Publisher('/landmarkCoord' , Float64, queue_size=10)

        self.landmarkcoords = np.empty((3,33), dtype = 'object')
        self.tempcoord = None

    def servicestarter(self):
        rospy.wait_for_service("/uav1/control_manager/velocity_reference")

        try:
            service1 = rospy.ServiceProxy("/uav1/control_manager/velocity_reference", VelocityReferenceStamped)
            resp1 = service1(self.landmarkcoords)

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def calculateLandmarks(self):

        # For webcam input:
        cap = cv2.VideoCapture(0)
        with mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as pose:
            while cap.isOpened():
                success, image = cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                    # If loading a video, use 'break' instead of 'continue'.
                    continue

                # To improve performance, optionally mark the image as not writeable to
                # pass by reference.
                image.flags.writeable = False
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                results = pose.process(image)

                # Draw the pose annotation on the image.
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                mp_drawing.draw_landmarks(
                    image,
                    results.pose_landmarks,
                    mp_pose.POSE_CONNECTIONS,
                    landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
                # Flip the image horizontally for a selfie-view display.
                cv2.imshow('MediaPipe Pose', cv2.flip(image, 1))
                if not results.pose_landmarks:
                    continue
                self.tempcoord = results.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE].x 
                self.landmarkpub.publish(self.tempcoord)
                # for landname in mp_pose.PoseLandmark:
                #     print(landname)
                #     print(": ")
                #     print (results.pose_landmarks.landmark[landname].visibility)

            # print(
            #     f'Nose coordinates: ('
            #     f'{results.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE].x }, '
            #     f'{results.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE].y })'
            # )
                if cv2.waitKey(5) & 0xFF == 27:
                    break
        cap.release()

    


def main():
    rospy.init_node('PoseFormation', anonymous= True)
    rate = rospy.Rate(50)
    flypos = flyingPoseClass()

    while not rospy.is_shutdown():
        flypos.calculateLandmarks()
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()


