#!/usr/bin/env python3

# Landmarks

# 0. nose                   17. left_pinky
# 1. left_eye_inner         18. right_pinky
# 2. left_eve               19. left_index
# 3. left_eye_outer         20. right_index
# 4. right_eye_inner        21. left_thumb
# 5. right_eye              22. right_thumb
# 6. right_eye_outer        23. left_hip
# 7. left_ear               24. right_hip
# 8. right_ear              25. left_knee
# 9. mouth_left             26. right_knee
# 10. mouth_right           27. left_ankle
# 11. left_shoulder         28. right_ankle
# 12. right_shoulder        29. left_heel
# 13. left_elbow            30. right_heel
# 14. right_elbow           31. left_foot index
# 15. left_wrist            32. right_foot_index
# 16. right_wrist




import rospy
import array
from mrs_msgs.msg import ControlManagerDiagnostics
from mrs_msgs.msg import Float64Stamped
from mrs_msgs.msg import VelocityReferenceStamped
from mrs_msgs.msg import ReferenceStamped
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from misc.msg import floatarray
import cv2
import mediapipe as mp
import math
import numpy as np
import time
import os
import sys
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage, Image

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose

from cv_bridge import CvBridge

class PoseDetectorClass(object):
    
    def __init__(self):
        self.landmarkpub = rospy.Publisher('/landmarkCoord' , floatarray, queue_size=10)
        self.gestuyrepub = rospy.Publisher('/gesture' , Int16, queue_size=10)
        self.image_pub = rospy.Publisher("/output/image_raw", Image, queue_size=10)
        self.subscriber = rospy.Subscriber("/uav7/cam_out", Image, self.callback,  queue_size = 1)
        self.landmarkcoords = floatarray()
        self.landmarkcoords.x = array.array('f',(0 for f in range(0,33)))
        self.landmarkcoords.y = array.array('f',(0 for f in range(0,33)))
        self.landmarkcoords.vis = array.array('f',(0 for f in range(0,33)))
        


    def callback(self, ros_data : Image):
        br = CvBridge()
        # np_arr = np.fromstring(ros_data.data, np.uint8)
        image = br.imgmsg_to_cv2(ros_data)
        print(image[0,0,:])
        # self.image_pub.publish(ros_data)
        # cv2.imshow("Lol", image)
       
        with mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as pose:

            
                # success, image = cap.read()
                # if not success:
                #     print("Ignoring empty camera frame.")
                #     # If loading a video, use 'break' instead of 'continue'.
                #     continue

                # To improve performance, optionally mark the image as not writeable to
                # pass by reference.
                # image.flags.writeable = False
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                
                results = pose.process(image)

                # Draw the pose annotation on the image.
                # image.flags.writeable = True
                
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                
                
                mp_drawing.draw_landmarks(
                    image,
                    results.pose_landmarks,
                    mp_pose.POSE_CONNECTIONS,
                    landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                to_ros = br.cv2_to_imgmsg(image)
                to_ros.encoding = "rgb8"
                self.image_pub.publish(to_ros)
                # Flip the image horizontally for a selfie-view display.
                # cv2.imshow('MediaPipe Pose', cv2.flip(image, 1))

                
                
                i=0
                for landname in mp_pose.PoseLandmark:
                    
                    self.landmarkcoords.x[i] = results.pose_landmarks.landmark[landname].x 
                    self.landmarkcoords.y[i] = results.pose_landmarks.landmark[landname].y
                    self.landmarkcoords.vis[i] = results.pose_landmarks.landmark[landname].visibility 
                    i+=1


                print(self.landmarkcoords)


                # #Calculate angle of Right hand with the shoulder line
                # angleRightHandShoulder = self.calculateAngle(self.landmarkcoords.x[16], 
                # self.landmarkcoords.y[16], self.landmarkcoords.x[12], self.landmarkcoords.y[12],
                # self.landmarkcoords.x[11], self.landmarkcoords.y[11])

                # #Calculate angle of Left hand with the shoulder line
                # angleLeftHandShoulder = self.calculateAngle(self.landmarkcoords.x[15], 
                # self.landmarkcoords.y[15], self.landmarkcoords.x[11], self.landmarkcoords.y[11],
                # self.landmarkcoords.x[12], self.landmarkcoords.y[12])

                # # print("Left: ")
                # # print(angleLeftHandShoulder)
                # # print("Right: ")
                # # print(angleRightHandShoulder)

                # if angleRightHandShoulder >155 and angleRightHandShoulder < 175 and angleLeftHandShoulder >95 and angleLeftHandShoulder< 115:
                #     print("Go Right")


                
        # cap.release()


        


    def servicestarter(self):
        rospy.wait_for_service("/uav1/control_manager/velocity_reference")

        try:
            service1 = rospy.ServiceProxy("/uav1/control_manager/velocity_reference", VelocityReferenceStamped)
            #resp1 = service1(self.landmarkcoords)

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def calculateAngle(self, firstPointx, firstPointy, midPointx, midPointy, endPointx, endPointy):

        landmarkAngle = math.degrees(math.atan2(endPointy - midPointy, endPointx - midPointx) - math.atan2(firstPointy - midPointy , firstPointx - midPointx))
        landmarkAngle = abs(landmarkAngle)

        if landmarkAngle >180:
            landmarkAngle = 360 - landmarkAngle


        return landmarkAngle

        


    def calculateLandmarks(self):

        # For webcam input:
        cap = cv2.VideoCapture(0)
        # rate = rospy.Rate(50)


        with mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as pose:

            while cap.isOpened() and not rospy.is_shutdown():
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
                
                i=0
                for landname in mp_pose.PoseLandmark:
                    
                    self.landmarkcoords.x[i] = results.pose_landmarks.landmark[landname].x 
                    self.landmarkcoords.y[i] = results.pose_landmarks.landmark[landname].y
                    self.landmarkcoords.vis[i] = results.pose_landmarks.landmark[landname].visibility 
                    i+=1


                self.landmarkpub.publish(self.landmarkcoords)


                #Calculate angle of Right hand with the shoulder line
                angleRightHandShoulder = self.calculateAngle(self.landmarkcoords.x[16], 
                self.landmarkcoords.y[16], self.landmarkcoords.x[12], self.landmarkcoords.y[12],
                self.landmarkcoords.x[11], self.landmarkcoords.y[11])

                #Calculate angle of Left hand with the shoulder line
                angleLeftHandShoulder = self.calculateAngle(self.landmarkcoords.x[15], 
                self.landmarkcoords.y[15], self.landmarkcoords.x[11], self.landmarkcoords.y[11],
                self.landmarkcoords.x[12], self.landmarkcoords.y[12])

                # print("Left: ")
                # print(angleLeftHandShoulder)
                # print("Right: ")
                # print(angleRightHandShoulder)

                if angleRightHandShoulder >155 and angleRightHandShoulder < 175 and angleLeftHandShoulder >95 and angleLeftHandShoulder< 115:
                    print("Go Right")


                if cv2.waitKey(5) & 0xFF == 27:
                    break

                # rate.sleep()
        cap.release()

    


def main():
    rospy.init_node('PoseFormation', anonymous= True)
    rate = rospy.Rate(50)
    flypos = PoseDetectorClass()

    #rospy.sleep(1)
    flypos.calculateLandmarks()

    rospy.spin()

if __name__ == '__main__':
    main()


