// Landmarks

// 0. nose                   17. left_pinky
// 1. left_eye_inner         18. right_pinky
// 2. left_eve               19. left_index
// 3. left_eye_outer         20. right_index
// 4. right_eye_inner        21. left_thumb
// 5. right_eye              22. right_thumb
// 6. right_eye_outer        23. left_hip
// 7. left_ear               24. right_hip
// 8. right_ear              25. left_knee
// 9. mouth_left             26. right_knee
// 10. mouth_right           27. left_ankle
// 11. left_shoulder         28. right_ankle
// 12. right_shoulder        29. left_heel
// 13. left_elbow            30. right_heel
// 14. right_elbow           31. left_foot index
// 15. left_wrist            32. right_foot_index
// 16. right_wrist

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <math.h>

class flyingPoseClass
{
    public:
        flyingPoseClass(ros::NodeHandle* nodehandle);
        void calculateAngle();

    private:
        int i;
        void initializeSubscribers();
        void subscriberCallback(const nav_msgs::Odometry& message_holder);
        




};

int main()
{
    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
}