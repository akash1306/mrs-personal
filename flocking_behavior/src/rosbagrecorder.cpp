#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <string>
#include "MathUtils.h"
#include <vector>


#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/transformer.h>

#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/Float64Stamped.h>

#include <nav_msgs/Odometry.h>
#include <iostream>
#include <geometry_msgs/PointStamped.h>

#include <std_srvs/Trigger.h>

#include <math.h>
#include <ctime> 
#include <sys/time.h>
#include <map>
#include <mutex>
#include <chrono>
#include <fstream>
using std::ofstream;
#include <cstdlib> // for exit function
#include <MathUtils.h>


using std::cout; using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;
using std::cerr;

class RosbagRecorderClass
{
    public:
        RosbagRecorderClass(ros::NodeHandle* nodehandle);
        float heading;
        float time_header;
        void FileWriter();
        ofstream outdata;

        private:
            ros::Subscriber sub_heading;
            ros::NodeHandle nh;
            void Callback(const nav_msgs::Odometry::ConstPtr& odom);

};

RosbagRecorderClass::RosbagRecorderClass(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    sub_heading = nh.subscribe("/uav5/odometry/odom_main", 100, 
                                &RosbagRecorderClass::Callback,this);
}

void RosbagRecorderClass::FileWriter(){

    outdata << time_header << "," << heading << endl;
    cout << heading << endl;
}

void RosbagRecorderClass::Callback(const nav_msgs::Odometry::ConstPtr& odom){

    heading = mrs_lib::AttitudeConverter(odom->pose.pose.orientation).getHeading();
    time_header = odom->header.stamp.sec + 
                                        (odom->header.stamp.nsec)/1000000000.0;                        
}

int main(int argc, char** argv) 
{
    
    ros::init(argc, argv, "RosbagRecorder");
    ros::NodeHandle nh;

    ROS_INFO("Main:: Instantiating an object of type UAVClass");
    RosbagRecorderClass RosbagObject(&nh);

    ros::Rate r(10);
    RosbagObject.outdata.open("UAV5Data.txt"); 
    if( !RosbagObject.outdata ) { // file couldn't be opened
      cerr << "Error: file could not be opened" << endl;
      exit(1);
   }
    while (ros::ok())
    {
        RosbagObject.FileWriter();
        ros::spinOnce();
        r.sleep();
    }
    RosbagObject.outdata.close();
    ros::spin();

    return 0;
}