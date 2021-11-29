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


using std::cout; using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;
using std::cerr;

class PsiMetricClass
{
    public:
        PsiMetricClass(ros::NodeHandle* nodehandle);
        float headingCosSum;
        float headingSinSum;
        float psiValue;
        std::vector<float> headerTime;
        time_t currentTime;
        void CalculateMetric();
        std::vector<std::string> _uav_names_;
        std::vector<float> uav_heading;
        ofstream outdata;


    private:
        std::vector<ros::Subscriber>    sub_heading;
        ros::NodeHandle nh_;
        void CallbackNeighborsUsingGPSOdom(const nav_msgs::Odometry::ConstPtr& 
                                        odom, const unsigned int uav_id);



};
PsiMetricClass::PsiMetricClass(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    currentTime = duration_cast<milliseconds>(
                    system_clock::now().time_since_epoch()).count();

    _uav_names_ = {"uav1", "uav2", "uav3", "uav4", "uav5"};

    

    for (unsigned int i = 0; i < _uav_names_.size(); i++){

    /* generate UAV index using UAV name */
    unsigned int uav_id = std::stoi(_uav_names_[i].substr(3));

    /* subscribe to slow_odom */
    sub_heading.push_back( nh_.subscribe<nav_msgs::Odometry>("/" + 
                              _uav_names_[i] + "/odometry/odom_main", 1,
                              boost::bind(
                              &PsiMetricClass::CallbackNeighborsUsingGPSOdom, 
                              this, _1, uav_id)));


  }
}

void PsiMetricClass::CalculateMetric(){
    psiValue = 0;
    headingCosSum= 0;
    headingSinSum = 0;
    time_t timeCalc = duration_cast<milliseconds>(
                        system_clock::now().time_since_epoch()).count();;
    float toPrint = (timeCalc-currentTime)/1000.0;
    for(int i=0; i<uav_heading.size(); i++){
        
        headingCosSum += cos(uav_heading[i]);
        headingSinSum += sin(uav_heading[i]);
        
    }
    psiValue = sqrt(pow(headingCosSum, 2)+ pow(headingSinSum,2))/(float)_uav_names_.size();
    std::cout<<psiValue<<std::endl;
    outdata<<toPrint << "," << psiValue <<std::endl;
    uav_heading.clear();


}

int main(int argc, char** argv){
    ros::init(argc, argv, "PhyMetric"); 
    ros::NodeHandle nh;
     ROS_INFO("main: instantiating an object of type PsiMetricClass");

     PsiMetricClass PsyObject(&nh);
     ROS_INFO("Entering Object");
     ros::Rate r(10);
     PsyObject.outdata.open("3DPlotdata.txt"); 
    if( !PsyObject.outdata ) { // file couldn't be opened
      cerr << "Error: file could not be opened" << endl;
      exit(1);
   }
     while (ros::ok())
     {
            ROS_INFO("Entered While");
            PsyObject.CalculateMetric();
            ros::spinOnce();
            r.sleep();
     }
      PsyObject.outdata.close();
     ros::spin();
     return 0;
}


void PsiMetricClass::CallbackNeighborsUsingGPSOdom
                                        (const nav_msgs::Odometry::ConstPtr& 
                                        odom, const unsigned int uav_id){

        uav_heading.emplace_back(mrs_lib::AttitudeConverter
                                    (odom->pose.pose.orientation).getHeading());
        headerTime.emplace_back(odom->header.stamp.sec + 
                                        (odom->header.stamp.nsec)/1000000000.0);                        
}
