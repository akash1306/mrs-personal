#ifndef SENSOR_NEIGHBOR_H
#define SENSOR_NEIGHBOR_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/transformer.h>

#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/Float64Stamped.h>

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PointStamped.h>

#include <std_srvs/Trigger.h>

#include <math.h>

#include <map>
#include <mutex>

#include<gesture_flocking/Neighbor.h>>



class neighbor_class
{
public:
    neighbor_class(ros::NodeHandle* nodehandle);

private:
    bool is_initialized_, has_this_pose_, has_started_swarming_mode_, last_message_invalid_;

    std::string _uav_name_;

    
};

#endif
