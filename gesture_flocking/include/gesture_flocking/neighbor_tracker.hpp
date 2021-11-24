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

#include<gesture_flocking/Neighbor.h>



class NeighborClass
{
public:
    NeighborClass(ros::NodeHandle* nodehandle);
    bool eland_tracker = false;

private:
    ros::NodeHandle                 nh; 
    ros::Subscriber                 this_uav_gps_odom;
    ros::Subscriber                 this_uav_local_height;
    std::vector<ros::Subscriber>    sub_gps_odom_uavs_;
    std::vector<ros::Subscriber>    sub_height_odom_local_uavs_;

    void InitializeSubscribers();

    std::string                 _sensor_type_;
    std::string                 _this_uav_name_;
    std::vector<std::string>    _uav_names_;

    std::map<unsigned int, geometry_msgs::PointStamped> neighbors_position_;

    bool                        _use_fixed_heading_;

    mrs_lib::Transformer                     tfr_;
    std::optional<mrs_lib::TransformStamped> tf_output_;

    
// |-----------------------------------Flags-----------------------------------|
    bool is_initialized_;
    bool has_this_pose_ ;
    bool has_started_swarming_mode_ ;
    bool last_message_invalid_      ;

// |-----------------------------------Subscriber Callbacks-----------------------------------|

    void    ThisGPSCallback(const nav_msgs::Odometry::ConstPtr& odom);
    void    ThisUAVHeight(const mrs_msgs::Float64Stamped::ConstPtr& height);
    void    CallbackNeighborsUsingGPSOdom(const nav_msgs::Odometry::ConstPtr& 
                                        odom, const unsigned int uav_id);
    void    CallbackNeighborsUsingHeightOdomLocal(const 
            mrs_msgs::Float64Stamped::ConstPtr& height, const unsigned int 
            uav_id);

    ros::ServiceClient srv_client_land_;

    void           callbackTimerPubNeighbors(const ros::TimerEvent& event);
    ros::Timer     timer_pub_neighbors_;
    ros::Publisher pub_neighbors_;
  
};

#endif
