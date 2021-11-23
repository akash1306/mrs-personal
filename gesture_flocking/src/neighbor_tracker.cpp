#include <neighbor_tracker.hpp>



NeighborClass::NeighborClass(ros::NodeHandle* nodehandle):nh(*nodehandle){

  mrs_lib::ParamLoader param_loader(nh, "NeighborTracker");

  /* set flags to false */
  is_initialized_            = false;
  has_this_pose_             = false;
  has_started_swarming_mode_ = false;
  last_message_invalid_      = false;

  //Loading Parameters
  param_loader.loadParam("uav_name", _this_uav_name_);
  param_loader.loadParam("use_fixed_heading", _use_fixed_heading_);

  if (!param_loader.loadedSuccessfully()) {
  ROS_ERROR("[SensorNeighbor]: failed to load non-optional parameters!");
  ros::shutdown();
  }


  InitializeSubscribers();

  ROS_INFO("Neighbor Tracking Node: Initialized");
  
}

void NeighborClass::InitializeSubscribers(){
  this_uav_gps_odom = nh.subscribe<nav_msgs::Odometry>("/" + _this_uav_name_ 
                      + "/odometry/odom_main", 1, 
                      &NeighborClass::ThisGPSCallback, this);

  this_uav_local_height = nh.subscribe<mrs_msgs::Float64Stamped>("/" + 
                          _this_uav_name_ + "/odometry/height", 1, 
                          &NeighborClass::ThisUAVHeight, this);

  

  
    
}

int main(int argc, char** argv){
  ros::init(argc, argv, "NeighborTracker");
  ros::Time::waitForValid();
  ros::NodeHandle nh("~"); 
  ROS_INFO("Main: Instantiating an object of type NeighborClass");
  NeighborClass n_obj(&nh);
  
  
}

// |-----------------------------------Subscriber Callbacks-----------------------------------|
void NeighborClass::ThisGPSCallback(const nav_msgs::Odometry::ConstPtr& odom){
  
}

void NeighborClass::ThisUAVHeight(const mrs_msgs::Float64Stamped::ConstPtr& height){

}