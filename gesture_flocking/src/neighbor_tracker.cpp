#include<neighbor_tracker.hpp>



neighbor_class::neighbor_class(ros::NodeHandle* nodehandle):nh_(*nodehandle){
  /* set flags to false */
  is_initialized_            = false;
  has_this_pose_             = false;
  has_started_swarming_mode_ = false;
  last_message_invalid_      = false;

  ROS_INFO("Neighbor Tracking Node: Initialized");
  
}
