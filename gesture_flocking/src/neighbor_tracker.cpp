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
  param_loader.loadParam("uav_names", _uav_names_);

  if (!param_loader.loadedSuccessfully()) {
  ROS_ERROR("[NeighborTracker]: failed to load non-optional parameters!");
  ros::shutdown();
  }

  /* check if this UAV name is in the list of all UAVs */
  auto itr = std::find(_uav_names_.begin(), _uav_names_.end(), _this_uav_name_);
  if (itr == _uav_names_.cend()) {
    ROS_ERROR("UAV name %s is not in the list of all UAVs.", 
    _this_uav_name_.c_str());
    ros::shutdown();
  }


  InitializeSubscribers();

  /* services */
  srv_client_land_ = nh.serviceClient<std_srvs::Trigger>("/" + _this_uav_name_ + "/uav_manager/land");

  /* publisher */
  pub_neighbors_ = nh.advertise<gesture_flocking::Neighbor>("/" + _this_uav_name_ + "/sensor_neighbor/neighbors", 1);

  /* timers */
  timer_pub_neighbors_ = nh.createTimer(ros::Rate(5.0), &NeighborClass::callbackTimerPubNeighbors, this);

  /* transformer */
  tfr_ = mrs_lib::Transformer("SensorNeighbor", _this_uav_name_);
  

  ROS_INFO("Neighbor Tracking Node: Initialized");
  is_initialized_ = true;
  
}

void NeighborClass::InitializeSubscribers(){
  this_uav_gps_odom = nh.subscribe<nav_msgs::Odometry>("/" + _this_uav_name_ 
                      + "/odometry/odom_main", 1, 
                      &NeighborClass::ThisGPSCallback, this);

  this_uav_local_height = nh.subscribe<mrs_msgs::Float64Stamped>("/" + 
                          _this_uav_name_ + "/odometry/height", 1, 
                          &NeighborClass::ThisUAVHeight, this);

  
  for (unsigned int i = 0; i < _uav_names_.size(); i++){

    if (_uav_names_[i] == _this_uav_name_){
      continue;
    }

    /* generate UAV index using UAV name */
    unsigned int uav_id = std::stoi(_uav_names_[i].substr(3));

    /* subscribe to slow_odom */
    sub_gps_odom_uavs_.push_back( nh.subscribe<nav_msgs::Odometry>("/" + 
                              _uav_names_[i] + "/odometry/slow_odom", 1,
                              boost::bind(
                              &NeighborClass::CallbackNeighborsUsingGPSOdom, 
                              this, _1, uav_id)));

    sub_height_odom_local_uavs_.push_back(nh.subscribe<mrs_msgs::Float64Stamped>(
                                    "/" + _uav_names_[i] + "/odometry/height",
                                     1, boost::bind(&NeighborClass::
                                     CallbackNeighborsUsingHeightOdomLocal, 
                                     this, _1, uav_id)));


  }

  
    
}

int main(int argc, char** argv){
  ros::init(argc, argv, "NeighborTracker");
  ros::Time::waitForValid();
  ros::NodeHandle nh("~"); 
  ROS_INFO("Main: Instantiating an object of type NeighborClass");
  NeighborClass n_obj(&nh);
  return 0;
  
}

// |-----------------------------------Subscriber Callbacks-----------------------------------|
void NeighborClass::ThisGPSCallback(const nav_msgs::Odometry::ConstPtr& odom){
  
}

void NeighborClass::ThisUAVHeight(const mrs_msgs::Float64Stamped::ConstPtr& 
                                  height){

}

void    NeighborClass::CallbackNeighborsUsingGPSOdom(const nav_msgs::Odometry::
                                                      ConstPtr& odom, 
                                                      const unsigned int uav_id)
                                                      {

}

void    NeighborClass::CallbackNeighborsUsingHeightOdomLocal(const 
            mrs_msgs::Float64Stamped::ConstPtr& height, const unsigned int 
            uav_id){

            }

void NeighborClass::callbackTimerPubNeighbors([[maybe_unused]] const 
                                                ros::TimerEvent& event) {
 
}