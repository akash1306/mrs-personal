#pragma once
#ifndef FORMATION_H
#define FORMATION_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/geometry/cyclic.h>

#include <mrs_msgs/ReferenceStampedSrv.h>

#include <mrs_msgs/Float64Stamped.h>

#include <nav_msgs/Odometry.h>

#include <std_srvs/Trigger.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <mrs_lib/batch_visualizer.h>
#include <mrs_lib/attitude_converter.h>

#include <string>

#include<gesture_flocking/Neighbor.h>

struct proximalPar {
  
  double _desired_distance_;
  double _range_multipler_;
  double _steepness_potential_;
  double _strength_potential_;

  double max_range_;
  double noise_;

};

struct motionControlPar{
  
  double _K1_;  // Linear gain
  double _K2_;  // Angular gain
  double _K3_;
  
  double _move_forward_;
  double _interpolate_coeff_;
  bool  _fixed_heading_;
};

#endif