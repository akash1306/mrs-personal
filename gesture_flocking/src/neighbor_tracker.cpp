#include<neighbor_tracker.hpp>

class neighbor_class
{
public:
    neighbor_class(/* args */);

private:
    bool is_initialized_, has_this_pose_, has_started_swarming_mode_, last_message_invalid_;
};

neighbor_class::neighbor_class(/* args */)
{
  /* set flags to false */
  is_initialized_            = false;
  has_this_pose_             = false;
  has_started_swarming_mode_ = false;
  last_message_invalid_      = false;
}
