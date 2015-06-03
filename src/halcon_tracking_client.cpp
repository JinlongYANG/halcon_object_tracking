#include "ros/ros.h"
#include <opencv2/core/core.hpp>


#include <string.h>

#include <perception_msgs/ObjRecState.h>
#include <perception_srvs/ObjRecStateService.h>
#include <perception_srvs/ObjRecConfigService.h>
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "halcon_tracking_client");
//  if (argc != 3)
//  {
//    ROS_INFO("usage: add_two_ints_client X Y");
//    return 1;
//  }

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<perception_srvs::ObjRecStateService>("perception/obj_rec_state_service");
  perception_srvs::ObjRecStateService srv;
  srv.request.new_state = atoll(argv[1]);
  srv.request.set = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Current state: %ld", (long int)srv.response.current_state);
    std::cout << "Success? " << std::boolalpha << srv.response.success << std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service perception/obj_rec_state_service");
    return 1;
  }

  return 0;
}
