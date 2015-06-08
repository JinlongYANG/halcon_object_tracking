#include "ros/ros.h"
#include <opencv2/core/core.hpp>


#include <string.h>
#include "halcon_shape_detector_3d.cpp"

#include <perception_msgs/ObjRecState.h>
#include <perception_srvs/ObjRecStateService.h>
#include <perception_srvs/ObjRecConfigService.h>

#include <QMutex>
#include <pthread.h>

#define OBJECT_RECOGNITION_RESTART 1001
#define OBJECT_RECOGNITION_PAUSE 1002
#define OBJECT_RECOGNITION_STOP 1003

using namespace std;
using namespace ros;

ros::ServiceServer set_config_server, status_server;
pthread_t  gThreadHandle;


QMutex mutex_state;
int32_t state = 1002;




void get_state(int32_t &state_local)
{
    mutex_state.lock();
    state_local = state;
    mutex_state.unlock();
}

void set_state(int32_t &state_local)
{
    mutex_state.lock();
    state = state_local;
    mutex_state.unlock();
}

bool obj_rec_config_service_callback(perception_srvs::ObjRecConfigServiceRequest &req, perception_srvs::ObjRecConfigServiceResponse &res){
    if(req.set){//set current model pose:

    }
    else{//retrieve current model pose:

    }
}

bool obj_rec_status_service_callback(perception_srvs::ObjRecStateServiceRequest &req, perception_srvs::ObjRecStateServiceResponse &res)
{
    int32_t state_local;
    if(req.set)//set current state
    {
        //set current state
        set_state(req.new_state);
        get_state(state_local);
        res.current_state = state_local;
        if(req.new_state == state_local)
            res.success = true;
        else
            res.success = false;
        std::cout << "state changed to : " << req.new_state << std::endl;

    }
    else //retrieve current state
    {
        get_state(state_local);
        res.current_state = state_local;
        res.success = true;
    }

    return true;
}

void workerThread(void *parameters)
{
    HALCON_SHAPE_DETECTOR_3D detector;
    detector.initialization();
    detector.create_model();
    detector.load_all_models();
    ROS_INFO("Detector initialized!");



    int32_t state_local;
    get_state(state_local);

    while(ros::ok()){
    if(state_local == OBJECT_RECOGNITION_RESTART){
        //cout <<"running!"<< endl;
        detector.detect_one_loaded_model(5);
    }
    else if(state_local == OBJECT_RECOGNITION_PAUSE){
        //cout <<"paused!"<< endl;

    }
    else if(state_local == OBJECT_RECOGNITION_STOP){
        //cout <<"stopped!"<< endl;
        detector.reset();
        detector.create_model();
        detector.load_all_models();
        ROS_INFO("Detector reinitialized!");
        state_local = OBJECT_RECOGNITION_PAUSE;
        set_state(state_local);
    }
    get_state(state_local);
    }

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "halcon_tracking");
  ros::NodeHandle nh;
  int foo;

  ROS_INFO("Detector initialization...");

  int error;

  if((error=pthread_create(&gThreadHandle,
                           NULL,
                           (void*(*)(void*))workerThread,
                           (void*)&foo))!=0)
  {
      cout << "Error in pthread_create. Error: " << error << endl;
      return 0;
  }


  ROS_INFO("Preparing services...");
  set_config_server = nh.advertiseService("perception/obj_rec_config_service", obj_rec_config_service_callback);
  status_server = nh.advertiseService("perception/obj_rec_state_service", &obj_rec_status_service_callback);
  ROS_INFO("Ready to detect and track object.");


  ros::spin();

  return 0;
}
