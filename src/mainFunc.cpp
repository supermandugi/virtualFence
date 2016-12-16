#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "openCV.h"
#include <cv_bridge/cv_bridge.h>
#include "NavigationFunction.h"
#include "LineFunction.h"
#include <iostream>
#include <stdlib.h>
#include <thread>
#include "openCV.h"
#include "MapBuilder.h"

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {   
    cv::Mat input(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    input = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::imshow("view", input);
    
    Robot *robot = new Robot();
    
    LineObservation *line = nullptr;
    ObservationData *ob = new ObservationData();

    ob->col = 640;
    ob->row = 480;
    ob->mode = 0;
    ob->draw_flag = 0;

    std::thread* t = new std::thread([&]() {
    while (1) {


      if (line == nullptr)
      //if (point == nullptr)
      {
        line = new LineObservation();
        line->data = ob;
        /*point = new PointObservation();
        point->data = ob;*/
      }

      std::memcpy(ob->RGB_data, input.data, sizeof(unsigned char) * ob->col * ob->row * 3);

      if (!line->run())
      {
        line = nullptr;
        break;
      }

      /*if (!point->run()) {
        point = nullptr;
        break;
      }*/
    }
  });

  MapBuilder mb(5, 5);

  mb.robotptr = robot;
  mb.obptr = ob;

  
  while (1)
  {
    //if (robot->front_prox >= 6) {
      //mb.isObstacleDetected = 1;
    //}else{
      mb.isObstacleDetected = 0;
    //}
    
    mb.run();

    // if (getchar()) {
    //   //isRunning = false;
    //   ob->mode = 9;
    //   t->detach();
    //   delete t;
    //   break;
    // }

//    cv::imshow("position", zumo.position);
    
    cv::waitKey(10);
  }
  
  
  robot->set_vel = 0.0;
  robot->set_rotvel = 0.0;

    
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub;
  sub = it.subscribe("camera/rgb/image_raw", 1, imageCallback);
  cv::namedWindow("view");
  cv::startWindowThread();
  

    
  ros::spin();
  cv::destroyWindow("view");

  //LineObservation *line = nullptr;
  
}