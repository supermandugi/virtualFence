#include "ros/ros.h"
#include <cstdlib>
#include "gazebo_msgs/GetModelState.h"
#include <stdio.h>

#define PI 3.14159265358979

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xyz_subscriber");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
  gazebo_msgs::GetModelState state;

  geometry_msgs::Pose pose;


  std::stringstream sts;
  sts << "mobile_base";
  state.request.model_name = sts.str();

  double x, y, z, w;
  double sqx, sqy, sqz, sqw;
  double roll, pitch, yaw;

  while(ros::ok())
  {
    if(client.call(state))
    {
      //ROS_INFO("succed: ");
      x = state.response.pose.orientation.x;
      y = state.response.pose.orientation.y;
      z = state.response.pose.orientation.z;
      w = state.response.pose.orientation.w;

      //x = state.response.pose.position.x;
      //y = state.response.pose.position.y;
      //z = state.response.pose.position.z;

      sqx = x * x;
      sqy = y * y;
      sqz = z * z;
      sqw = w * w;

      yaw = (double)(atan2(2.0 * (x*y + z*w), (sqx -sqy -sqz +sqw)));
      yaw = yaw * 180 / PI ; 
      std::cout << "theta : " << yaw << std::endl;
    }
    else
    {
      ROS_INFO("failed");
    }
  }
  //ros::service::call(gazebo/get_model_state '{model_name: mobile_base}');

  return 0;
}
