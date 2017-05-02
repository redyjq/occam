#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_requester");

  ros::NodeHandle n;
  ros::Publisher odom_req = n.advertise<std_msgs::Empty>("/beam/publish_odom", 1);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    odom_req.publish(std_msgs::Empty());
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
