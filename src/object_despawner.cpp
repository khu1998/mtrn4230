#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"


void clearTable(void) {

}

void clearTableCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data) {
      clearTable();
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "object_despawner");

  ros::NodeHandle n;

  ros::Subscriber clearTableSub = n.subscribe("clear_table", 1000, clearTableCallback);

  ros::spin();

  return 0;
}