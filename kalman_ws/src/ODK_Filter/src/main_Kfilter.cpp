#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <memory>
#include "ODK_Filter.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "main_Kfilter");
  std::cout << "===node ODK_Kfilter starts===" << std::endl;

  Kalman::ODK_Filter sub;

  ros::spin();
  return 0;
}