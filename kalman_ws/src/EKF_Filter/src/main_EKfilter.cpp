#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <memory>
#include "EK_Filter.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "EK_Filter");
  std::cout << "===node EK_Filter starts===" << std::endl;

  EKalman::EK_Filter sub;
  sub.init();
  std::cout << "===node EK_Filter starts===" << std::endl;

  ros::spin();
  return 0;
}