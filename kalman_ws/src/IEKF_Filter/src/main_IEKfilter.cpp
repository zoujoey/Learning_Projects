#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <memory>
#include "IEK_Filter.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "IEK_Filter");
  std::cout << "===node IEK_Filter starts===" << std::endl;

  IEKalman::IEK_Filter sub;
  sub.init();
  std::cout << "===node IEK_Filter starts===" << std::endl;

  ros::spin();
  return 0;
}