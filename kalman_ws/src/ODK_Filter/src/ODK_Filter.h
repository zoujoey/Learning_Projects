#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <memory>
#include <cmath>

namespace Kalman {

class ODK_Filter {
 public:

  struct Quaternion {
   double w, x, y, z;
  };

  struct EulerAngles {
   double roll, pitch, yaw;
  };

  ODK_Filter(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ODK_Filter()
      : ODK_Filter(ros::NodeHandle(), ros::NodeHandle("~")) {}
  ~ODK_Filter() {}

 private:
  double pv_0 = 2500;
  double px_0 = 10000;
  double vr_0 = 2500;
  double xr_0 = 100000;
  double x_0 = 0;
  double vx_0 = 0;
  double qv = 0.001;
  double qx = 0.001;
  double t_0 = 0;
  double dt = 0;
  void loadParams();
  void odom_Callback(const nav_msgs::Odometry &msg);
  double kalmanFilter_x(double xz_0, double vxz_0, double dt);
  
  
  double state_update(double x_0, double xz_0, double K_1);
  double covariance_update(double K_1, double px_0);
  double kalman_gain(double p_0, double r);
  double state_extrapolation_x(double x_0, double v_0, double dt);
  double state_extrapolation_v(double v_0);
  double covariance_extrapolation_v(double pv_0);
  double covariance_extrapolation_x(double px_0, double pv_0, double dt);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber infoSub_;
  ros::Publisher posePub_;
  int waitTimeMilliseconds_{30};
};

}  // namespace Kalman