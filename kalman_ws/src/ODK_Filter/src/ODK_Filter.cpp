#include "ODK_Filter.h"


namespace Kalman {

ODK_Filter::ODK_Filter(const ros::NodeHandle &nh,
                                 const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh) {
  infoSub_ =
      nh_.subscribe("odom_topic", 1, &ODK_Filter::odom_Callback, this);
  posePub_ = nh_.advertise<nav_msgs::Odometry>("estimated_pose", 1000);
  double pv_0 = 10000;
  double px_0 = 10000;
  double vr_0 = 0.01;
  double xr_0 = 0.01;
  double x_0 = 0;
  double vx_0 = 0;
  double qv = 0.001;
  double qx = 0.001;
  double t_0 = 0;
  double dt = 0;
  //need to edit
  loadParams();
}

void ODK_Filter::loadParams() {
  pnh_.getParam("waitTimeMilliseconds", waitTimeMilliseconds_);
}

void ODK_Filter::odom_Callback(const nav_msgs::Odometry &msg) {
    // Assuming that the position information is in the pose field of Odometry message
    double xz_0 = msg.pose.pose.position.x;
    double vxz_0 = msg.pose.pose.position.y;
    if (t_0 == 0){
      t_0 = msg.header.stamp.toSec(); 
      x_0 = msg.pose.pose.position.x;
    }
    else{
      dt = msg.header.stamp.toSec()-t_0;
      x_0 = kalmanFilter_x(xz_0, vxz_0, dt);
      t_0 = msg.header.stamp.toSec();
    }
    nav_msgs::Odometry fmsg;
    fmsg.header.stamp = msg.header.stamp;
    fmsg.pose.pose.position.x = x_0;
    posePub_.publish(fmsg);
    std::cout << "filter-based position of the drone: " << x_0 << std::endl;
}
double ODK_Filter::kalmanFilter_x(double xz_0, double vxz_0, double dt){
    double px_1 = covariance_extrapolation_x(px_0, pv_0, dt);
    double pv_1 = covariance_extrapolation_v(pv_0);
    double vx_1 = state_extrapolation_v(vx_0);
    double x_1 = state_extrapolation_x(x_0, vx_0, dt);
    double KX_1 = kalman_gain(px_1, xr_0);
    double KV_1 = kalman_gain(pv_1, vr_0);
    x_1 = state_update(x_1, xz_0, KX_1);
    vx_1 = state_update(vx_1, vxz_0, KV_1);
    px_1 = covariance_update(KX_1,px_1);
    pv_1 = covariance_update(KV_1, pv_1);
    x_0 = x_1;
    vx_0 = vx_1;
    pv_0 = pv_1;
    px_0 = px_1;
    return x_1;
    }
double ODK_Filter::state_update(double x_0, double xz_0, double K_1){
  double x_1 = x_0 + K_1*(xz_0 - x_0);
  return x_1;
}
double ODK_Filter::covariance_update(double K_1, double px_0){
  double px_1 = (1-K_1)*px_0;
  return px_1;
}
double ODK_Filter::kalman_gain(double p_0, double r){
  double K_1 = p_0/(p_0+r);    
  return K_1;
}
double ODK_Filter::state_extrapolation_x(double x_0, double v_0, double dt){
  double x_1 = x_0 + dt*v_0;
  return x_1;
}
double ODK_Filter::state_extrapolation_v(double v_0){
  double v_1 = v_0;
  return v_1;
}
double ODK_Filter::covariance_extrapolation_v(double pv_0){
  double pv_1 = pv_0; 
  return pv_1;
}
double ODK_Filter::covariance_extrapolation_x(double px_0, double pv_0, double dt){
    double px_1 = px_0 + dt*dt*pv_0;
    return px_1;
} 
}
// namespace Kalman
