#include "EK_Filter.h"


namespace EKalman {

EK_Filter::EK_Filter(const ros::NodeHandle &nh,
                                 const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh) {
  infoSub_ = nh_.subscribe("noisy_pose_topic", 1, &EK_Filter::odom_Callback, this);
  posePub_ = nh_.advertise<nav_msgs::Odometry>("est_pose_topic", 1000);
  double t_0 = 0;
  double dt = 0;
}
void EK_Filter::init(){
  // Initial Prior Estimates
  x = Eigen::VectorXd::Zero(3); // State Posterior Vector

  P.resize(3,3); // State Covariance
  float p = 0.05;
  P   << p, 0, 0,
         0, p, 0,
         0, 0, p;

  u = Eigen::VectorXd::Zero(2); //Control Input Vector

  Q.resize(2,2); // Control Input Covariance
  float q = 0.01;
  Q   << q, 0,
         0, q;
  
  y = Eigen::VectorXd::Zero(2); // Measurement Vector
  
  R.resize(2,2); // Measurement Covariance
  float r = 1;
  R   << r, 0,
         0, r;

  K.resize(3,2); // Kalman Gain
  K   << 0, 0,
         0, 0,
         0, 0;
  
  I.resize(3,3); // Identity Matrix
  I   << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
  xp = Eigen::VectorXd::Zero(3); // State Estimate/Prior Vector
  yp = Eigen::VectorXd::Zero(2); // Observation Model Prior Vector
  Pp.resize(3,3); // State Estimate/Prior Covariance
  
  F.resize(3,3); // Motion Model Jacobian
  Qp.resize(2,2); // Motion Noise Jacobian
  G.resize(2,3); // Observation Model Jacobian
  Rp.resize(2,2); // Observation Noise Jacobian
  wp.resize(3,2); // Observation Noise Derivative
}

void EK_Filter::odom_Callback(const nav_msgs::Odometry &msg) {
    // Assuming that the position information is in the pose field of Odometry message
    y   << msg.pose.pose.position.x,
           msg.pose.pose.position.y;
    u   << msg.twist.twist.linear.x,
           msg.twist.twist.linear.y;
    if (t_0 == 0){
      t_0 = msg.header.stamp.toSec(); 
    x   << y(0),0, -3.141592/2;
    }
    else{
      dt = msg.header.stamp.toSec()-t_0;
      Extended_Kalman_Filter();
      t_0 = msg.header.stamp.toSec();
    }
    nav_msgs::Odometry fmsg;
    fmsg.header.stamp = msg.header.stamp;
    fmsg.header.frame_id = "World";
    tf2::Quaternion quat;
    quat.setRPY(0, 0, x(2));
    fmsg.pose.pose.position.x = x(0);
    fmsg.pose.pose.position.y = x(1);
    fmsg.pose.pose.orientation = tf2::toMsg(quat);
    posePub_.publish(fmsg);
    std::cout << "filter-based position of the drone: " << x(0) << "," << x(1) << "," << x(2) << "," << t_0 << std::endl;
}
void EK_Filter::Extended_Kalman_Filter(){
    //Jacobian Calculations
    motion_model_Jacobian();
    motion_noise_Jacobian();
    observation_model_Jacobian();
    observation_noise_Jacobian();
    //Prediction Step
    state_prediction();
    covariance_prediction();
    //Kalman Gain
    kalman_gain();
    //Correction Step
    covariance_correction();
    state_correction();
    }
void EK_Filter::state_prediction(){
       xp   << (x(0) + cos(x(2))*u(0)*dt),
               (x(1) + sin(x(2))*u(0)*dt),
               (x(2) + u(1)*dt);
       xp(2) = std::fmod(xp(2) + M_PI, 2.0 * M_PI) - M_PI;
}
void EK_Filter::covariance_prediction(){
       Pp = F * P * F.transpose() + Qp;
}
void EK_Filter::kalman_gain(){
       K = Pp * G.transpose() * (G * Pp * G.transpose() + Rp).inverse();
}
void EK_Filter::state_correction(){
       yp   <<(sqrt((xp(0)*xp(0))+(xp(1))*(xp(1)))), 
              (std::atan2(-xp(1),-xp(0)) - xp(2)); 
           // Ensure yp(1) is in the range [-pi, pi]
       if (yp(1) < -M_PI) {
           yp(1) += 2 * M_PI;
       } else if (yp(1) > M_PI) {
           yp(1) -= 2 * M_PI;
       }

       std::cout << "yp " << yp(0) <<" , " << yp(1)<< std::endl;
       std::cout << "y " << y(0) <<" , " << y(1)<< std::endl;
       x = xp + K * (y - yp);
}
void EK_Filter::covariance_correction(){
       P = (I - K * G) * Pp;
}
void EK_Filter::motion_model_Jacobian(){
   F   << 1, 0, (-sin(x(2))*u(0)*dt),
          0, 1, (cos(x(2))*u(0)*dt),
          0, 0, 1;
}
void EK_Filter::motion_noise_Jacobian(){
   wp   <<  cos(x(2)), 0,
            sin(x(2)), 0,
            0,         1;
   Qp = dt*dt * wp * Q * wp.transpose();
}
void EK_Filter::observation_model_Jacobian(){
   G   << (x(0))/(sqrt((x(0)*x(0))+(x(1))*(x(1)))), (x(1))/(sqrt((x(0)*x(0))+(x(1))*(x(1)))), 0,
          (-x(1))/(sqrt((x(0)*x(0))+(x(1))*(x(1)))), (x(0))/(sqrt((x(0)*x(0))+(x(1))*(x(1)))), -1;
}
void EK_Filter::observation_noise_Jacobian(){
   Rp = R;
}
}
// namespace Kalman
