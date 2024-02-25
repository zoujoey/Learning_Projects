#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <memory>
#include <cmath>
#include <math_utils.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace EKalman {

class EK_Filter {
 public:

  struct Quaternion {
   double w, x, y, z;
  };

  struct EulerAngles {
   double roll, pitch, yaw;
  };

  EK_Filter(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  EK_Filter()
      : EK_Filter(ros::NodeHandle(), ros::NodeHandle("~")) {}
  ~EK_Filter() {}

  //prediction terms
  Eigen::VectorXd x; //State Posterior Vector
  Eigen::MatrixXd P; //State Posterior Covariance
  Eigen::VectorXd u; //Control Input Vector
  Eigen::MatrixXd Q; //Control Input Covariance
  Eigen::MatrixXd F; //Motion Model Jacobian
  
  //correction terms
  Eigen::VectorXd xp; //State Estimate/Prior Vector
  Eigen::VectorXd yp; //State Estimate/Prior Vector - motion model
  Eigen::MatrixXd Pp; //State Estimate/Prior Covariance
  Eigen::MatrixXd K; //Kalman Gain
  Eigen::VectorXd y; //Measurement Vector 
  Eigen::MatrixXd R; //Measurement Covariance
  Eigen::MatrixXd G; //Observation Model Jacobian
  
  //Jacobian Calculations
  Eigen::MatrixXd Qp; //Motion Model Jacobian Noise
  Eigen::MatrixXd Rp; //Observation Model Jacobian Noise
  Eigen::MatrixXd wp; //OBservation Model Derivative Noise

  //useful constants
  Eigen::MatrixXd I; //Identity Matrix 
  double t_0 = 0.0;
  double dt = 0.0;

  //main functions
  void init();
  void odom_Callback(const nav_msgs::Odometry &msg);
  void Extended_Kalman_Filter();
  //prediction functions
  void state_prediction();
  void covariance_prediction();
  //kalman gain function
  void kalman_gain();
  //correction functions
  void state_correction();
  void covariance_correction();
  //Jacobian Calculators
  void motion_model_Jacobian();
  void motion_noise_Jacobian();
  void observation_model_Jacobian();
  void observation_noise_Jacobian();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber infoSub_;
  ros::Publisher posePub_;
};

}  // namespace Kalman