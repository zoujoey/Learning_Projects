#include "feature_detector.h"


namespace lec4 {

FeatureDetector::FeatureDetector(const ros::NodeHandle &nh,
                                 const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh) {
  imageSub_ =
      nh_.subscribe("image_topic", 1, &FeatureDetector::imageCallback, this);
  odometrySub_ =
      nh_.subscribe("odometry_topic", 1, &FeatureDetector::odomCallback, this);
  imagePub_ = nh_.advertise<sensor_msgs::Image>("feature_image", 1000);

  loadParams();
}

void FeatureDetector::loadParams() {
  pnh_.getParam("waitTimeMilliseconds", waitTimeMilliseconds_);

  pnh_.getParam("maxCorners", params_.maxCorners);
  pnh_.getParam("minDistance", params_.minDistance);
  pnh_.getParam("qualityLevel", params_.qualityLevel);
}

void FeatureDetector::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImageConstPtr ptrMono8;
  cv_bridge::CvImagePtr ptrRBG8;
  ptrMono8 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  ptrRBG8 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

  detectFeatures(ptrMono8, params_, features_);
  drawFeatures(ptrRBG8, features_);
  
  imagePub_.publish(ptrRBG8->toImageMsg());

  // cv::imshow("view", ptrRBG8->image);
  // cv::waitKey(waitTimeMilliseconds_);
}

void FeatureDetector::odomCallback(const nav_msgs::Odometry &msg) {
    // Assuming that the position information is in the pose field of Odometry message
    double x = msg.pose.pose.position.x;
    double y = msg.pose.pose.position.y;
    double z = msg.pose.pose.position.z;
    Quaternion q;
    q.x = msg.pose.pose.orientation.x;
    q.y = msg.pose.pose.orientation.y;
    q.z = msg.pose.pose.orientation.z;
    q.w = msg.pose.pose.orientation.w;
    quatToEuler(q);
    std::cout << "position of the drone: " << x << "," << y << "," << z << std::endl;

}
void FeatureDetector::quatToEuler(Quaternion q){
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    std::cout << "orientation of the drone: " << angles.roll << "," << angles.pitch << "," << angles.yaw << std::endl;
}
void FeatureDetector::detectFeatures(cv_bridge::CvImageConstPtr imagePtr,
                                     const DetectorParams &params,
                                     Features &features) {
  cv::goodFeaturesToTrack(imagePtr->image, features, params.maxCorners,
                          params.qualityLevel, params.minDistance, params.mask);
}

void FeatureDetector::drawFeatures(cv_bridge::CvImagePtr imagePtr,
                                   const Features &features) {
  for (const auto &ftr : features) {
    // cv::circle(imagePtr->image, ftr, 2, cv::Scalar(0, 255, 0), 2);
    cv::circle(imagePtr->image, ftr, CIRCLE_RADIUS, cv::Scalar(COLOR_RGB[0], COLOR_RGB[1], COLOR_RGB[2]), CIRCLE_THICKNESS);
  }
}

}  // namespace lec4