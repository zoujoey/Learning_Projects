#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <random>

class RandomPoseGenerator {
public:
    RandomPoseGenerator(double stddev) : distribution(0.0, stddev) {}

    double addGaussianNoise(double value) {
        return value + distribution(generator);
    }

private:
    std::default_random_engine generator;
    std::normal_distribution<double> distribution;
};

// Function to generate random motion parameters
void randomMotionParameters(double& x0, double& y0, double& z0, double& v0x, double& v0y, double& v0z, double& ax, double& ay, double& az) {
    x0 = rand() % 11;
    y0 = rand() % 11;
    z0 = rand() % 11;
    v0x = rand() % 11;
    v0y = rand() % 11;
    v0z = rand() % 11;
    ax = rand() % 11;
    ay = rand() % 11;
    az = rand() % 11;
}

// Function to calculate motion equations
void motionEquations(double t, double x0, double y0, double z0, double v0x, double v0y, double v0z, double ax, double ay, double az, double& x, double& y, double& z, double& vx, double& vy, double& vz, double& ax_final, double& ay_final, double& az_final) {
    // Calculate position equations
    x = x0 + v0x * t + 0.5 * ax * t * t;
    y = y0 + v0y * t + 0.5 * ay * t * t;
    z = z0 + v0z * t + 0.5 * az * t * t;
    x = x/10;
    y = y/10;
    z = z/10;
    // Calculate velocity equations
    vx = v0x + ax * t;
    vy = v0y + ay * t;
    vz = v0z + az * t;
    vx = vx/10;
    vy = vy/10;
    vz = vz/10;
    // Acceleration remains constant, so it's just the given acceleration values
    ax_final = ax;
    ay_final = ay;
    az_final = az;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ex1_Kfilter");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom_topic", 1000);
    ros::Publisher pos_pub = n.advertise<nav_msgs::Odometry>("position_topic", 1000);  // New topic for position data
    ros::Rate loop_rate(10);

    // Generate random motion parameters once
    double x0, y0, z0, v0x, v0y, v0z, ax, ay, az;
    randomMotionParameters(x0, y0, z0, v0x, v0y, v0z, ax, ay, az);
    ax = 0;
    ay = 0;
    az = 0;
    

    // Specify standard deviations for Gaussian noise
    double pose_noise_stddev = 0.1;  // Adjust as needed

    // Create a RandomPoseGenerator instance for adding noise
    RandomPoseGenerator poseNoiseGenerator(pose_noise_stddev);

    int count = 0;
    double t = 0.0;

    // Record the start time
    ros::Time start_time = ros::Time::now();

    while (ros::ok()) {
        nav_msgs::Odometry msg;

        // Calculate elapsed time
        ros::Time current_time = ros::Time::now();
        ros::Duration elapsed_time = current_time - start_time;
        t = elapsed_time.toSec();
        ros::Time elapsed_time_2 = ros::Time (t, 0);

        // Calculate position, velocity, and acceleration at time t
        double x, y, z, vx, vy, vz, ax_final, ay_final, az_final;
        motionEquations(t, x0, y0, z0, v0x, v0y, v0z, ax, ay, az, x, y, z, vx, vy, vz, ax_final, ay_final, az_final);

        // Publish actual position to pos topic
        nav_msgs::Odometry pos_msg;
        pos_msg.header.stamp = elapsed_time_2;  // Set the timestamp to the same as the main message
        pos_msg.pose.pose.position.x = x;
        pos_msg.pose.pose.position.y = y;
        pos_msg.pose.pose.position.z = z;
        pos_pub.publish(pos_msg);
        std::cout << "actual position of the drone: " << x << "," << y << "," << z << "," << t << std::endl;

        // Add Gaussian noise to pose data
        vx = poseNoiseGenerator.addGaussianNoise(vx);
        vy = poseNoiseGenerator.addGaussianNoise(vy);
        vz = poseNoiseGenerator.addGaussianNoise(vz);
        x = poseNoiseGenerator.addGaussianNoise(x);
        y = poseNoiseGenerator.addGaussianNoise(y);
        z = poseNoiseGenerator.addGaussianNoise(z);

        // Fill in the Odometry message
        msg.header.stamp = elapsed_time_2;
        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = vx;

        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, 0);  // Assuming no rotation for simplicity
        msg.pose.pose.orientation.x = quaternion.x();
        msg.pose.pose.orientation.y = quaternion.y();
        msg.pose.pose.orientation.z = quaternion.z();
        msg.pose.pose.orientation.w = quaternion.w();

        odom_pub.publish(msg);

        std::cout << "noisy velocity of the drone: " << x << "," << y << "," << z << "," << t << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
// namespace Kalman
