/*!
 * \file  cj02_imu_node.cpp
 * \brief CJ02-IMU ROS1 driver node.
 *
 * Publishes:
 *   /imu/data_raw     (sensor_msgs/Imu)         — raw accel + gyro at 800 Hz
 *   /imu/data         (sensor_msgs/Imu)         — with orientation from ESKF
 *   /imu/attitude     (geometry_msgs/Vector3Stamped) — roll/pitch/yaw in degrees
 *
 * Parameters:
 *   ~port   (string, default "/dev/ttyUSB0") — serial port
 *   ~baud   (int, default 460800)            — baud rate
 *   ~frame_id (string, default "imu_link")   — TF frame ID
 *
 * Usage:
 *   rosrun cj02_imu_node cj02_imu_node _port:=/dev/ttyUSB0
 *   roslaunch cj02_imu_node cj02_imu_node.launch  (create launch file as needed)
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include "cj02_imu.h"

#include <cmath>
#include <thread>
#include <atomic>

class CJ02ImuNode {
public:
    CJ02ImuNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), running_(false) {
        // Parameters
        pnh_.param<std::string>("port", port_, "/dev/ttyUSB0");
        pnh_.param<int>("baud", baud_, 460800);
        pnh_.param<std::string>("frame_id", frame_id_, "imu_link");

        // Publishers
        pub_raw_      = nh_.advertise<sensor_msgs::Imu>("/imu/data_raw", 100);
        pub_data_     = nh_.advertise<sensor_msgs::Imu>("/imu/data", 100);
        pub_attitude_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/imu/attitude", 100);

        // SDK callbacks (assignment, not function-call)
        imu_.onRaw = [this](const cj02::RawFrame& f) { rawCallback(f); };
        imu_.onAttitude = [this](const cj02::AttitudeFrame& f) { attitudeCallback(f); };

        ROS_INFO("CJ02-IMU node: port=%s baud=%d frame=%s",
                 port_.c_str(), baud_, frame_id_.c_str());
    }

    ~CJ02ImuNode() { stop(); }

    bool start() {
        if (!imu_.open(port_, baud_)) {
            ROS_ERROR("Cannot open serial port %s @ %d", port_.c_str(), baud_);
            return false;
        }
        running_ = true;
        reader_ = std::thread([this]() { imu_.run(); });
        ROS_INFO("CJ02-IMU connected and running");
        return true;
    }

    void stop() {
        running_ = false;
        imu_.stop();
        if (reader_.joinable()) reader_.join();
        imu_.close();
    }

    bool isRunning() const { return running_; }

    uint32_t goodFrames() const { return imu_.goodFrames(); }
    uint32_t badFrames() const { return imu_.badFrames(); }

private:
    ros::NodeHandle nh_, pnh_;
    cj02::CJ02IMU imu_;
    std::thread reader_;
    std::atomic<bool> running_;
    std::string port_, frame_id_;
    int baud_;

    ros::Publisher pub_raw_, pub_data_, pub_attitude_;

    void rawCallback(const cj02::RawFrame& f) {
        sensor_msgs::Imu msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame_id_;

        // Convert to SI units
        constexpr float DEG2RAD = 3.14159265358979f / 180.0f;
        constexpr float G2MSS = 9.80665f;

        msg.linear_acceleration.x = f.accX_mg() / 1000.0f * G2MSS;
        msg.linear_acceleration.y = f.accY_mg() / 1000.0f * G2MSS;
        msg.linear_acceleration.z = f.accZ_mg() / 1000.0f * G2MSS;

        msg.angular_velocity.x = f.gyrX_dps() * DEG2RAD;
        msg.angular_velocity.y = f.gyrY_dps() * DEG2RAD;
        msg.angular_velocity.z = f.gyrZ_dps() * DEG2RAD;

        // Covariances (unknown)
        msg.linear_acceleration_covariance[0] = -1;
        msg.angular_velocity_covariance[0] = -1;

        pub_raw_.publish(msg);
    }

    void attitudeCallback(const cj02::AttitudeFrame& f) {
        ros::Time now = ros::Time::now();

        // Full IMU message with orientation
        sensor_msgs::Imu msg;
        msg.header.stamp = now;
        msg.header.frame_id = frame_id_;

        // Euler → Quaternion
        float roll_rad  = f.roll  * 3.14159265358979f / 180.0f;
        float pitch_rad = f.pitch * 3.14159265358979f / 180.0f;
        float yaw_rad   = f.yaw   * 3.14159265358979f / 180.0f;

        tf2::Quaternion q;
        q.setRPY(roll_rad, pitch_rad, yaw_rad);
        msg.orientation.x = q.x();
        msg.orientation.y = q.y();
        msg.orientation.z = q.z();
        msg.orientation.w = q.w();
        msg.orientation_covariance[0] = 0.0025;  // ~2.5° std
        msg.orientation_covariance[4] = 0.0025;
        msg.orientation_covariance[8] = 0.0025;

        // Raw data also in the combined message
        constexpr float G2MSS = 9.80665f;
        // (these come from the raw callback, so we don't duplicate here —
        //  /imu/data has orientation, /imu/data_raw has raw sensor values)

        pub_data_.publish(msg);

        // Simple attitude message
        geometry_msgs::Vector3Stamped att;
        att.header.stamp = now;
        att.header.frame_id = frame_id_;
        att.vector.x = f.roll;
        att.vector.y = f.pitch;
        att.vector.z = f.yaw;
        pub_attitude_.publish(att);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cj02_imu_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    CJ02ImuNode node(nh, pnh);

    if (!node.start()) {
        ROS_ERROR("Failed to start CJ02-IMU node");
        return 1;
    }

    ros::Rate diag_rate(1.0);  // 1 Hz diagnostics
    while (ros::ok() && node.isRunning()) {
        ROS_INFO_THROTTLE(5.0, "CJ02-IMU: good=%u bad=%u",
                          node.goodFrames(), node.badFrames());
        ros::spinOnce();
        diag_rate.sleep();
    }

    node.stop();
    ROS_INFO("CJ02-IMU node stopped");
    return 0;
}
