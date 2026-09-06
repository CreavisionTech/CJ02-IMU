/*!
 * \file  cj02_imu_node.cpp
 * \brief CJ02-IMU ROS2 driver node.
 *
 * Publishes:
 *   /imu/data_raw     (sensor_msgs/Imu)              — raw accel + gyro at 800 Hz
 *   /imu/data         (sensor_msgs/Imu)              — with orientation from ESKF
 *   /imu/attitude     (geometry_msgs/Vector3Stamped) — roll/pitch/yaw in degrees
 *
 * Parameters:
 *   port      (string, default "/dev/ttyUSB0") — serial port
 *   baud      (int, default 460800)            — baud rate
 *   frame_id  (string, default "imu_link")     — TF frame ID
 *
 * Usage:
 *   ros2 run cj02_imu cj02_imu_node --ros-args -p port:=/dev/ttyUSB0
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "cj02_imu.h"

#include <cmath>
#include <thread>
#include <atomic>

using namespace std::chrono_literals;

class CJ02ImuNode : public rclcpp::Node {
public:
    CJ02ImuNode() : Node("cj02_imu_node"), running_(false) {
        // Parameters
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud", 460800);
        this->declare_parameter<std::string>("frame_id", "imu_link");

        port_ = this->get_parameter("port").as_string();
        baud_ = this->get_parameter("baud").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

        // Publishers
        pub_raw_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 100);
        pub_data_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 100);
        pub_attitude_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/attitude", 100);

        // SDK callbacks (assignment, not function-call)
        imu_.onRaw = [this](const cj02::RawFrame& f) { rawCallback(f); };
        imu_.onAttitude = [this](const cj02::AttitudeFrame& f) { attitudeCallback(f); };

        RCLCPP_INFO(this->get_logger(),
                    "CJ02-IMU node: port=%s baud=%d frame=%s",
                    port_.c_str(), (int)baud_, frame_id_.c_str());
    }

    ~CJ02ImuNode() { stop(); }

    bool start() {
        if (!imu_.open(port_, (int)baud_)) {
            RCLCPP_ERROR(this->get_logger(),
                         "Cannot open serial port %s @ %d",
                         port_.c_str(), (int)baud_);
            return false;
        }
        running_ = true;
        reader_ = std::thread([this]() { imu_.run(); });
        RCLCPP_INFO(this->get_logger(), "CJ02-IMU connected and running");
        return true;
    }

    void stop() {
        running_ = false;
        imu_.stop();
        if (reader_.joinable()) reader_.join();
        imu_.close();
    }

private:
    cj02::CJ02IMU imu_;
    std::thread reader_;
    std::atomic<bool> running_;
    std::string port_, frame_id_;
    int64_t baud_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_raw_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_data_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_attitude_;

    sensor_msgs::msg::Imu last_raw_;
    uint16_t last_raw_seq_ = 0;
    bool have_raw_ = false;

    void rawCallback(const cj02::RawFrame& f) {
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->now();
        msg.header.frame_id = frame_id_;

        constexpr float DEG2RAD = 3.14159265358979f / 180.0f;
        constexpr float G2MSS = 9.80665f;

        msg.linear_acceleration.x = f.accX_mg() / 1000.0f * G2MSS;
        msg.linear_acceleration.y = f.accY_mg() / 1000.0f * G2MSS;
        msg.linear_acceleration.z = f.accZ_mg() / 1000.0f * G2MSS;

        msg.angular_velocity.x = f.gyrX_dps() * DEG2RAD;
        msg.angular_velocity.y = f.gyrY_dps() * DEG2RAD;
        msg.angular_velocity.z = f.gyrZ_dps() * DEG2RAD;

        // All-zero covariance means unknown; -1 means the field is absent.
        msg.orientation_covariance[0] = -1;
        last_raw_ = msg;
        last_raw_seq_ = f.seq;
        have_raw_ = true;

        pub_raw_->publish(msg);
    }

    void attitudeCallback(const cj02::AttitudeFrame& f) {
        auto now = this->now();

        // Pair only the raw frame with the same output sequence. If it was
        // lost, mark those fields unavailable instead of publishing fake zeros.
        sensor_msgs::msg::Imu msg;
        if (have_raw_ && last_raw_seq_ == static_cast<uint16_t>(f.seq)) {
            msg = last_raw_;
        } else {
            msg.linear_acceleration_covariance[0] = -1;
            msg.angular_velocity_covariance[0] = -1;
        }
        have_raw_ = false;
        msg.header.stamp = now;
        msg.header.frame_id = frame_id_;

        float roll_rad  = f.roll  * 3.14159265358979f / 180.0f;
        float pitch_rad = f.pitch * 3.14159265358979f / 180.0f;
        float yaw_rad   = f.yaw   * 3.14159265358979f / 180.0f;

        tf2::Quaternion q;
        q.setRPY(roll_rad, pitch_rad, yaw_rad);
        msg.orientation.x = q.x();
        msg.orientation.y = q.y();
        msg.orientation.z = q.z();
        msg.orientation.w = q.w();
        msg.orientation_covariance[0] = f.mode == 0 ? -1.0 : 0.0025;
        msg.orientation_covariance[4] = 0.0025;
        msg.orientation_covariance[8] = 0.0025;

        pub_data_->publish(msg);

        // Simple attitude message (euler angles in degrees)
        auto att = geometry_msgs::msg::Vector3Stamped();
        att.header.stamp = now;
        att.header.frame_id = frame_id_;
        att.vector.x = f.roll;
        att.vector.y = f.pitch;
        att.vector.z = f.yaw;
        pub_attitude_->publish(att);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CJ02ImuNode>();

    if (!node->start()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to start CJ02-IMU node");
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(node);
    node->stop();
    rclcpp::shutdown();
    return 0;
}
