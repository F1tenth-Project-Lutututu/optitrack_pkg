// STD
#include <cstdio>
#include <iostream>
#include <memory>
#include <sstream>
#include <termios.h>

// ROS
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/convert.h>
#include <tf2/impl/utils.h>

#include "nav_msgs/msg/odometry.hpp"
// // other
#include "Mocap.hpp"
#include "optitrack_interfaces_msgs/msg/marker.hpp"
#include "optitrack_interfaces_msgs/msg/rigid_body.hpp"

// derivative calc
#include "etl/circular_buffer.h"


float unwrap(float previous_angle, float new_angle) {
    float d = new_angle - previous_angle;
    if (d > M_PI) {
        return new_angle - 2 * M_PI;
    } else if (d < -M_PI) {
        return new_angle + 2 * M_PI;
    } 
    return new_angle;
}


int main(int argc, char *argv[])
{

    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> rbPubs;
    std::vector<rclcpp::Publisher<optitrack_interfaces_msgs::msg::RigidBody>::SharedPtr> rbDebugPubs;

    std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> rbOdomPubs;

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("optitrack");

    int nbodies = 1;
    // node->get_parameter("nbodies", nbodies);

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);

    std::string localAddress = "192.168.1.54";
    std::string serverAddress = "192.168.1.2";

    // velocity filter window 9
    constexpr static std::array savgol_1st_derivative_coef{-4.0f, -3.0f, -2.0f, -1.0f, 0.0f, 1.0f, 2.0f, 3.0f, 4.0f};
    constexpr static float norm_factor_1st_derivative = 60.0f;

    constexpr static std::array savgol_smoother_coef{-21.0, 14.0, 39.0, 54.0, 59.0, 54.0, 39.0, 14.0, -21.0};
    constexpr static float norm_factor_smoother = 231.0f;

    // velocity filter window 5
    // constexpr static std::array savgol_1st_derivative_coef{-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
    // constexpr static float norm_factor_1st_derivative = 10.0f;

    // constexpr static std::array savgol_smoother_coef{-3.0, 12.0, 17.0, 12.0, -3.0};
    // constexpr static float norm_factor_smoother = 35.0f;

    constexpr static float dt = 1.0f / 240.0f;
    etl::circular_buffer<std::array<float, 6>, savgol_1st_derivative_coef.size()> pose_buffer;

    // if(!node->get_parameter("local_address", localAddress)){
    //     RCLCPP_ERROR(node->get_logger(), "Could not read local_address from parameters");
    //     rclcpp::shutdown();
    // }
    // if(!node->get_parameter("server_address", serverAddress)){
    //     RCLCPP_ERROR(node->get_logger(), "Could not read server_address from parameters");
    //     rclcpp::shutdown();
    // }

    Mocap mocap(localAddress, serverAddress);

    // vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> rbPubs;
    std::vector<uint> seqs;
    for (int r = 0; r < nbodies; ++r)
    {
        rbPubs.push_back(node->create_publisher<geometry_msgs::msg::PoseStamped>("optitrack/rigid_body_" + std::to_string(r), 10));
        rbDebugPubs.push_back(node->create_publisher<optitrack_interfaces_msgs::msg::RigidBody>("optitrack/rigid_body_debug_" + std::to_string(r), 10));
        rbOdomPubs.push_back(node->create_publisher<nav_msgs::msg::Odometry>("optitrack/odom", 10));
        seqs.push_back(0);
    }
    rclcpp::Rate loop_rate(360);

    int count = 0;
    while (rclcpp::ok())
    {
        vectorPose poses = mocap.getLatestPoses();
        rclcpp::Time curTimestamp = node->now();

        for (const Pose &curPose : poses)
        {
            int r = curPose.id - 1;

            geometry_msgs::msg::Point point;
            point.x = curPose.t.x();
            point.y = curPose.t.y();
            point.z = curPose.t.z();

            geometry_msgs::msg::Quaternion quat;
            quat.x = curPose.r.x();
            quat.y = curPose.r.y();
            quat.z = curPose.r.z();
            quat.w = curPose.r.w();

            tf2::Quaternion q_tf2{quat.x, quat.y, quat.z, quat.w};
            double roll, pitch, yaw;
            tf2::impl::getEulerYPR(q_tf2, yaw, pitch, roll);
            std::array<float, 6> pose{(float)point.x, (float)point.y, (float)point.z,
                                      (float)roll, (float)pitch, (float)yaw};

            pose_buffer.push(pose);

            {
                geometry_msgs::msg::PoseStamped poseStamped;
                poseStamped.header.frame_id = "odom";
                poseStamped.header.stamp = curTimestamp;
                poseStamped.pose.position = point;
                poseStamped.pose.orientation = quat;
                rbPubs[r]->publish(poseStamped);
            }

            {
                // quaternion with 90 deg in yaw
                tf2::Quaternion t;
                t.setRPY(0, 0, 0);
                t = t * q_tf2;

                geometry_msgs::msg::Quaternion to_send;
                to_send.x = t.getX();
                to_send.y = t.getY();
                to_send.z = t.getZ();
                to_send.w = t.getW();

                geometry_msgs::msg::TransformStamped transformStamped;
                transformStamped.header.frame_id = "odom_2";
                transformStamped.header.stamp = curTimestamp;
                transformStamped.child_frame_id = "base_link";
                transformStamped.transform.translation.x = point.x;
                transformStamped.transform.translation.y = point.y;
                transformStamped.transform.translation.z = point.z;
                transformStamped.transform.rotation = to_send;
                // tf_broadcaster_->sendTransform(transformStamped);
            }

            { // Odometry

                // iterator based for loop
                float vx = 0.0;
                float vy = 0.0;
                float vz = 0.0;
                float roll_rate = 0.0;
                float pitch_rate = 0.0;
                float yaw_rate = 0.0;

                float x = 0.0;
                float y = 0.0;
                float z = 0.0;
                float roll = 0.0;
                float pitch = 0.0;
                float yaw = 0.0;

                float last_roll = pose_buffer[0][3];
                float last_pitch = pose_buffer[0][4];
                float last_yaw = pose_buffer[0][5];
                for (size_t i = 0; i < savgol_1st_derivative_coef.size(); ++i)
                {
                    // unwrap angles to make them coninous among the buffer
                    float unwrapped_roll = unwrap(last_roll, pose_buffer[i][3]);
                    float unwrapped_pitch = unwrap(last_pitch, pose_buffer[i][4]);
                    float unwrapped_yaw = unwrap(last_yaw, pose_buffer[i][5]);
                    vx += pose_buffer[i][0] * savgol_1st_derivative_coef[i];
                    vy += pose_buffer[i][1] * savgol_1st_derivative_coef[i];
                    vz += pose_buffer[i][2] * savgol_1st_derivative_coef[i];
                    roll_rate += unwrapped_roll * savgol_1st_derivative_coef[i];
                    pitch_rate += unwrapped_pitch * savgol_1st_derivative_coef[i];
                    yaw_rate += unwrapped_yaw * savgol_1st_derivative_coef[i];

                    x += pose_buffer[i][0] * savgol_smoother_coef[i];
                    y += pose_buffer[i][1] * savgol_smoother_coef[i];
                    z += pose_buffer[i][2] * savgol_smoother_coef[i];
                    roll += unwrapped_roll * savgol_smoother_coef[i];
                    pitch += unwrapped_pitch * savgol_smoother_coef[i];
                    yaw += unwrapped_yaw * savgol_smoother_coef[i];
                    last_roll = unwrapped_roll;
                    last_pitch = unwrapped_pitch;
                    last_yaw = unwrapped_yaw;
                }

                vx /= norm_factor_1st_derivative * dt;
                vy /= norm_factor_1st_derivative * dt;
                vz /= norm_factor_1st_derivative * dt;
                roll_rate /= norm_factor_1st_derivative * dt;
                pitch_rate /= norm_factor_1st_derivative * dt;
                yaw_rate /= norm_factor_1st_derivative * dt;

                x /= norm_factor_smoother;
                y /= norm_factor_smoother;
                z /= norm_factor_smoother;
                roll /= norm_factor_smoother;
                pitch /= norm_factor_smoother;
                yaw /= norm_factor_smoother;

                // wrap yaw
                roll = std::atan2(std::sin(roll), std::cos(roll));
                pitch = std::atan2(std::sin(pitch), std::cos(pitch));
                yaw = std::atan2(std::sin(yaw), std::cos(yaw));

                // quat from yaw
                tf2::Quaternion quat_tf2;
                quat_tf2.setRPY(roll, pitch, yaw);

                geometry_msgs::msg::Quaternion quat;
                quat.x = quat_tf2.getX();
                quat.y = quat_tf2.getY();
                quat.z = quat_tf2.getZ();
                quat.w = quat_tf2.getW();

                nav_msgs::msg::Odometry odom{};
                odom.header.frame_id = "odom_2";
                odom.header.stamp = curTimestamp;
                odom.child_frame_id = "base_link";
                odom.pose.pose.position.x = x;
                odom.pose.pose.position.y = y;
                odom.pose.pose.position.z = z;

                odom.pose.pose.orientation = quat;
                odom.twist.twist.linear.x = vx;
                odom.twist.twist.linear.y = vy;
                odom.twist.twist.linear.z = vz;

                odom.pose.covariance[0] = 0.1;  ///< x
                odom.pose.covariance[7] = 0.1;  ///< y
                odom.pose.covariance[35] = 0.2; ///< yaw

                odom.twist.twist.angular.x = roll_rate;
                odom.twist.twist.angular.y = pitch_rate;
                odom.twist.twist.angular.z = yaw_rate;

                rbOdomPubs[r]->publish(odom);
            }

            {
                optitrack_interfaces_msgs::msg::RigidBody rigidBody;
                rigidBody.header.frame_id = "optitrack";
                rigidBody.header.stamp = curTimestamp;
                // rigidBody.header.seq = seqs[r];
                rigidBody.pose.position = point;
                rigidBody.pose.orientation = quat;
                rigidBody.timestamp = curPose.timestamp;
                rigidBody.mean_error = curPose.meanError;
                for (const Marker &marker : curPose.markers)
                {
                    optitrack_interfaces_msgs::msg::Marker markerRos;
                    markerRos.location.x = marker.location(0);
                    markerRos.location.y = marker.location(1);
                    markerRos.location.z = marker.location(2);
                    markerRos.residual = marker.residual;
                    markerRos.occluded = marker.occluded;
                    rigidBody.markers.push_back(markerRos);
                }

                rbDebugPubs[r]->publish(rigidBody);
            }

            ++seqs[r];
        }
        rclcpp::spin_some(node);
        loop_rate.sleep();
        ++count;
    }
}