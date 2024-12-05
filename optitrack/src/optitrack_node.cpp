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

    // velocity filter
    constexpr static std::array savgol_dev1_polynomial{-4.0f, -3.0f, -1.0f, 0.0f, 1.0f, 2.0f, 3.0f, 4.0f};
    constexpr static float norm_factor = 60.0f;
    etl::circular_buffer<std::array<float, 3>, savgol_dev1_polynomial.size()> pose_buffer;

    // if(!node->get_parameter("local_address", localAddress)){
    //     RCLCPP_ERROR(node->get_logger(), "Could not read local_address from parameters");
    //     rclcpp::shutdown();
    // }
    // if(!node->get_parameter("server_address", serverAddress)){
    //     RCLCPP_ERROR(node->get_logger(), "Could not read server_address from parameters");
    //     rclcpp::shutdown();
    // }

    geometry_msgs::msg::Point last_point;
    geometry_msgs::msg::Quaternion last_rot;

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
            std::array<float, 3> pose{(float)point.x, (float)point.y, (float)tf2::impl::getYaw(q_tf2)};
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
                tf2::Quaternion q{quat.x, quat.y, quat.z, quat.w};
                // quaternion with 90 deg in yaw
                tf2::Quaternion t;
                t.setRPY(0, 0, 0);
                t = t * q;

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
                float yaw_rate = 0.0;
                for (size_t i = 0; i < pose_buffer.size(); ++i)
                {
                    vx += pose_buffer[i][0] * savgol_dev1_polynomial[i];
                    vy += pose_buffer[i][1] * savgol_dev1_polynomial[i];
                    yaw_rate += pose_buffer[i][2] * savgol_dev1_polynomial[i];
                }
                vx /= norm_factor;
                vy /= norm_factor;
                yaw_rate /= norm_factor;

                // vx_b = vx_g * np.cos(yaw) + vy_g * np.sin(yaw)
                // vy_b = -1.0 * vx_g * np.sin(yaw) + vy_g * np.cos(yaw)

                const float yaw = pose_buffer.back()[2]; // last yaw
                const float vx_b = vx * std::cos(yaw) + vy * std::sin(yaw);
                const float vy_b = -1.0 * vx * std::sin(yaw) + vy * std::cos(yaw);

                nav_msgs::msg::Odometry odom{};
                odom.header.frame_id = "odom_2";
                odom.header.stamp = curTimestamp;
                odom.child_frame_id = "base_link";
                odom.pose.pose.position = point;
                odom.pose.pose.orientation = quat;
                odom.twist.twist.linear.x = vx_b;
                odom.twist.twist.linear.y = vy_b;
                odom.twist.twist.linear.z = 0.0;

                odom.pose.covariance[0] = 0.1;  ///< x
                odom.pose.covariance[7] = 0.1;  ///< y
                odom.pose.covariance[35] = 0.2; ///< yaw

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
