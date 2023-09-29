// STD
#include <sstream>
#include <iostream>
#include <memory>
#include <termios.h>
#include <cstdio>
#include <cstdio>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/impl/utils.h>
#include <tf2/convert.h>

#include "nav_msgs/msg/odometry.hpp"
// // other
#include "Mocap.hpp"
#include "optitrack_msgs/msg/rigid_body.hpp"
#include "optitrack_msgs/msg/marker.hpp"



int main(int argc, char *argv[]) {
    
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> rbPubs;
    std::vector<rclcpp::Publisher<optitrack_msgs::msg::RigidBody>::SharedPtr> rbDebugPubs;

    
    std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> rbOdomPubs;



    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("optitrack");
    
    int nbodies = 1;
    // node->get_parameter("nbodies", nbodies);

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);

    std::string localAddress = "192.168.2.175";
    std::string serverAddress = "192.168.2.162";

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
    // vector<rclcpp::Publisher<optitrack_msgs::msg::RigidBody>::SharedPtr> rbDebugPubs;
    std::vector<uint> seqs;
    for(int r = 0; r < nbodies; ++r) {
        rbPubs.push_back(node->create_publisher<geometry_msgs::msg::PoseStamped>("rigid_body_" + std::to_string(r), 1000));
        rbDebugPubs.push_back(node->create_publisher<optitrack_msgs::msg::RigidBody>("rigid_body_debug_" + std::to_string(r), 1000));
        rbOdomPubs.push_back(node->create_publisher<nav_msgs::msg::Odometry>("odom", 1000));
        seqs.push_back(0);
    }
    rclcpp::Rate loop_rate(240);
    
    
    int count = 0;
    while (rclcpp::ok()) {
        vectorPose poses = mocap.getLatestPoses();
        rclcpp::Time curTimestamp = node->now();

        
        for(const Pose &curPose : poses){
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
                nav_msgs::msg::Odometry odom{};
                odom.header.frame_id = "odom_2";
                odom.header.stamp = curTimestamp;
                odom.child_frame_id = "base_link";
                odom.pose.pose.position = point;
                odom.pose.pose.orientation = quat;
                odom.twist.twist.linear.x = (point.x - last_point.x) / (1.0 / 120.0);
                odom.twist.twist.linear.y = (point.y - last_point.y) / (1.0 / 120.0);
                odom.twist.twist.linear.z = (point.z - last_point.z) / (1.0 / 120.0);

                odom.pose.covariance[0] = 0.1;   ///< x
                odom.pose.covariance[7] = 0.1;   ///< y
                odom.pose.covariance[35] = 0.2;  ///< yaw
                
                tf2::Quaternion last_quat{last_rot.x, last_rot.y, last_rot.z, last_rot.w};
                tf2::Quaternion cur_quat{quat.x, quat.y, quat.z, quat.w};

                auto yaw_last = tf2::impl::getYaw(last_quat);
                auto yaw = tf2::impl::getYaw(cur_quat);
                auto current_angular_velocity = (yaw - yaw_last) / (1.0 / 120.0);
                odom.twist.twist.angular.z = current_angular_velocity;

                // rbOdomPubs[r]->publish(odom);
                
                last_point = point;
                last_rot = quat;
            }


            {
                optitrack_msgs::msg::RigidBody rigidBody;
                rigidBody.header.frame_id = "optitrack";
                rigidBody.header.stamp = curTimestamp;
                // rigidBody.header.seq = seqs[r];
                rigidBody.pose.position = point;
                rigidBody.pose.orientation = quat;
                rigidBody.timestamp = curPose.timestamp;
                rigidBody.mean_error = curPose.meanError;
                for(const Marker &marker : curPose.markers){
                    optitrack_msgs::msg::Marker markerRos;
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


