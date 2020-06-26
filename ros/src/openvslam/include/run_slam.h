#ifndef RUN_SLAM_H
#define RUN_SLAM_H
#include <openvslam/system.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Header.h"

namespace PoseGen{
    void generate_pose_msg(openvslam::Mat44_t& raw_matrix, std_msgs::Header& header, geometry_msgs::PoseStamped& out){
            // Positional (Point)
        out.pose.position.x = raw_matrix(0,3);
        out.pose.position.y = raw_matrix(1,3);
        out.pose.position.z = raw_matrix(2,3);

        // Orientation (Quaternion)
        openvslam::Mat33_t rotation_matrix;
        // Creating the rotation matrix 
        for(int row=0; row<3; row++){
            for(int col=0; col<3; col++){
                rotation_matrix(row,col) = raw_matrix(row,col);
            }
        }
        openvslam::Quat_t temp_quaternion(rotation_matrix); // The rotation matrix is automatically converted to a Quaternion through this object
        out.pose.orientation.x = temp_quaternion.x();
        out.pose.orientation.y = temp_quaternion.y();
        out.pose.orientation.z = temp_quaternion.z();
        out.pose.orientation.w = temp_quaternion.w();
        if(out.pose.orientation.w < 0){
            out.pose.orientation.x *= -1; 
            out.pose.orientation.y *= -1;
            out.pose.orientation.z *= -1;
            out.pose.orientation.w *= -1;
        }
        out.header = header;
    }
};


#endif