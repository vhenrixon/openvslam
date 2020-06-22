#include <gtest/gtest.h>
#include "../include/run_slam.h"
#include <openvslam/system.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Header.h"

TEST(PoseGenerationTest, PoseMSG){
    openvslam::Mat44_t test_matrix;
    test_matrix << 0.999998,  0.00133198 , 0.00141354, 0.00402181,
                -0.00133388, 0.999998, 0.00134452, 0.000896306,
                -0.00141175, -0.0013464, 0.999998,  -0.00268095,
                    0,          0,          0,          1;

    geometry_msgs::PoseStamped pose_msg; 
    std_msgs::Header header; 
    header.seq = 0;                        // Start Sequence 
    header.stamp.sec = 0;                // Start Seconds
    header.stamp.nsec = 0;              // Start nano seconds
    header.frame_id = "/camera_pose";
    PoseGen::generate_pose_msg(test_matrix, header, pose_msg);

    // [ -0.0006727, 0.0007063, -0.0006665, 0.9999993 ] Quaternion [x, y, z, w]  (in Radians)
    EXPECT_NEAR(test_matrix(0,3), pose_msg.pose.position.x, .00010); // X
    EXPECT_NEAR(test_matrix(1,3), pose_msg.pose.position.y, .0010); // Y
    EXPECT_NEAR(test_matrix(2,3), pose_msg.pose.position.z, .0010); // Z

 
    EXPECT_NEAR(-0.0006727, pose_msg.pose.orientation.x, .00010); // X
    EXPECT_NEAR(0.0007063, pose_msg.pose.orientation.y, .00010);  // Y
    EXPECT_NEAR(-0.0006665, pose_msg.pose.orientation.z, .00010);  // Z
    EXPECT_NEAR(0.9999993, pose_msg.pose.orientation.w, .00010); // W
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
