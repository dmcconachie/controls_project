#ifndef CONTROLS_PROJECT_ROS_PARAMS_HPP
#define CONTROLS_PROJECT_ROS_PARAMS_HPP

#include <string>
#include <arc_utilities/ros_helpers.hpp>
#include <exception>


namespace controls_project
{
    ////////////////////////////////////////////////////////////////////////////
    // ROS Topic settings
    ////////////////////////////////////////////////////////////////////////////

    inline std::string GetRightGripperTargetTopic( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam< std::string >( nh, "pr2_right_gripper_target_topic", "/r_arm_pose_controller/target" );
    }

    inline std::string GetLeftGripperTargetTopic( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam< std::string >( nh, "pr2_left_gripper_target_topic", "/l_arm_pose_controller/target" );
    }

    inline std::string GetRightGripperPoseTopic( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam< std::string >( nh, "pr2_right_gripper_target_topic", "/r_arm_pose_controller/pose" );
    }

    inline std::string GetLeftGripperPoseTopic( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam< std::string >( nh, "pr2_left_gripper_target_topic", "/l_arm_pose_controller/pose" );
    }

    inline std::string GetClothPointCloudTopic( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam< std::string >( nh, "cloth_point_cloud_topic", "cloth_point_cloud" );
    }
}

#endif // ROS_PARAMS_HPP
