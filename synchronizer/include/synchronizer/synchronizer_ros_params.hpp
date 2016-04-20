#ifndef CONTROLS_PROJECT_ROS_PARAMS_HPP
#define CONTROLS_PROJECT_ROS_PARAMS_HPP

#include <exception>
#include <string>
#include <arc_utilities/ros_helpers.hpp>


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

    ////////////////////////////////////////////////////////////////////////////
    // TF Frame Name Settings
    ////////////////////////////////////////////////////////////////////////////

    inline std::string GetTableTfFrameName( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam< std::string >( nh, "table_tf_frame_name", "table_frame" );
    }

    ////////////////////////////////////////////////////////////////////////////
    // Cloth Size Settings - Get rid of these, pull from smmap_experiment params
    ////////////////////////////////////////////////////////////////////////////

    inline double GetClothXAxisSize( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam( nh, "cloth_x_axis_size", 0.7 );
    }

    inline double GetClothYAxisSize( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam( nh, "cloth_y_axis_size", 0.7 );
    }

    inline int GetClothNumXAxisPoints( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam( nh, "cloth_x_axis_num_points", 9 );
    }

    inline int GetClothNumYAxisPoints( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam( nh, "cloth_y_axis_num_points", 9 );
    }
}

#endif // ROS_PARAMS_HPP
