#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <random>
#include <Eigen/Geometry>
#include <time.h>
#include <chrono>
#include <ros/ros.h>
#include <urdf_model/model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>

#define _USE_MATH_DEFINES
#define PR2_ARM_JOINTS 7
#define TWIST_DOF 6

// Set the level of debugging verbosity
#define VERBOSE_DEBUGGING

// Set the "unit" maximum joint change/sec
#define MAXIMUM_JOINT_VELOCITY 1.0
// Set the control rate
#define CONTROL_RATE 5.0
// From the control rate, set the other core operating values
#define CONTROL_INTERVAL (1.0 / CONTROL_RATE)
#define EXECUTION_INTERVAL (2.0 * CONTROL_INTERVAL)
#define WATCHDOG_INTERVAL (3.0 * CONTROL_INTERVAL)
#define MAXIMUM_JOINT_CORRECTION (MAXIMUM_JOINT_VELOCITY * CONTROL_INTERVAL)

// Set the default PID gains
#define DEFAULT_KP 1.0
#define DEFAULT_KI 0.0
#define DEFAULT_KD 0.0

// Set damping for each of the arm joints
#define SHOULDER_PAN_DAMPING 0.5
#define SHOULDER_LIFT_DAMPING 0.5
#define UPPER_ARM_ROLL_DAMPING 0.75
#define ELBOW_FLEX_DAMPING 0.75
#define FOREARM_ROLL_DAMPING 1.0
#define WRIST_FLEX_DAMPING 1.0
#define WRIST_ROLL_DAMPING 1.0

// Set additional offsets for each of the arm joints
#define SHOULDER_PAN_OFFSET 0.0
#define SHOULDER_LIFT_OFFSET 0.0
#define UPPER_ARM_ROLL_OFFSET 0.0
#define ELBOW_FLEX_OFFSET 0.0
#define FOREARM_ROLL_OFFSET 0.0
#define WRIST_FLEX_OFFSET 0.0
#define WRIST_ROLL_OFFSET 0.0

#ifndef SERVOING_CONTROLLER_HPP
#define SERVOING_CONTROLLER_HPP

namespace pr2_mocap_servoing
{
    typedef Eigen::Affine3d Pose;
    typedef Eigen::Matrix<double, TWIST_DOF, 1> Twist;

    class MocapServoingController
    {
    protected:

        enum OPERATING_MODE {INTERNAL_POSE, EXTERNAL_POSE};
        OPERATING_MODE mode_;

        enum OPERATING_STATE {PAUSED, RUNNING};
        OPERATING_STATE state_;

        enum OPERATING_SIDE {LEFT, RIGHT};
        OPERATING_SIDE side_;

        std::vector<std::string> joint_names_;

        double max_joint_correction_;
        double execution_timestep_;

        double watchdog_timeout_;

        ros::NodeHandle nh_;

        robot_model::RobotModelPtr pr2_model_;
        robot_model::RobotStatePtr pr2_kinematic_state_;
        std::unique_ptr<robot_model::JointModelGroup> pr2_arm_group_;
        std::unique_ptr<const robot_model::LinkModel> pr2_arm_link_;
        std::unique_ptr<const robot_model::LinkModel> pr2_torso_link_;
        Eigen::Affine3d gripper_frame_to_wrist_roll_link_transform_;
        Eigen::Affine3d world_to_torso_lift_link_transform_;

        bool arm_pose_valid_;
        bool target_pose_valid_;
        bool arm_config_valid_;
        Pose current_arm_pose_;
        Pose current_target_pose_;
        std::vector<double> current_arm_config_;
        std::vector<double> default_config_;

        // Storage for PID control
        Twist pose_error_integral_;
        Twist last_pose_error_;
        double kp_;
        double ki_;
        double kd_;

        ros::Subscriber arm_pose_sub_;
        ros::Subscriber target_pose_sub_;
        ros::Subscriber arm_config_sub_;

        tf::TransformListener transform_listener_;

        ros::Publisher arm_pose_pub_;

        ros::ServiceServer abort_server_;

        std::unique_ptr<actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>> arm_client_;

        ros::Timer arm_pose_watchdog_;
        ros::Timer target_pose_watchdog_;
        ros::Timer arm_config_watchdog_;

//        void ArmPoseCB(geometry_msgs::PoseStamped arm_pose);

//        void ArmPoseWatchdogCB(const ros::TimerEvent& e);

        void TargetPoseCB(geometry_msgs::PoseStamped target_pose);

        void TargetPoseWatchdogCB(const ros::TimerEvent& e);

        void ArmConfigCB(pr2_controllers_msgs::JointTrajectoryControllerState arm_config);

        void ArmConfigWatchdogCB(const ros::TimerEvent& e);

        bool AbortCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

        Pose ComputeArmPose(std::vector<double>& current_configuration);

        Eigen::MatrixXd ComputeJacobian(std::vector<double>& current_configuration);

        Eigen::MatrixXd weightedInverseKinematicsXd( const Eigen::MatrixXd& J,const std::vector< double >& theta, const double manipubility_threshold,const double damping_ratio );

//        Twist ComputePoseError(Pose& arm_pose, Pose& target_pose);

        void RefreshGlobalStatus();

        std::vector<double> ComputeNextStep(Pose& current_arm_pose, Pose& current_target_pose, std::vector<double>& current_configuration);

        void CommandToTarget(std::vector<double>& current_config, std::vector<double>& target_config);

    public:

        MocapServoingController(ros::NodeHandle& nh, std::string group_name, std::string arm_pose_topic, std::string target_pose_topic, std::string arm_config_topic, std::string arm_command_action, std::string abort_service, double kp, double ki, double kd);

        void Loop();
    };
}

#endif // SERVOING_CONTROLLER_HPP
