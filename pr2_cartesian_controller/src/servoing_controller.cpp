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
#include <geometry_msgs/PoseStamped.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <smmap_experiment_params/ros_params.hpp>
#include <kinematics_toolbox/kinematics.h>

#include "pr2_cartesian_controller/servoing_controller.hpp"

using namespace pr2_mocap_servoing;

MocapServoingController::MocapServoingController(ros::NodeHandle &nh, std::string group_name, std::string arm_pose_topic, std::string target_pose_topic, std::string arm_config_topic, std::string arm_command_action, std::string abort_service, double kp, double ki, double kd)
    : nh_(nh)
    , transform_listener_( nh_, ros::Duration( 20.0 ) )
{
    // Wait for tf to be ready
    {
        bool tf_ready = false;
        do
        {
            try
            {
                tf::StampedTransform transform;
                transform_listener_.lookupTransform( smmap::GetWorldFrameName(), "/torso_lift_link", ros::Time(0), transform );
                const Eigen::Translation3d translation_w( transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z() );
                const Eigen::Quaterniond rotation_w( transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z() );
                world_to_torso_lift_link_transform_ = translation_w * rotation_w;

                if (group_name == std::string("left_arm"))
                {
                    transform_listener_.lookupTransform( "/l_gripper_frame", "/l_wrist_roll_link", ros::Time(0), transform );
                }
                else if (group_name == std::string("right_arm"))
                {
                    transform_listener_.lookupTransform( "/r_gripper_frame", "/r_wrist_roll_link", ros::Time(0), transform );
                }
                else
                {
                    throw std::invalid_argument("Invalid group name");
                }
                transform_listener_.lookupTransform( "/l_gripper_frame", "/l_wrist_roll_link", ros::Time(0), transform );


                const Eigen::Translation3d translation( transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z() );
                const Eigen::Quaterniond rotation( transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z() );
                gripper_frame_to_wrist_roll_link_transform_ = translation * rotation;

                // ROS_INFO_STREAM("gripper to wrist_roll: " << gripper_frame_to_wrist_roll_link_transform_.matrix() << "\n");

                // exit(-1);

                tf_ready = true;
            }
            catch ( tf::TransformException ex )
            {
                ROS_WARN( "%s",ex.what() );
                ros::Duration(1.0).sleep();
            }
        }
        while ( ros::ok() && !tf_ready );
    }

    // Set mode
    mode_ = INTERNAL_POSE;
    // Set up an internal robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    pr2_model_ = robot_model_loader.getModel();
    pr2_kinematic_state_ = robot_model::RobotStatePtr(new robot_state::RobotState(pr2_model_));
    pr2_kinematic_state_->setToDefaultValues();
    pr2_kinematic_state_->update();
    if (group_name == std::string("left_arm"))
    {
        side_ = LEFT;
        pr2_arm_group_ = std::unique_ptr<robot_model::JointModelGroup>(pr2_model_->getJointModelGroup(group_name));
        pr2_arm_link_ = std::unique_ptr<const robot_model::LinkModel>(pr2_kinematic_state_->getLinkModel(std::string("l_wrist_roll_link")));
        pr2_torso_link_ = std::unique_ptr<const robot_model::LinkModel>(pr2_kinematic_state_->getLinkModel(std::string("torso_lift_link")));
        // Set the joint names
        joint_names_.resize(PR2_ARM_JOINTS);
        joint_names_[0] = "l_shoulder_pan_joint";
        joint_names_[1] = "l_shoulder_lift_joint";
        joint_names_[2] = "l_upper_arm_roll_joint";
        joint_names_[3] = "l_elbow_flex_joint";
        joint_names_[4] = "l_forearm_roll_joint";
        joint_names_[5] = "l_wrist_flex_joint";
        joint_names_[6] = "l_wrist_roll_joint";

        default_config_ = { 0.46554443363374876, 0.25733775608375103, 0.9016356363484644, -0.36291338027676234, 0.9615921263791672, -0.7101873020292652, -6.297547394516469 };
    }
    else if (group_name == std::string("right_arm"))
    {
        side_ = RIGHT;
        pr2_arm_group_ = std::unique_ptr<robot_model::JointModelGroup>(pr2_model_->getJointModelGroup(group_name));
        pr2_arm_link_ = std::unique_ptr<const robot_model::LinkModel>(pr2_kinematic_state_->getLinkModel(std::string("r_wrist_roll_link")));
        pr2_torso_link_ = std::unique_ptr<const robot_model::LinkModel>(pr2_kinematic_state_->getLinkModel(std::string("torso_lift_link")));
        // Set the joint names
        joint_names_.resize(PR2_ARM_JOINTS);
        joint_names_[0] = "r_shoulder_pan_joint";
        joint_names_[1] = "r_shoulder_lift_joint";
        joint_names_[2] = "r_upper_arm_roll_joint";
        joint_names_[3] = "r_elbow_flex_joint";
        joint_names_[4] = "r_forearm_roll_joint";
        joint_names_[5] = "r_wrist_flex_joint";
        joint_names_[6] = "r_wrist_roll_joint";

        default_config_ = { -0.46554443363374876, 0.25733775608375103, -0.9016356363484644, -0.36291338027676234, -0.9615921263791672, -0.7101873020292652, -6.297547394516469 };
    }
    else
    {
        throw std::invalid_argument("Invalid group name");
    }
    // Setup topics
    target_pose_sub_ = nh_.subscribe(target_pose_topic, 1, &MocapServoingController::TargetPoseCB, this);
    arm_config_sub_ = nh_.subscribe(arm_config_topic, 1, &MocapServoingController::ArmConfigCB, this);

    arm_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(arm_pose_topic,1, this);
    // Setup abort service
    abort_server_ = nh_.advertiseService(abort_service, &MocapServoingController::AbortCB, this);
    // Setup trajectory controller interface
    arm_client_ = std::unique_ptr<actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>>(new actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>(arm_command_action, true));
    ROS_INFO("Waiting for arm controllers to come up...");
    arm_client_->waitForServer();
    // Set gains
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    // Set max size for qdot
    max_joint_correction_ = MAXIMUM_JOINT_CORRECTION;
    // Set execution timestep
    execution_timestep_ = EXECUTION_INTERVAL;
    // Set timeout
    watchdog_timeout_ = WATCHDOG_INTERVAL;
    // Initialize the control variables to safe values
    arm_pose_valid_ = false;
    arm_config_valid_ = false;
    target_pose_valid_ = false;
    // Initialize the PID values to zero
    pose_error_integral_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    last_pose_error_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // Start in PAUSED mode
    state_ = PAUSED;
}

void MocapServoingController::TargetPoseCB(geometry_msgs::PoseStamped target_pose)
{
    try
    {
        // Check if the provided pose is a special "cancel target" message
        if (target_pose.pose.orientation.x == 0.0 && target_pose.pose.orientation.y == 0.0 && target_pose.pose.orientation.z == 0.0 && target_pose.pose.orientation.w == 0.0)
        {
            ROS_INFO("Cancelling pose target, switching to PAUSED mode");
            // Set the status
            target_pose_valid_ = false;
            // Check and set the global status
            RefreshGlobalStatus();
            // We don't reset the timer, instead we cancel it
            target_pose_watchdog_.stop();
        }
        else
        {
//            ROS_INFO_STREAM("Servoing to " << target_pose.pose.position.x << " "
//                                           << target_pose.pose.position.y << " "
//                                           << target_pose.pose.position.z << " "
//                                           << target_pose.pose.orientation.x << " "
//                                           << target_pose.pose.orientation.y << " "
//                                           << target_pose.pose.orientation.z << " "
//                                           << target_pose.pose.orientation.w);
            // First, check to make sure the frame is correct
            Eigen::Affine3d torso_lift_link_frame_to_given_frame( Eigen::Translation3d( 0, 0, 0 ) );
            if (target_pose.header.frame_id != std::string("/torso_lift_link") && target_pose.header.frame_id != std::string("torso_lift_link"))
            {
                tf::StampedTransform tf_transform;
                transform_listener_.lookupTransform( "/torso_lift_link", target_pose.header.frame_id, ros::Time( 0.0 ), tf_transform );

                const Eigen::Translation3d translation( tf_transform.getOrigin().x(), tf_transform.getOrigin().y(), tf_transform.getOrigin().z() );
                const Eigen::Quaterniond rotation( tf_transform.getRotation().w(), tf_transform.getRotation().x(), tf_transform.getRotation().y(), tf_transform.getRotation().z() );
                torso_lift_link_frame_to_given_frame = translation * rotation;
            }

            // Convert to Eigen
            Eigen::Translation3d translation(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
            Eigen::Quaterniond rotation(target_pose.pose.orientation.w, target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z);
            Pose new_target_pose = translation * rotation;

            // Set the pose
            current_target_pose_ = torso_lift_link_frame_to_given_frame * new_target_pose * gripper_frame_to_wrist_roll_link_transform_;
            // Set the status
            target_pose_valid_ = true;
            // Check and set global status
            RefreshGlobalStatus();
            // Reset watchdog timer
            target_pose_watchdog_ = nh_.createTimer(ros::Duration(watchdog_timeout_), &MocapServoingController::TargetPoseWatchdogCB, this, true);
        }
    }
    catch ( tf::TransformException ex )
    {
        (void)ex;
        ROS_ERROR_STREAM( "Unable to lookup transform from " << target_pose.header.frame_id << " to " << "/torso_lift_link" );
        return;
    }
}

void MocapServoingController::TargetPoseWatchdogCB(const ros::TimerEvent& e)
{
    (void)e;
    ROS_WARN("Target pose hasn't been updated in %f seconds - continuing to current target", watchdog_timeout_);
}

void MocapServoingController::ArmConfigCB(pr2_controllers_msgs::JointTrajectoryControllerState arm_config)
{
    // Extract joint positions in the right order
    if (arm_config.joint_names.size() != arm_config.actual.positions.size() || arm_config.joint_names.size() != PR2_ARM_JOINTS)
    {
        ROS_ERROR("Malformed configuration update - skipping update");
        return;
    }
    std::map<std::string, double> arm_configuration;
    for (size_t idx = 0; idx < arm_config.joint_names.size(); idx ++)
    {
        arm_configuration[arm_config.joint_names[idx]] = arm_config.actual.positions[idx];
    }
    //std::cout << "Got updated config: " << PrettyPrint(arm_configuration, true) << std::endl;
    // Set the config
    std::vector<double> new_arm_config(PR2_ARM_JOINTS);
    new_arm_config[0] = arm_configuration[joint_names_[0]];
    new_arm_config[1] = arm_configuration[joint_names_[1]];
    new_arm_config[2] = arm_configuration[joint_names_[2]];
    new_arm_config[3] = arm_configuration[joint_names_[3]];
    new_arm_config[4] = arm_configuration[joint_names_[4]];
    new_arm_config[5] = arm_configuration[joint_names_[5]];
    new_arm_config[6] = arm_configuration[joint_names_[6]];
    current_arm_config_ = new_arm_config;
    // If we're in INTERNAL_POSE mode, compute the arm pose
    if (mode_ == INTERNAL_POSE)
    {
        current_arm_pose_ = ComputeArmPose(current_arm_config_);
    }
    // Set the status
    arm_config_valid_ = true;
    // Check and set global status
    RefreshGlobalStatus();
    // Reset watchdog timer
    arm_config_watchdog_ = nh_.createTimer(ros::Duration(watchdog_timeout_), &MocapServoingController::ArmConfigWatchdogCB, this, true);
}

void MocapServoingController::ArmConfigWatchdogCB(const ros::TimerEvent& e)
{
    (void)e;
    ROS_WARN("Arm config hasn't been updated in %f seconds - pausing execution until a new config update received", watchdog_timeout_);
    arm_config_valid_ = false;
    state_ = PAUSED;
}

bool MocapServoingController::AbortCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    (void)req;
    (void)res;
    // Cancel the current pose target
    ROS_INFO("Cancelling pose target, switching to PAUSED mode");
    // Set the status
    target_pose_valid_ = false;
    // Check and set the global status
    RefreshGlobalStatus();
    // We don't reset the timer, instead we cancel it
    target_pose_watchdog_.stop();
    return true;
}

Pose MocapServoingController::ComputeArmPose(std::vector<double>& current_configuration)
{
    // Update joint values
    pr2_kinematic_state_->setJointGroupPositions(pr2_arm_group_.get(), current_configuration);
    // Update the joint transforms
    pr2_kinematic_state_->enforceBounds();
    pr2_kinematic_state_->update(true);
    // Get the transform from base to torso
    Pose current_base_to_torso_pose = pr2_kinematic_state_->getGlobalLinkTransform(pr2_torso_link_.get());
    // Get the transform from base to wrist
    Pose current_base_to_wrist_pose = pr2_kinematic_state_->getGlobalLinkTransform(pr2_arm_link_.get());
    // Get the arm pose
    Pose current_arm_pose = current_base_to_torso_pose.inverse() * current_base_to_wrist_pose;
    return current_arm_pose;
}

Eigen::MatrixXd MocapServoingController::ComputeJacobian(std::vector<double>& current_configuration)
{
    // Update joint values
    pr2_kinematic_state_->setJointGroupPositions(pr2_arm_group_.get(), current_configuration);
    // Update the joint transforms
    pr2_kinematic_state_->enforceBounds();
    pr2_kinematic_state_->update(true);
    // Compute the Jacobian
    Eigen::MatrixXd current_jacobian = pr2_kinematic_state_->getJacobian(pr2_arm_group_.get());
    return current_jacobian;
}

Eigen::MatrixXd MocapServoingController::weightedInverseKinematicsXd( const Eigen::MatrixXd& J,
                                          const std::vector< double >& theta,
                                          const double manipubility_threshold,
                                          const double damping_ratio )
{
    const ssize_t num_joints = (ssize_t)theta.size();
    const ssize_t num_velocities = J.rows();

    const kinematics::Matrix6d W_x = kinematics::Matrix6d::Identity();
    Eigen::MatrixXd W_q = Eigen::MatrixXd::Identity( num_joints, num_joints );

    W_q(0,0) = 1./SHOULDER_PAN_DAMPING;
    W_q(1,1) = 1./SHOULDER_LIFT_DAMPING;
    W_q(2,2) = 1./UPPER_ARM_ROLL_DAMPING;
    W_q(3,3) = 1./ELBOW_FLEX_DAMPING;
    W_q(4,4) = 1./FOREARM_ROLL_DAMPING;
    W_q(5,5) = 1./WRIST_FLEX_DAMPING;
    W_q(6,6) = 1./WRIST_ROLL_DAMPING;

    // Yes, this is ugly. This is to suppress a warning on type conversion related to Eigen operations
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wconversion"
    const Eigen::MatrixXd J_w = W_x * J * W_q;
    #pragma GCC diagnostic pop

    // Yes, this is ugly. This is to suppress a warning on type conversion related to Eigen operations
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wconversion"
    const kinematics::Matrix6d JwJwtranspose = J_w * J_w.transpose();
    #pragma GCC diagnostic pop

    // find the damping ratio
    // Based on Manipulability, in 'Prior Work' of above paper
    double manipubility = JwJwtranspose.determinant();
#ifdef VERBOSE_DEBUGGING
    std::cout << "Manipubility: " << manipubility << std::endl;
#endif
    double damping = 0;
    if ( manipubility < manipubility_threshold )
    {
        damping = damping_ratio * std::pow( 1 - manipubility / manipubility_threshold, 2 );

        std::cout << "Damping the pseudoinverse with value " << damping << std::endl;
        std::cout << "Jw * Jw_transpose:\n" << JwJwtranspose << std::endl;
    }

    const Eigen::MatrixXd tmp = JwJwtranspose + damping * Eigen::MatrixXd::Identity( num_velocities, num_velocities );
    // Yes, this is ugly. This is to suppress a warning on type conversion related to Eigen operations
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wconversion"
    const Eigen::MatrixXd J_inv = J_w.transpose() * tmp.inverse();
    #pragma GCC diagnostic pop

    return W_q * J_inv * W_x;
}

void MocapServoingController::RefreshGlobalStatus()
{
    if (mode_ == INTERNAL_POSE)
    {
        if (target_pose_valid_ && arm_config_valid_)
        {
            state_ = RUNNING;
        }
        else
        {
            state_ = PAUSED;
        }
    }
    else
    {
        if (arm_pose_valid_ && target_pose_valid_ && arm_config_valid_)
        {
            state_ = RUNNING;
        }
        else
        {
            state_ = PAUSED;
        }
    }
}

std::vector<double> MocapServoingController::ComputeNextStep(Pose& current_arm_pose, Pose& current_target_pose, std::vector<double>& current_configuration)
{
    // Get the current jacobian
    Eigen::MatrixXd current_jacobian = ComputeJacobian(current_configuration);
    // Compute the pose error in our 'world frame'
    Twist pose_error =  kinematics::calculateError( current_arm_pose, current_target_pose );  // ComputePoseError(current_arm_pose, current_target_pose);

    std::cout << "target:\n" << current_arm_pose.matrix() << std::endl;
    std::cout << "current:\n" << current_target_pose.matrix() << std::endl;

    std::cout << "pose error:\n" << pose_error.transpose() << std::endl;

    // Compute the integral of pose error & update the stored value
    pose_error_integral_ = pose_error_integral_ + (pose_error * CONTROL_INTERVAL);
    // Compute the derivative of pose error
    Twist pose_error_derivative = (pose_error - last_pose_error_) / CONTROL_INTERVAL;
    // Update the stored pose error
    last_pose_error_ = pose_error;
    // Convert pose errors into cartesian velocity
    Twist pose_correction = (pose_error * kp_) + (pose_error_integral_ * ki_) + (pose_error_derivative * kd_);
    std::cout << "pose correction:\n" << pose_correction.transpose() << std::endl << std::endl;


    // Use the Jacobian pseudoinverse
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wconversion"
    Eigen::MatrixXd damped_pseudo_inverse = weightedInverseKinematicsXd( current_jacobian, current_arm_config_, 1e-3, 1e-2 );
    Eigen::VectorXd joint_correction = damped_pseudo_inverse * pose_correction;

    //Eigen::VectorXd joint_correction = EigenHelpers::Pinv(current_jacobian, EigenHelpers::SuggestedRcond()) * pose_correction;
    #pragma GCC diagnostic pop

    //Eigen::VectorXd joint_correction = joint_velocities * CONTROL_INTERVAL;
#ifdef VERBOSE_DEBUGGING
    std::cout << "Current raw joint correction:\n" << joint_correction.transpose() << std::endl;
#endif
//    Eigen::VectorXd damped_joint_correction = joint_correction;
//    damped_joint_correction[0] = damped_joint_correction[0] * SHOULDER_PAN_DAMPING;
//    damped_joint_correction[1] = damped_joint_correction[1] * SHOULDER_LIFT_DAMPING;
//    damped_joint_correction[2] = damped_joint_correction[2] * UPPER_ARM_ROLL_DAMPING;
//    damped_joint_correction[3] = damped_joint_correction[3] * ELBOW_FLEX_DAMPING;
//    damped_joint_correction[4] = damped_joint_correction[4] * FOREARM_ROLL_DAMPING;
//    damped_joint_correction[5] = damped_joint_correction[5] * WRIST_FLEX_DAMPING;
//    damped_joint_correction[6] = damped_joint_correction[6] * WRIST_ROLL_DAMPING;

    // Bound qdot to max magnitude of 0.05
    double joint_correction_magnitude = joint_correction.norm();
#ifdef VERBOSE_DEBUGGING
    std::cout << "Correction Magnitude: " << joint_correction_magnitude << std::endl;
#endif
    if (joint_correction_magnitude > max_joint_correction_)
    {
        joint_correction = (joint_correction / joint_correction_magnitude) * max_joint_correction_;
    }
#ifdef VERBOSE_DEBUGGING
    std::cout << "Current limited joint correction:\n" << joint_correction.transpose() << std::endl;
#endif
    // Combine the joint correction with the current configuration to form the target configuration
    std::vector<double> target_configuration(PR2_ARM_JOINTS);
    target_configuration[0] = current_configuration[0] + joint_correction[0] + SHOULDER_PAN_OFFSET;
    target_configuration[1] = current_configuration[1] + joint_correction[1] + SHOULDER_LIFT_OFFSET;
    target_configuration[2] = current_configuration[2] + joint_correction[2] + UPPER_ARM_ROLL_OFFSET;
    target_configuration[3] = current_configuration[3] + joint_correction[3] + ELBOW_FLEX_OFFSET;
    target_configuration[4] = current_configuration[4] + joint_correction[4] + FOREARM_ROLL_OFFSET;
    target_configuration[5] = current_configuration[5] + joint_correction[5] + WRIST_FLEX_OFFSET;
    target_configuration[6] = current_configuration[6] + joint_correction[6] + WRIST_ROLL_OFFSET;
#ifdef VERBOSE_DEBUGGING
    std::cout << "Current configuration:    " << PrettyPrint::PrettyPrint(current_configuration, true) << std::endl;
    std::cout << "New target configuration: " << PrettyPrint::PrettyPrint(target_configuration, true) << std::endl;
#endif

    std::cout << std::endl << std::endl;
    return target_configuration;
}

void MocapServoingController::Loop()
{
    ros::Rate spin_rate(CONTROL_RATE);
    while (ros::ok())
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id=smmap::GetWorldFrameName();
        pose.pose = EigenHelpersConversions::EigenAffine3dToGeometryPose( world_to_torso_lift_link_transform_ * current_arm_pose_ * gripper_frame_to_wrist_roll_link_transform_.inverse());
        arm_pose_pub_.publish(pose);
        // Do the next step
        if (state_ == RUNNING)
        {
            // Compute the next step
            std::vector<double> target_config = ComputeNextStep(current_arm_pose_, current_target_pose_, current_arm_config_);
            // std::vector<double> target_config = { -5.8503660753217446e-05, -4.691989941019159e-05, -0.003717017512879117, 1.57, 0.0005635499845348946, -0.47191914133014023, 4.367701055585371e-05 };
            // Command the robot
            CommandToTarget(current_arm_config_, target_config);
        }
        else if (current_arm_config_.size() == PR2_ARM_JOINTS)
        {
            CommandToTarget(current_arm_config_, current_arm_config_);
        }

        // Process callbacks
        ros::spinOnce();
        // Spin
        spin_rate.sleep();
    }
}

void MocapServoingController::CommandToTarget(std::vector<double>& current_config, std::vector<double>& target_config)
{
    pr2_controllers_msgs::JointTrajectoryGoal command;
    // Populate command
    command.trajectory.joint_names = joint_names_;
    command.trajectory.header.stamp = ros::Time::now();
    // Populate target point
    trajectory_msgs::JointTrajectoryPoint start_point;
    start_point.positions = current_config;
    start_point.velocities.resize(start_point.positions.size(), 0.0);
    start_point.time_from_start = ros::Duration(0.0);
    // Populate target point
    trajectory_msgs::JointTrajectoryPoint target_point;



    target_point.positions = target_config;
//    start_point.positions = current_config;



    target_point.velocities.resize(target_point.positions.size(), 0.0);
    // Set the execution time
    target_point.time_from_start = ros::Duration(execution_timestep_);
    // Add point
    command.trajectory.points.push_back(target_point);
    // Command the arm
    arm_client_->sendGoal(command);
}
