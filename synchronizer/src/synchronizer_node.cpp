#include <boost/thread.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <smmap_msgs/messages.h>
#include <smmap/ros_params.hpp>
#include <smmap/robot_interface.hpp>
#include <arc_utilities/voxel_grid.hpp>

#include "synchronizer/synchronizer_ros_params.hpp"

using namespace smmap;
using namespace controls_project;



class Syncronizer{
    public:
        /**
         * @brief Syncronizer
         */
        Syncronizer()
            : nh_( "" )
            , cmd_grippers_traj_as_( nh_, GetCommandGripperTrajTopic( nh_ ), false )
            , cmd_grippers_traj_goal_( nullptr )
            , gripper_names_( { "r_gripper", "l_gripper" } )
            , current_time_( 0.0 )
            , transform_listener_( nh_, ros::Duration( 20.0 ) )
        {
            // TODO: put delay in until transform_listener_ can find needed frames

            // Store the initial configuration as it will be needed by other libraries
            // TODO: find a better way to do this that exposes less internals
//                object_initial_configuration_ = ;

            ROS_INFO( "Creating subscribers and publishers" );
            // Publish to the feedback channel
            system_fbk_pub_ = nh_.advertise< smmap_msgs::SimulatorFeedback >(
                    GetSimulatorFeedbackTopic( nh_ ), 20 );

            r_gripper_cmd_pub_ = nh_.advertise< geometry_msgs::PoseStamped >(
                        GetRightGripperTargetTopic( nh_ ), 1 );
            l_gripper_cmd_pub_ = nh_.advertise< geometry_msgs::PoseStamped >(
                        GetLeftGripperTargetTopic( nh_ ), 1 );

            r_gripper_pose_sub_ = nh_.subscribe(
                        GetRightGripperPoseTopic( nh_ ), 1, &Syncronizer::rightGripperPoseCallback, this );
            l_gripper_pose_sub_ = nh_.subscribe(
                        GetLeftGripperPoseTopic( nh_ ), 1, &Syncronizer::leftGripperPoseCallback, this );

            cloth_tracking_sub_ = nh_.subscribe(
                        GetClothPointCloudTopic( nh_ ), 1, &Syncronizer::clothPointCloudCallback, this );


            ROS_INFO( "Creating services" );
            // Create a service to let others know the internal gripper names
            gripper_names_srv_ = nh_.advertiseService(
                    GetGripperNamesTopic( nh_ ), &Syncronizer::getGripperNamesCallback, this );

            // Create a service to let others know what nodes the grippers are attached too
            gripper_attached_node_indices_srv_ = nh_.advertiseService(
                    GetGripperAttachedNodeIndicesTopic( nh_ ), &Syncronizer::getGripperAttachedNodeIndicesCallback, this );

            // Create a service to let others know the current gripper pose
            gripper_pose_srv_ = nh_.advertiseService(
                    GetGripperPoseTopic( nh_ ), &Syncronizer::getGripperPoseCallback, this);

            // Create a service to let others know the current gripper pose
            gripper_collision_check_srv_ = nh_.advertiseService(
                    GetGripperCollisionCheckTopic( nh_ ), &Syncronizer::gripperCollisionCheckCallback, this);

            // Create a service to let others know the cover points
            cover_points_srv_ = nh_.advertiseService(
                    GetCoverPointsTopic( nh_ ), &Syncronizer::getCoverPointsCallback, this );

            // Create a service to let others know the mirror line data
//                mirror_line_srv_ = nh_.advertiseService(
//                        GetMirrorLineTopic( nh_ ), &Syncronizer::getMirrorLineCallback, this );

            // Create a service to let others know the object initial configuration
            object_initial_configuration_srv_ = nh_.advertiseService(
                    GetObjectInitialConfigurationTopic( nh_ ), &Syncronizer::getObjectInitialConfigurationCallback, this );
        }



        /**
         * @brief run
         */
        void run()
        {
            // Create a service to let others know the object current configuration
            object_current_configuration_srv_ = nh_.advertiseService(
                    GetObjectCurrentConfigurationTopic( nh_ ), &Syncronizer::getObjectCurrentConfigurationCallback, this );

            while( ros::ok() )
            {
                // Check if we've been asked to follow a trajectory
                if ( cmd_grippers_traj_as_.isNewGoalAvailable() )
                {
                    // If we already have a trajectory, premept the current one with the results so far
                    if ( cmd_grippers_traj_as_.isActive() )
                    {
                        cmd_grippers_traj_as_.setPreempted( cmd_grippers_traj_result_ );
                    }

                    cmd_grippers_traj_goal_ = cmd_grippers_traj_as_.acceptNewGoal();
                    cmd_grippers_traj_result_.sim_state_trajectory.clear();
                    cmd_grippers_traj_result_.sim_state_trajectory.reserve( cmd_grippers_traj_goal_->trajectory.size() );
                    cmd_grippers_traj_next_index_ = 0;
                }

                // If the current goal (new or not) has been preempted, send our current results and clear the goal
                if ( cmd_grippers_traj_as_.isPreemptRequested() )
                {
                    cmd_grippers_traj_as_.setPreempted( cmd_grippers_traj_result_ );
                    cmd_grippers_traj_goal_ = nullptr; // stictly speaking, this shouldn't be needed
                }

                // If we have not reached the end of the current gripper trajectory, execute the next step
                if ( cmd_grippers_traj_as_.isActive() )
                {
                    // Advance the sim time and record the sim state
                    smmap_msgs::SimulatorFeedback msg = createSystemFbk();

                    // publish the message
                    system_fbk_pub_.publish( msg );

                    // Deal with the action server parts of feedback
                    smmap_msgs::CmdGrippersTrajectoryFeedback as_feedback;
                    as_feedback.sim_state = msg;
                    cmd_grippers_traj_as_.publishFeedback( as_feedback );

                    cmd_grippers_traj_result_.sim_state_trajectory.push_back( msg );

                    if ( cmd_grippers_traj_next_index_ == cmd_grippers_traj_goal_->trajectory.size() )
                    {
                        cmd_grippers_traj_as_.setSucceeded( cmd_grippers_traj_result_ );
                    }
                }

                usleep( (__useconds_t)(40.0*RobotInterface::DT * 1e6) );
                current_time_ += RobotInterface::DT;
            }
        }

    private:

        ////////////////////////////////////////////////////////////////////////
        // Feedback 'forwarding' related functions
        ////////////////////////////////////////////////////////////////////////

        void rightGripperPoseCallback( const geometry_msgs::PoseStamped::ConstPtr& pose )
        {
            boost::mutex::scoped_lock lock( input_mtx_ );
            r_gripper_pose_ = pose->pose;
        }

        void leftGripperPoseCallback( const geometry_msgs::PoseStamped::ConstPtr& pose )
        {
            boost::mutex::scoped_lock lock( input_mtx_ );
            l_gripper_pose_ = pose->pose;
        }

        void clothPointCloudCallback( const smmap_msgs::PointCloud::ConstPtr& point_cloud )
        {
            boost::mutex::scoped_lock lock( input_mtx_ );
            cloth_config_ = point_cloud->point_cloud;
        }

        smmap_msgs::SimulatorFeedback createSystemFbk()
        {
            boost::mutex::scoped_lock lock( input_mtx_ );

            smmap_msgs::SimulatorFeedback msg;
            msg.object_configuration = cloth_config_;

            msg.gripper_names = gripper_names_;
            msg.gripper_poses.reserve( gripper_names_.size() );
            msg.gripper_poses.push_back( r_gripper_pose_ );
            msg.gripper_poses.push_back( l_gripper_pose_ );

            // TODO: fill out gripper collision data

            msg.sim_time = current_time_;

            return msg;
        }

        ////////////////////////////////////////////////////////////////////////
        // ROS Callbacks
        ////////////////////////////////////////////////////////////////////////

        bool getGripperNamesCallback(
                smmap_msgs::GetGripperNames::Request& req,
                smmap_msgs::GetGripperNames::Response& res )
        {
            (void)req;
            res.names = gripper_names_;
            return true;
        }

        bool getGripperAttachedNodeIndicesCallback(
                smmap_msgs::GetGripperAttachedNodeIndices::Request& req,
                smmap_msgs::GetGripperAttachedNodeIndices::Response& res )
        {
            res.indices.clear();

            // TODO: confirm these indices
            if ( req.name.compare( gripper_names_[0] ) == 0 )
            {
                boost::mutex::scoped_lock lock( input_mtx_ );
                res.indices.push_back( 1 );
            }
            else if ( req.name.compare( gripper_names_[1] ) == 0 )
            {
                res.indices.push_back( 8 );
            }
            else
            {
                ROS_ERROR_STREAM( "Unknown gripper name: " << req.name );
                return false;
            }

            return true;
        }

        bool getGripperPoseCallback(
                smmap_msgs::GetGripperPose::Request& req,
                smmap_msgs::GetGripperPose::Response& res )
        {
            if ( req.name.compare( gripper_names_[0] ) == 0 )
            {
                boost::mutex::scoped_lock lock( input_mtx_ );
                res.pose = r_gripper_pose_;
            }
            else if ( req.name.compare( gripper_names_[1] ) == 0 )
            {
                boost::mutex::scoped_lock lock( input_mtx_ );
                res.pose = l_gripper_pose_;
            }
            else
            {
                ROS_ERROR_STREAM( "Unknown gripper name: " << req.name );
                return false;
            }

            return true;
        }

        bool gripperCollisionCheckCallback(
                smmap_msgs::GetGripperCollisionReport::Request& req,
                smmap_msgs::GetGripperCollisionReport::Response& res )
        {
            return true;
        }

        bool getCoverPointsCallback(
                smmap_msgs::GetPointSet::Request& req,
                smmap_msgs::GetPointSet::Response& res )
        {
            (void)req;
            // NOTE: parent_frame is the frame that we get the result in.
            // Thus, parent * result = child
            tf::StampedTransform tf_transform;
            try
            {
                transform_listener_.lookupTransform( "/parent_frame", "/child_frame", ros::Time( 0.0 ), tf_transform );
            }
            catch ( tf::TransformException ex )
            {
                (void)ex;
                ROS_FATAL( "Unable to lookup transform from /parent_frame to /child_frame" );
                return false;
            }
            const Eigen::Translation3d translation(tf_transform.getOrigin().x(), tf_transform.getOrigin().y(), tf_transform.getOrigin().z());
            const Eigen::Quaterniond rotation(tf_transform.getRotation().w(), tf_transform.getRotation().x(), tf_transform.getRotation().y(), tf_transform.getRotation().z());
            const Eigen::Affine3d transform = translation * rotation;

            // TODO: confirm this math
            res.points.resize( TABLE_NUM_X_TICKS * TABLE_NUM_Y_TICKS );
            for ( ssize_t x_ind = 0; x_ind < TABLE_NUM_X_TICKS; x_ind++ )
            {
                for ( ssize_t y_ind = 0; y_ind < TABLE_NUM_Y_TICKS; y_ind++ )
                {
                    res.points[x_ind * TABLE_NUM_X_TICKS + y_ind] =
                            EigenHelpersConversions::EigenVector3dToGeometryPoint(
                                (transform * Eigen::Vector4d( (double)x_ind * TABLE_X_STEP + TABLE_X_OFFSET, (double)y_ind + TABLE_Y_STEP + TABLE_Y_OFFSET, TABLE_Z_OFFSET, 1.0) ).segment< 3 >( 0 ) );
                }
            }

            return true;
        }

//        bool getMirrorLineCallback(
//                smmap_msgs::GetMirrorLine::Request& req,
//                smmap_msgs::GetMirrorLine::Response& res );

        bool getObjectInitialConfigurationCallback(
                smmap_msgs::GetPointSet::Request& req,
                smmap_msgs::GetPointSet::Response& res )
        {

        }

        bool getObjectCurrentConfigurationCallback(
                smmap_msgs::GetPointSet::Request& req,
                smmap_msgs::GetPointSet::Response& res )
        {
            (void)req;
            boost::mutex::scoped_lock lock( input_mtx_ );
            res.points = cloth_config_;
            return true;
        }

        ////////////////////////////////////////////////////////////////////////
        // ROS Objects and Helpers
        ////////////////////////////////////////////////////////////////////////

        ros::NodeHandle nh_;

        ros::Publisher system_fbk_pub_;
        ros::Publisher r_gripper_cmd_pub_;
        ros::Publisher l_gripper_cmd_pub_;

        ros::ServiceServer gripper_names_srv_;
        ros::ServiceServer gripper_attached_node_indices_srv_;
        ros::ServiceServer gripper_pose_srv_;
        ros::ServiceServer gripper_collision_check_srv_;
        ros::ServiceServer cover_points_srv_;
//        ros::ServiceServer mirror_line_srv_;
        std::vector< geometry_msgs::Point > object_initial_configuration_;
        ros::ServiceServer object_initial_configuration_srv_;
        ros::ServiceServer object_current_configuration_srv_;

        actionlib::SimpleActionServer< smmap_msgs::CmdGrippersTrajectoryAction > cmd_grippers_traj_as_;
        smmap_msgs::CmdGrippersTrajectoryGoalConstPtr cmd_grippers_traj_goal_;
        smmap_msgs::CmdGrippersTrajectoryResult cmd_grippers_traj_result_;
        size_t cmd_grippers_traj_next_index_;

        ////////////////////////////////////////////////////////////////////////
        // Experiment parameters
        ////////////////////////////////////////////////////////////////////////

        std::vector< std::string > gripper_names_;
        // Note that we're going to lie about the current time, just to keep things consistent for the planner
        double current_time_;

        static const int TABLE_NUM_X_TICKS = 9;
        static const int TABLE_NUM_Y_TICKS = 9;
        static constexpr double TABLE_X_STEP = 0.05;
        static constexpr double TABLE_Y_STEP = 0.05;
        static constexpr double TABLE_X_OFFSET = -0.3;
        static constexpr double TABLE_Y_OFFSET = -0.3;
        static constexpr double TABLE_Z_OFFSET = 1.0;

        ////////////////////////////////////////////////////////////////////////
        // System feedback/input Objects
        ////////////////////////////////////////////////////////////////////////

        boost::mutex input_mtx_;

        tf::TransformListener transform_listener_;

        ros::Subscriber r_gripper_pose_sub_;
        ros::Subscriber l_gripper_pose_sub_;
        geometry_msgs::Pose r_gripper_pose_;
        geometry_msgs::Pose l_gripper_pose_;

        ros::Subscriber cloth_tracking_sub_;
        std::vector< geometry_msgs::Point > cloth_config_;
};




int main( int argc, char* argv[] )
{
    // Read in all ROS parameters
    ros::init( argc, argv, "synchronizer_node" );

    Syncronizer sync;
    sync.run();

    return 0;
}
