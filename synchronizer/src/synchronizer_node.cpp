#include <Eigen/StdVector> // Hack around typedef/partial spec problem in arc_utilities
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <smmap_msgs/messages.h>
#include <smmap/ros_params.hpp>
#include <smmap/robot_interface.hpp>
#include <arc_utilities/voxel_grid.hpp>

#include "synchronizer/synchronizer_ros_params.hpp"
#include "synchronizer/table_sdf.hpp"

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
            , TABLE_FRAME_NAME( GetTableTfFrameName( nh_ ) )
            , TABLE_X_SIZE( GetTableXSize( nh_ ) )
            , TABLE_Y_SIZE( GetTableYSize( nh_ ) )
            , TABLE_Z_SIZE( GetTableZSize( nh_ ) )
            , TABLE_LEG_WIDTH( GetTableLegWidth( nh_ ) )
            , TABLE_THICKNESS( GetTableThickness( nh_ ) )
            , table_sdf_( nh_, TABLE_FRAME_NAME, TABLE_X_SIZE, TABLE_Y_SIZE, TABLE_Z_SIZE, TABLE_LEG_WIDTH, TABLE_THICKNESS )
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

            // Startup the action server
            cmd_grippers_traj_as_.start();

            boost::thread spin_thread( boost::bind( &Syncronizer::spin, this, 1000 ) );

            ROS_INFO( "Syncronizer ready." );

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

            spin_thread.join();
        }

    private:

        ////////////////////////////////////////////////////////////////////////
        // Our internal spin function
        ////////////////////////////////////////////////////////////////////////

        void spin( double loop_rate )
        {
            ros::NodeHandle ph("~");
            ROS_INFO( "Starting feedback spinner" );
            while ( ros::ok() )
            {
                ros::getGlobalCallbackQueue()->callAvailable( ros::WallDuration( loop_rate ) );
            }
        }

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

            #warning "Hard coded gripper node indices here"
            // TODO: confirm these indices
            if ( req.name.compare( gripper_names_[0] ) == 0 )
            {
                boost::mutex::scoped_lock lock( input_mtx_ );
                res.indices.push_back( 0 );
            }
            else if ( req.name.compare( gripper_names_[1] ) == 0 )
            {
                res.indices.push_back( GetClothNumXAxisPoints( nh_ ) - 1 );
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
            size_t num_checks = req.pose.size();

            res.gripper_distance_to_obstacle.resize( num_checks );
            res.gripper_nearest_point_to_obstacle.resize( num_checks );
            res.obstacle_surface_normal.resize( num_checks );

            for ( size_t pose_ind = 0; pose_ind < num_checks; pose_ind++ )
            {
                res.gripper_distance_to_obstacle[pose_ind] = table_sdf_.getDistance( req.pose[pose_ind] );
                res.obstacle_surface_normal[pose_ind] = table_sdf_.getGradient( req.pose[pose_ind] );

                #warning "Gripper nearest point to obstacle setting is incorrect"
                // TODO: Make this something other than just the pose itself
                res.gripper_nearest_point_to_obstacle[pose_ind] = req.pose[pose_ind].position;
            }
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
            const Eigen::Translation3d translation( tf_transform.getOrigin().x(), tf_transform.getOrigin().y(), tf_transform.getOrigin().z() );
            const Eigen::Quaterniond rotation( tf_transform.getRotation().w(), tf_transform.getRotation().x(), tf_transform.getRotation().y(), tf_transform.getRotation().z() );
            const Eigen::Affine3d transform = translation * rotation;

            // TODO: confirm this math
            // Note that I assume that the table transform is centered in the AABB of the table
            res.points.resize( TABLE_NUM_X_TICKS * TABLE_NUM_Y_TICKS );
            const double x_offset = -TABLE_X_SIZE / 2.0;
            const double y_offset = -TABLE_Y_SIZE / 2.0;
            const double z_offset = TABLE_Z_SIZE / 2.0;
            const double x_step = TABLE_X_SIZE / (double)(TABLE_NUM_X_TICKS - 1);
            const double y_step = TABLE_Y_SIZE / (double)(TABLE_NUM_Y_TICKS - 1);
            for ( ssize_t x_ind = 0; x_ind < TABLE_NUM_X_TICKS; x_ind++ )
            {
                for ( ssize_t y_ind = 0; y_ind < TABLE_NUM_Y_TICKS; y_ind++ )
                {
                    res.points[x_ind * TABLE_NUM_X_TICKS + y_ind] =
                            EigenHelpersConversions::EigenVector3dToGeometryPoint(
                                (transform * Eigen::Vector4d( (double)x_ind * x_step + x_offset, (double)y_ind * y_step + y_offset, z_offset, 1.0) ).segment< 3 >( 0 ) );
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
            (void)req;

            const double x_axis_size = GetClothXAxisSize( nh_ );
            const double y_axis_size = GetClothYAxisSize( nh_ );
            const int num_x_axis_points = GetClothNumXAxisPoints( nh_ );
            const int num_y_axis_points = GetClothNumYAxisPoints( nh_ );
            const double x_axis_step = x_axis_size / ( num_x_axis_points - 1 );
            const double y_axis_step = y_axis_size / ( num_y_axis_points - 1 );

            res.points.clear();
            res.points.reserve( num_x_axis_points * num_y_axis_points );

            for ( int x_ind = 0; x_ind < num_x_axis_points; x_ind++ )
            {
                for ( int y_ind = 0; y_ind < num_y_axis_points; y_ind++ )
                {
                    geometry_msgs::Point p;
                    p.x = (double)x_ind * x_axis_step;
                    p.y = (double)y_ind * y_axis_step;
                    p.z = 0;
                    res.points.push_back( p );
                }
            }

            return true;
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

        const std::string TABLE_FRAME_NAME;
        const double TABLE_X_SIZE;
        const double TABLE_Y_SIZE;
        const double TABLE_Z_SIZE;
        const double TABLE_LEG_WIDTH;
        const double TABLE_THICKNESS;
        const TableSDF table_sdf_;

        // Cover points parameters
        static const int TABLE_NUM_X_TICKS = 7;
        static const int TABLE_NUM_Y_TICKS = 7;

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
