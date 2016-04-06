#include <Eigen/Geometry>
#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>


#define R_STATE_TOPIC "/r_arm_contoller/state"

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class RobotArm
{
private:
  TrajClient* traj_client_;

  ros::Subscriber arm_config_sub_;

  ros::NodeHandle nh_;

  std::vector<std::string> joint_names_;

  std::vector<double> current_arm_config_;


public:
  RobotArm(ros::NodeHandle& nh) : nh_(nh)
  {
  	joint_names_.resize(7);
    joint_names_[0] = "r_shoulder_pan_joint";
    joint_names_[1] = "r_shoulder_lift_joint";
    joint_names_[2] = "r_upper_arm_roll_joint";
    joint_names_[3] = "r_elbow_flex_joint";
    joint_names_[4] = "r_forearm_roll_joint";
    joint_names_[5] = "r_wrist_flex_joint";
    joint_names_[6] = "r_wrist_roll_joint";

    arm_config_sub_ = nh_.subscribe("/r_arm_controller/state",1, &RobotArm::ArmConfigCB, this);

    traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action",true);

    while(!traj_client_->waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the joint_trajectory_action_server");
    }

  }

  ~RobotArm()
  {
    delete traj_client_;
  }

  inline void ArmConfigCB(pr2_controllers_msgs::JointTrajectoryControllerState arm_config) {
  	std::map<std::string, double> arm_configuration;
    for (size_t idx = 0; idx < arm_config.joint_names.size(); idx ++)
    {
        arm_configuration[arm_config.joint_names[idx]] = arm_config.actual.positions[idx];
    }
    //std::cout << "Got updated config: " << PrettyPrint(arm_configuration, true) << std::endl;
    // Set the config
    ROS_INFO("Getting the current config!");
    std::vector<double> new_arm_config(7);
    new_arm_config[0] = arm_configuration[joint_names_[0]];
    new_arm_config[1] = arm_configuration[joint_names_[1]];
    new_arm_config[2] = arm_configuration[joint_names_[2]];
    new_arm_config[3] = arm_configuration[joint_names_[3]];
    new_arm_config[4] = arm_configuration[joint_names_[4]];
    new_arm_config[5] = arm_configuration[joint_names_[5]];
    new_arm_config[6] = arm_configuration[joint_names_[6]];
    current_arm_config_ = new_arm_config;
  }

  std::vector<double> getConfig() {
  	return current_arm_config_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal)
  {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
  }

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
  pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectory()
  {
    //our goal variable
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
    goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
    goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
    goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(2);

    // First trajectory point
    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = 0.0;
    goal.trajectory.points[ind].positions[1] = 0.0;
    goal.trajectory.points[ind].positions[2] = 0.0;
    goal.trajectory.points[ind].positions[3] = 0.0;
    goal.trajectory.points[ind].positions[4] = 0.0;
    goal.trajectory.points[ind].positions[5] = 0.0;
    goal.trajectory.points[ind].positions[6] = 0.0;
    // Velocities
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

    // Second trajectory point
    // Positions
    ind += 1;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = -0.3;
    goal.trajectory.points[ind].positions[1] = 0.2;
    goal.trajectory.points[ind].positions[2] = -0.1;
    goal.trajectory.points[ind].positions[3] = -1.2;
    goal.trajectory.points[ind].positions[4] = 1.5;
    goal.trajectory.points[ind].positions[5] = -0.3;
    goal.trajectory.points[ind].positions[6] = 0.5;
    // Velocities
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 2 seconds after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

    //we are done; return the goal
    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }
 
};

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "robot_driver");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  RobotArm arm(nh);

  ROS_INFO("Arm has been made");

  ros::Rate loop_rate(10000);

  while(ros::ok()) {

      loop_rate.sleep();
	  ros::spinOnce();

	  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	  robot_model::RobotModelPtr pr2_model_ = robot_model_loader.getModel();
      robot_model::RobotStatePtr pr2_kinematic_state_ = robot_model::RobotStatePtr(new robot_state::RobotState(pr2_model_));
      pr2_kinematic_state_->setToDefaultValues();
      pr2_kinematic_state_->update();

	  std::vector<double> arm_config_ = arm.getConfig();
	  if(!arm_config_.empty()) {
	  	ROS_INFO("State of arm before move: %f, %f, %f, %f, %f, %f, %f",arm_config_[0],arm_config_[1],arm_config_[2],arm_config_[3],arm_config_[4],arm_config_[5],arm_config_[6]);
	  }
	  // Start the trajectory
	  arm.startTrajectory(arm.armExtensionTrajectory());
	  // Wait for trajectory completion
	  while(!arm.getState().isDone() && ros::ok())
	  {
	    usleep(50000);
	  }

	  std::vector<double> new_arm_config_ = arm.getConfig();
	  if(!new_arm_config_.empty()) {
	    ROS_INFO("State of arm after move: %f, %f, %f, %f, %f, %f, %f",new_arm_config_[0],new_arm_config_[1],new_arm_config_[2],new_arm_config_[3],new_arm_config_[4],new_arm_config_[5],new_arm_config_[6]);
	  }
  }

}
