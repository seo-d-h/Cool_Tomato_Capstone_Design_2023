

#ifndef OPEN_MANIPULATOR_MOVE_XYZ_H_
#define OPEN_MANIPULATOR_MOVE_XYZ_H_

#include <termios.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include "open_manipulator_msgs/Coordinate.h"

#define NUM_OF_JOINT 4
#define DELTA 0.01
#define JOINT_DELTA 0.05
#define PATH_TIME 2

#define GRIPPER_POSITION_X 0.121
#define GRIPPER_POSITION_Y 0
#define GRIPPER_POSITION_Z 0.166

#define REALSENSE_POSITION_X 0.06
#define REALSENSE_POSITION_Y 0.012
#define REALSENSE_POSITION_Z 0.055

#define ALPHA_X 0.003
#define ALPHA_Y 0
#define ALPHA_Z 0.005

class OpenManipulatorMoving
{
 public:
  OpenManipulatorMoving();
  ~OpenManipulatorMoving();

  // update
  void printText();
  void setGoal(char ch);
  void showCoordinate(void);

 
  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  /*****************************************************************************
  ** Variables
  *****************************************************************************/
  std::vector<double> present_joint_angle_;
  std::vector<double> present_kinematic_position_;
  open_manipulator_msgs::KinematicsPose kinematics_pose_;
  open_manipulator_msgs::Coordinate coordinate_;

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initSubscriber();
  void initClient();

  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/
  ros::Subscriber joint_states_sub_;
  ros::Subscriber kinematics_pose_sub_;
  ros::Subscriber coordinate_sub_;

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);
  void coordinateCallback(const open_manipulator_msgs::Coordinate::ConstPtr &msg);

  /*****************************************************************************
  ** ROS Clients and Callback Functions
  *****************************************************************************/
  ros::ServiceClient goal_joint_space_path_client_;
  ros::ServiceClient goal_joint_space_path_from_present_client_;
  ros::ServiceClient goal_task_space_path_from_present_position_only_client_;
  ros::ServiceClient goal_tool_control_client_;
  ros::ServiceClient goal_task_space_path_position_only_client_;

  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time);
  bool setToolControl(std::vector<double> joint_angle);

  /*****************************************************************************
  ** making sangho
  *****************************************************************************/
  bool setTaskSpacePath(std::vector<double> kinematics_pose, double path_time);
  void movePath(void);
  void moveInit(void);
  void moveHome(void);
  void move9(void);
  void move0(void);

  void placeTomato(void);
  void grapTomato(void);
  void openGripper(void);
  void closeGripper(void);
  void moveDetect(void);
  void detecting_start(void);
  void afterGrip(void);

  /*****************************************************************************
  ** Others
  *****************************************************************************/
  struct termios oldt_;

  void disableWaitingForEnter(void);
  void restoreTerminalSettings(void);
  std::vector<double> getPresentJointAngle();
  std::vector<double> getPresentKinematicsPose();
};

#endif //OPEN_MANIPULATOR_MOVE_XYZ_H_
