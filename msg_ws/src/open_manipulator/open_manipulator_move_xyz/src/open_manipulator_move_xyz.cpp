/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#include "open_manipulator_move_xyz/open_manipulator_move_xyz.h"

OpenManipulatorMoving::OpenManipulatorMoving()
: node_handle_(""),
  priv_node_handle_("~")
{
  /************************************************************
  ** Initialize variables
  ************************************************************/
  present_joint_angle_.resize(NUM_OF_JOINT);
  present_kinematic_position_.resize(3);

  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initSubscriber();
  initClient();

  disableWaitingForEnter();
  ROS_INFO("OpenManipulator Moving to XYZ start");
}

OpenManipulatorMoving::~OpenManipulatorMoving()
{
  restoreTerminalSettings();
  ROS_INFO("Terminate OpenManipulator Moving");
  ros::shutdown();
}

void OpenManipulatorMoving::initClient()
{
  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_joint_space_path_from_present_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path_from_present");
  goal_task_space_path_from_present_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present_position_only");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
  goal_task_space_path_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_position_only");
}

void OpenManipulatorMoving::initSubscriber()
{
  joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &OpenManipulatorMoving::jointStatesCallback, this);
  kinematics_pose_sub_ = node_handle_.subscribe("kinematics_pose", 10, &OpenManipulatorMoving::kinematicsPoseCallback, this);
  coordinate_sub_ = node_handle_.subscribe("xyz_topic", 10, &OpenManipulatorMoving::coordinateCallback, this);
}

void OpenManipulatorMoving::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for (std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
  {
    if (!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;
}

void OpenManipulatorMoving::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);

  present_kinematic_position_ = temp_position;

  kinematics_pose_.pose = msg->pose;
}

void OpenManipulatorMoving::coordinateCallback(const open_manipulator_msgs::Coordinate::ConstPtr &msg)
{
  coordinate_ = *msg;
  open_manipulator_msgs::Coordinate temp = coordinate_;
  coordinate_.x = temp.z - REALSENSE_POSITION_X + GRIPPER_POSITION_X + ALPHA_X;
  coordinate_.y = -temp.x + REALSENSE_POSITION_Y + ALPHA_Y;
  coordinate_.z = -temp.y + REALSENSE_POSITION_Z + GRIPPER_POSITION_Z + ALPHA_Z;
  // printf("------\n");
  // printf("Get_date\n");
  // printf("Start Time(sec): %d\n", msg->start_time);
  // printf("Message Sequence: %d\n", msg->msg_seq);
  // printf("X: %f\n", msg->x);
  // printf("Y: %f\n", msg->y);
  // printf("Z: %f\n\n", msg->z);
  // showCoordinate();
}


std::vector<double> OpenManipulatorMoving::getPresentJointAngle()
{
  return present_joint_angle_;
}

std::vector<double> OpenManipulatorMoving::getPresentKinematicsPose()
{
  return present_kinematic_position_;
}

bool OpenManipulatorMoving::setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_from_present_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorMoving::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorMoving::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back(priv_node_handle_.param<std::string>("end_effector_name", "gripper"));
  srv.request.joint_position.position = joint_angle;

  if (goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorMoving::setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;
  srv.request.planning_group = priv_node_handle_.param<std::string>("end_effector_name", "gripper");
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);
  srv.request.path_time = path_time;


  if (goal_task_space_path_from_present_position_only_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorMoving::setTaskSpacePath(std::vector<double> kinematics_pose, double path_time){
  open_manipulator_msgs::SetKinematicsPose srv;

  srv.request.end_effector_name = "gripper";

  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

  srv.request.kinematics_pose.pose.orientation.w = kinematics_pose_.pose.orientation.w;
  srv.request.kinematics_pose.pose.orientation.x = kinematics_pose_.pose.orientation.x;
  srv.request.kinematics_pose.pose.orientation.y = kinematics_pose_.pose.orientation.y;
  srv.request.kinematics_pose.pose.orientation.z = kinematics_pose_.pose.orientation.z;

  srv.request.path_time = path_time;

  if(goal_task_space_path_position_only_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

void OpenManipulatorMoving::movePath(void){
  std::vector<double> kinematics_pose;

  // home position
  // kinematics_pose.push_back(0.134);
  // kinematics_pose.push_back(0);
  // kinematics_pose.push_back(0.241);

  kinematics_pose.push_back(0.263);
  kinematics_pose.push_back(-0.11);
  kinematics_pose.push_back(0.117);

    if(!setTaskSpacePath(kinematics_pose, PATH_TIME))
  {
    return;
  }
}

void OpenManipulatorMoving::moveInit(void){
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0);
    joint_name.push_back("joint2"); joint_angle.push_back(0);
    joint_name.push_back("joint3"); joint_angle.push_back(0);
    joint_name.push_back("joint4"); joint_angle.push_back(0);
    setJointSpacePath(joint_name, joint_angle, path_time);
}
void OpenManipulatorMoving::moveHome(void){
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(-1.05);
    joint_name.push_back("joint3"); joint_angle.push_back(0.35);
    joint_name.push_back("joint4"); joint_angle.push_back(0.70);
    setJointSpacePath(joint_name, joint_angle, path_time);
}

void OpenManipulatorMoving::moveDetect(void){
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(-1.5);
    joint_name.push_back("joint3"); joint_angle.push_back(1.0);
    joint_name.push_back("joint4"); joint_angle.push_back(0.50);
    setJointSpacePath(joint_name, joint_angle, path_time);
}

void OpenManipulatorMoving::move9(void){
  std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(-1.05);
    joint_name.push_back("joint3"); joint_angle.push_back(0.35);
    joint_name.push_back("joint4"); joint_angle.push_back(0.70);
    setJointSpacePath(joint_name, joint_angle, path_time);
sleep(2);


  std::vector<double> kinematics_pose;

  kinematics_pose.push_back(0.265);
  kinematics_pose.push_back(0.060);
  kinematics_pose.push_back(0.148);

    if(!setTaskSpacePath(kinematics_pose, PATH_TIME))
  {
    // return;
  }
  

    sleep(2);
    closeGripper();
    sleep(2);
  std::vector<double> kinematics_pose1;
  kinematics_pose1.push_back(0.165);
  kinematics_pose1.push_back(0.060);
  kinematics_pose1.push_back(0.148);

    if(!setTaskSpacePath(kinematics_pose1, PATH_TIME))
  {
    // return;
  }
    sleep(2);
    placeTomato();
    sleep(2);
    openGripper();
    sleep(2);
    moveHome();
    
}

void OpenManipulatorMoving::move0(void){
std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(-1.05);
    joint_name.push_back("joint3"); joint_angle.push_back(0.35);
    joint_name.push_back("joint4"); joint_angle.push_back(0.70);
    setJointSpacePath(joint_name, joint_angle, path_time);
    sleep(2);

  std::vector<double> kinematics_pose;

  // home position
  // kinematics_pose.push_back(0.134);
  // kinematics_pose.push_back(0);
  // kinematics_pose.push_back(0.241);

  kinematics_pose.push_back(0.309);
  kinematics_pose.push_back(-0.030);
  kinematics_pose.push_back(0.189);

    if(!setTaskSpacePath(kinematics_pose, PATH_TIME))
  {
    // return;
  }
  sleep(2);
  closeGripper();
  sleep(2);
  std::vector<double> kinematics_pose1;
  kinematics_pose1.push_back(0.209);
  kinematics_pose1.push_back(-0.030);
  kinematics_pose1.push_back(0.189);

    if(!setTaskSpacePath(kinematics_pose1, PATH_TIME))
  {
    // return;
  }
    sleep(2);
    placeTomato();
    sleep(2);
    openGripper();
    sleep(2);
    moveHome();
    
}

void OpenManipulatorMoving::afterGrip(void){
    sleep(2);
    placeTomato();
    sleep(2);
    openGripper();
    sleep(1);
    moveDetect();
    sleep(2);
}

void OpenManipulatorMoving::detecting_start(void){

    open_manipulator_msgs::Coordinate temp = coordinate_;

    if(!temp.msg_seq==0){
      moveHome();
      sleep(2);

      std::vector<double> kinematics_pose;

      kinematics_pose.push_back(temp.x);
      kinematics_pose.push_back(temp.y);
      kinematics_pose.push_back(temp.z);

      printf("temp_x1 : %f",temp.x);
      printf("temp_y1 : %f",temp.y);
      printf("temp_x1 : %f",temp.z);
      
      if(!setTaskSpacePath(kinematics_pose, PATH_TIME)){
        printf("can not solve IK1\n");
        
        moveInit();
        sleep(2);

        std::vector<double> kinematics_pose1;

        kinematics_pose1.push_back(temp.x);
        kinematics_pose1.push_back(temp.y);
        kinematics_pose1.push_back(temp.z);


        if(!setTaskSpacePath(kinematics_pose1, PATH_TIME)){
          printf("can not solve IK2\n");
          moveDetect();
          sleep(2);
          return;
        }
        else{
          sleep(2);
          closeGripper();
          sleep(2);
          kinematics_pose.clear();
          kinematics_pose.push_back(temp.x - 0.100);
          kinematics_pose.push_back(temp.y);
          kinematics_pose.push_back(temp.z);
          if(!setTaskSpacePath(kinematics_pose, PATH_TIME)){
            printf("can not solve IK3");
            afterGrip();
            coordinate_.msg_seq = 0;
            return;
          }
          else{
            afterGrip();
            coordinate_.msg_seq = 0;
          }
        }
        return;
      }
      else{
        sleep(2);
        closeGripper();
        sleep(2);
        kinematics_pose.clear();
        kinematics_pose.push_back(temp.x - 0.100);
        kinematics_pose.push_back(temp.y);
        kinematics_pose.push_back(temp.z);
        if(!setTaskSpacePath(kinematics_pose, PATH_TIME)){
          printf("can not solve IK4");
          afterGrip();
          coordinate_.msg_seq = 0;
          return;
        }
        else{
          afterGrip();
          coordinate_.msg_seq = 0;
        }
      }
    }
    else{
      printf("no detect");
    }
}

void OpenManipulatorMoving::placeTomato(void){
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(1.5);
    joint_name.push_back("joint2"); joint_angle.push_back(-0.4);
    joint_name.push_back("joint3"); joint_angle.push_back(0);
    joint_name.push_back("joint4"); joint_angle.push_back(1.8);
    setJointSpacePath(joint_name, joint_angle, path_time);
    
    sleep(2);
    openGripper();
}

void OpenManipulatorMoving::grapTomato(void){
    // coordinate
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(1.5);
    joint_name.push_back("joint2"); joint_angle.push_back(-0.4);
    joint_name.push_back("joint3"); joint_angle.push_back(0);
    joint_name.push_back("joint4"); joint_angle.push_back(1.8);
    setJointSpacePath(joint_name, joint_angle, path_time);

    sleep(2);    
    closeGripper();
}

void OpenManipulatorMoving::openGripper(void){
    std::vector<double> joint_angle;
    joint_angle.push_back(0.01);
    setToolControl(joint_angle);
}

void OpenManipulatorMoving::closeGripper(void){
    std::vector<double> joint_angle;
    joint_angle.push_back(-0.008);
    setToolControl(joint_angle);
}

void OpenManipulatorMoving::showCoordinate(void){
    printf("------\n");
    printf("Get_date\n");
    printf("Start Time(sec): %d\n", coordinate_.start_time);
    printf("Message Sequence: %d\n", coordinate_.msg_seq);
    printf("X: %f\n", coordinate_.x);
    printf("Y: %f\n", coordinate_.y);
    printf("Z: %f\n\n", coordinate_.z);
}

void OpenManipulatorMoving::printText()
{
  printf("\n");
  printf("g : gripper open\n");
  printf("f : gripper close\n");
  printf("       \n");
  printf("s : detecting start\n");
  printf("1 : init pose\n");
  printf("2 : home pose\n");
  printf("       \n");
  printf("3 : custom_pose\n");
  printf("       \n");
  printf("q to quit\n");
  printf("---------------------------\n");
  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
         getPresentJointAngle().at(0),
         getPresentJointAngle().at(1),
         getPresentJointAngle().at(2),
         getPresentJointAngle().at(3));
  printf("Present Kinematics Position X: %.3lf Y: %.3lf Z: %.3lf\n",
         getPresentKinematicsPose().at(0),
         getPresentKinematicsPose().at(1),
         getPresentKinematicsPose().at(2));
  printf("---------------------------\n");

}

void OpenManipulatorMoving::setGoal(char ch)
{
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  std::vector<double> goalJoint; goalJoint.resize(NUM_OF_JOINT, 0.0);

  if (ch == 'g' || ch == 'G')
  {
    printf("input : g \topen gripper\n");
    openGripper();
  }
  else if (ch == 'f' || ch == 'F')
  {
    printf("input : f \tclose gripper\n");
    closeGripper();
  }
  else if (ch == 's' || ch == 'S')
  {
    printf("input : s \tdetecting start\n");
    detecting_start();
  }
  else if (ch == '2')
  {
    printf("input : 2 \thome pose\n");
    moveHome();
  }
  else if (ch == '1')
  {
    printf("input : 1 \tinit pose\n");
    moveInit();
  }
  else if (ch == '3')
  {
    printf("input : 3 \tcustom pose\n");
    movePath();
  }
  else if (ch == '9')
  {
    move9();   
  }
  else if (ch == '0')
  {
    move0();
  }

}

void OpenManipulatorMoving::restoreTerminalSettings(void)
{
  tcsetattr(0, TCSANOW, &oldt_);  /* Apply saved settings */
}

void OpenManipulatorMoving::disableWaitingForEnter(void)
{
  struct termios newt;

  tcgetattr(0, &oldt_);  /* Save terminal settings */
  newt = oldt_;  /* Init new settings */
  newt.c_lflag &= ~(ICANON | ECHO);  /* Change settings */
  tcsetattr(0, TCSANOW, &newt);  /* Apply settings */
}

// global variable 
u_char detecting_mode = 0;



int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_move_xyz");
  OpenManipulatorMoving openManipulatorMoving;
  
  openManipulatorMoving.moveDetect();
  sleep(2);

  openManipulatorMoving.printText();

  char ch;
  // openManipulatorMoving.printText();
  ros::spinOnce();
  while (ros::ok() && (ch = std::getchar()) != 'q')
  // while (ros::ok())
  {
    ros::spinOnce();
    openManipulatorMoving.printText();
    ros::spinOnce();
    openManipulatorMoving.setGoal(ch);
    
    // ros::spinOnce();
    
  }

  return 0;
}
