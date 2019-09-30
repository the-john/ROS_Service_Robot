#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>		//Goal position
#include <actionlib/client/simple_action_client.h>	//Comand to navigate to goal

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  // Tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up ...");
  }

  move_base_msgs::MoveBaseGoal goal;

  // Send a goal to the robot to move 1 meter forward

  // Set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 1.0;
  //goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal for new location ...");
  ac.sendGoal(goal);

  // Wait for the result
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot reached destination of 1 meter forward");
  else
    ROS_INFO("The robot failed to move forward!");

  return 0;
}

