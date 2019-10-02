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

  // Send a goal to the robot to move to first location, 1, 2, 1

  // Set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.position.y = 2.0;
  goal.target_pose.pose.orientation.w = 1.0;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal for pick-up location ...");
  ac.sendGoal(goal);

  // Wait for the result
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("The robot reached the pick-up location");
    // Robot made its goal, wait 5 seconds for pick-up
    ROS_INFO("Now waiting 5 seconds for the pickup");
    ros::Duration(5.0).sleep();

    // Define the position and orientation for the next destination for the robot to reah
    goal.target_pose.pose.position.x = -3.0;
    goal.target_pose.pose.position.y = -3.0;
    goal.target_pose.pose.orientation.w = 2.0;

    // Send the goal position and orienteation for the next destination
    ROS_INFO("Sending goal for drop-off location ...");
    ac.sendGoal(goal);

    // Wait for the result
    ac.waitForResult();

    // Check if the robot reached its second goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("The robot reached the drop-off location");
    // Now wait 5 seconds so that I can ensure that these messages are read
    ROS_INFO("Giving you time to read this");
    ros::Duration(5.0).sleep();
    }
    else
      ROS_INFO("The robot failed to reach its second destination!");
  }
  else
    ROS_INFO("The robot failed to move to first destination!");

  return 0;
}

