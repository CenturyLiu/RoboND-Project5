#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

double pick_odom_x = -12.0;
double pick_odom_y = -7.0;
double drop_odom_x = 6.0;
double drop_odom_y = 10.0;

double pick_marker_x = pick_odom_x;//-7.0;
double pick_marker_y = pick_odom_y;//12.0;
double drop_marker_x = drop_odom_x;//10.0;
double drop_marker_y = drop_odom_y;//-6.0;


bool ReachGoalPose(MoveBaseClient& ac, double x, double y, double w){
    move_base_msgs::MoveBaseGoal goal;

    // set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = w;

    // Send the goal position and orientation of the picking up point
    ROS_INFO("Sending robot to goal");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        return true;
    else
        return false;
}



int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "navigate_by_movebase");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }


  // let my_robot go to the pick up zone

  bool reach_pick_up = ReachGoalPose(ac, pick_odom_x, pick_odom_y, 0.7);
  if (reach_pick_up){
      ROS_INFO("Hooray, my_robot has reached the pick up zone!");
  }
  else{
      ROS_INFO("Robot failed to reach the pick up zone! Cancel mission");
  }

  if (reach_pick_up){
      // sleep for 5 seconds
      ros::Duration(5.0).sleep();


      // let my_robot go to the drop off zone, if it is now at the pick up zone
      reach_pick_up = ReachGoalPose(ac, drop_odom_x, drop_odom_y, 0.7);
      if (reach_pick_up){
          ROS_INFO("Hooray, my_robot has reached the drop off zone!");
      }
      else{
          ROS_INFO("Robot failed to reach the drop off zone! Cancel mission");
      }
  }







  return 0;
}
