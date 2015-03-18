/*
 * Local controller to follow a trajectory
 *
 * By: Eric Danziger
 * March 1, 2015
 */


#include "trajectory_controller/local_controller.h"


localController::localController()
{
  currentPose.position.x = 0;
  currentPose.position.y = 0;
  
}

localController::~localController()
{
}

void localController::publishMessage(ros::Publisher *pub_message)
{
  //Go through the path, find all poses within 1m of currentPose
  nav_msgs::Path pathWindow;
  double timeStep = .2;
  std::vector<double> dVel,dYaw;
  double closestDist = 10000;
  int closestIndex;
  geometry_msgs::Twist atDes, toDes;
  geometry_msgs::Twist msg;
  if(currentPath.poses.size() < 1)
    {
      pub_message->publish(msg);
      return;
    }
 for(int i=0;i<currentPath.poses.size();i++)
    {
      if (abs(currentPath.poses[i].pose.position.x - currentPose.position.x) < 1 && abs(currentPath.poses[i].pose.position.y - currentPose.position.y) < 1)
	{
	  pathWindow.poses.push_back(currentPath.poses[i]);
	  // ROS_INFO("Pushed a new path onto the path window %d",i);
	}


    }

 for(int i=0;i<(pathWindow.poses.size()-1);i++)
    {
	  //find closest Position
	  //posXdiff
	  double posXdiff = pathWindow.poses[i].pose.position.x - currentPose.position.x;
	  //posYdiff
	  double posYdiff = pathWindow.poses[i].pose.position.y - currentPose.position.y;
	  double poseDiff = sqrt(posXdiff*posXdiff + posYdiff*posYdiff);
	  if(poseDiff < closestDist) 
	    {
	      closestDist = poseDiff;
	      closestIndex = i;
	    }
       

	  //xdiff
	  double xdiff = pathWindow.poses[i+1].pose.position.x - pathWindow.poses[i].pose.position.x;
	  //ydiff
	  double ydiff = pathWindow.poses[i+1].pose.position.y - pathWindow.poses[i].pose.position.y;
	  
	  //desired velocity along the line
	  dVel.push_back( sqrt(xdiff*xdiff + ydiff*ydiff)/timeStep );

	  //desired yaw to match the line
	  dYaw.push_back( atan(ydiff/xdiff) );

	  //	  ROS_INFO("desired yaw:  %d",dYaw.back());
	

    }

  //make sure closestIndex isn't the first point
  if (!closestIndex) closestIndex = 1;
   ROS_INFO("Closest index %d",closestIndex);
  //figure out the desired twist at the desired point
  double xVel = (dVel[closestIndex-1] + dVel[closestIndex]) /2;
  double aVel = (dYaw[closestIndex-1] - dYaw[closestIndex])/timeStep;
  ROS_INFO("desired linear Vel: %f",xVel);
  ROS_INFO("desired angular Vel: %f",aVel);
  //figure out the twist to get to the desired point
  //xdiff
  double xdiff = pathWindow.poses[closestIndex].pose.position.x - currentPose.position.x;
  //ydiff
  double ydiff = pathWindow.poses[closestIndex].pose.position.y - currentPose.position.y;
  //desired velocity along the line
  double desVel =  sqrt(xdiff*xdiff + ydiff*ydiff)/timeStep ;
  //current yaw
  double roll,pitch,yaw;
  //currentPose.orientation.getRPY(roll,pitch,yaw);
  //desired yaw to match the line 
  double desAng =  (atan(ydiff/xdiff)-yaw) / timeStep; 
  //average them to create the twist to publish 
  msg.linear.x = (desVel + xVel) / 2; 
  msg.angular.z = (desAng + aVel) / 2;
  pub_message->publish(msg);

}

void localController::pathMessageCallback(const nav_msgs::Path::ConstPtr &msg)
{
  //get a path

  //update current path
  currentPath = *msg;
  pathCounter = 0;

  ROS_INFO("Got a New Path\n");
}

void localController::odomUpdateCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  //update position in worldFrame
  currentVel  = msg->twist.twist;
  currentPose = msg->pose.pose;
  ROS_INFO("Got a New Odom\n");
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "locom");
  ros::NodeHandle nh;

  localController *local_controller = new localController();
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
  ros::Subscriber odom_sub = nh.subscribe("/odom",1000,&localController::odomUpdateCallback,local_controller);
  ros::Subscriber path_sub = nh.subscribe("/plan",1000,&localController::pathMessageCallback,local_controller);
  ros::Rate r(10);

  while(nh.ok())
    {
      local_controller->publishMessage(&pub);
      r.sleep();
      ros::spinOnce();
    }

  return 0;
}
