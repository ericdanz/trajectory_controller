/*
 * Global controller to follow a trajectory
 *
 * By: Christopher Dunkers, cmdunkers@cmu.edu
 * March 1, 2015
 */

#ifndef GLOBAL_CONTROLLER_H_
#define GLOBAL_CONTROLLER_H_

class global_controller {

  public:
    global_planner();
    set_goal();
 
  private:
    create_path();
    update_waypoints();
    nav_msgs::Path goal_waypoints;
    tf::TransformListener tf_listener;
    
    double step;
    geometry_msgs::PoseStamped cur_robot_pose;
    nav_msgs::Path plan;

    //alphas start with the constant term and increase 
    //ie a[0] + a[1]*t + a[2]*t^2 ...
    std::vector<std::vector<double>> x_alphas;
    std::vector<std::vector<double>> y_alphas;
    std::vector<std::vector<double>> h_alphas;

}


#endif
