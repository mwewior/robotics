#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <global_planner/planner_core.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <clear_costmap_recovery/clear_costmap_recovery.h>
#include <nav_msgs/Odometry.h>
#include <rotate_recovery/rotate_recovery.h>

#include <string>


std::vector<geometry_msgs::PoseStamped> plan;
geometry_msgs::PoseStamped current_position;
geometry_msgs::PoseStamped goal_position;
geometry_msgs::PoseStamped start_position;
geometry_msgs::Twist cmd_vel;
bool move = false;


void goalPositionRecieverCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{   
    
    current_position.header.frame_id = "map";
    start_position.header.frame_id = "map";
    start_position = current_position;
    goal_position.pose = msg->pose;
    goal_position.header = msg->header;
    goal_position.header.frame_id = "map";
    move = true;

}


// void currentPositionRecieverCallback(const nav_msgs::Odometry::ConstPtr& msg)
// {
    
//     current_position.header = msg->header;
//     current_position.header.frame_id = "map";
//     current_position.pose.position = msg->pose.pose.position;
//     current_position.pose.orientation = msg->pose.pose.orientation;

// }

int main(int argc, char **argv){

    ros::init(argc, argv, "path_executor");
    ros::NodeHandle n;

    ros::Rate loop_rate(10);


    ros::Subscriber goal_position_reciever = n.subscribe("move_base_simple/goal", 1000, goalPositionRecieverCallback);
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/nav_vel", 1000);
    

    tf2_ros::Buffer tfBuffer(ros::Duration(10));
    tf2_ros::TransformListener tf(tfBuffer);

    costmap_2d::Costmap2DROS glob_costmap("global_costmap", tfBuffer);
    costmap_2d::Costmap2DROS loc_costmap("local_costmap", tfBuffer);

    global_planner::GlobalPlanner glob_planner("global_planner", glob_costmap.getCostmap(), "map");

    base_local_planner::TrajectoryPlannerROS loc_planner;
    loc_planner.initialize("local_planner", &tfBuffer, &loc_costmap);

    rotate_recovery::RotateRecovery rr;
    rr.initialize("rotate_recovery", &tfBuffer, &glob_costmap, &loc_costmap);


    while (ros::ok())
    {
        glob_costmap.getRobotPose(current_position);
        if (move)
        {
            glob_planner.makePlan(current_position, goal_position, plan);
            if (plan.size() == 0)
            {
                rr.runBehavior();
            }
            loc_planner.setPlan(plan);
            if (plan.size() == 0)
            {
                rr.runBehavior();
            }
            if (loc_planner.computeVelocityCommands(cmd_vel))
            {
                velocity_publisher.publish(cmd_vel);
            }
            if (loc_planner.isGoalReached())
            {
                move = false;
            }
            
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    

    return 0;
}



// #include "ros/ros.h"

// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Twist.h>
// #include <nav_msgs/Odometry.h>

// #include <tf/transform_listener.h>
// #include <costmap_2d/costmap_2d_ros.h>
// #include <dwa_local_planner/dwa_planner_ros.h>
// #include <nav_core/base_global_planner.h>
// #include <global_planner/planner_core.h>




// std::vector<geometry_msgs::PoseStamped> plan;

// geometry_msgs::PoseStamped current_robot_pose;

// geometry_msgs::PoseStamped start_pose, goal_pose;

// bool goal_set = false;



// void odomCallback(nav_msgs::Odometry odom_msg) {

//     odom_msg.header.frame_id = "map";
//     current_robot_pose.header = odom_msg.header;
//     current_robot_pose.pose.position = odom_msg.pose.pose.position;
//     current_robot_pose.pose.orientation = odom_msg.pose.pose.orientation;

// }


// void navGoalCallback(geometry_msgs::PoseStamped msg){

//     msg.header.frame_id = "map";
//     current_robot_pose.header.frame_id = "map";
//     start_pose = current_robot_pose;
//     start_pose.header.frame_id = "map";
//     goal_pose = msg;
//     goal_set = true;

// }



// int main(int argc, char **argv){

//     ros::init(argc, argv, "Navigation");


//     ros::NodeHandle n;
//     int rate = 10;

//     ros::Rate loop_rate(rate);


//     tf2_ros::Buffer buffer(ros::Duration(10));
//     tf2_ros::TransformListener tf(buffer);


//     costmap_2d::Costmap2DROS globalCostmap("global_costmap", buffer);
//     costmap_2d::Costmap2DROS localCostmap("local_costmap", buffer);

//     global_planner::GlobalPlanner globalPlanner("global_planner", globalCostmap.getCostmap(), "map");

//     dwa_local_planner::DWAPlannerROS dp;   
//     dp.initialize("my_dwa_planner", &buffer, &localCostmap);

  

//     ros::Subscriber set_goal = n.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000, navGoalCallback);

//     ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/mobile_base_controller/odom", 1000, odomCallback);

//     /* Tiago nasluchuje na navel*/
//     ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/nav_vel", 1000);


//     start_pose.header.frame_id = "map";

//     geometry_msgs::Twist cmd_vel;



//     while (ros::ok())
//     {
//         // sprawdzac ze goal set byl ustawiony
//         if(goal_set){

//             globalPlanner.makePlan(current_robot_pose, goal_pose, plan);
//             dp.setPlan(plan);

//             if(dp.computeVelocityCommands(cmd_vel)){

//                 // Add this after dp.computeVelocityCommands(cmd_vel);
//                 ROS_INFO("Computed velocity commands: linear=(%f, %f), angular=%f", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

//                 vel_pub.publish(cmd_vel);
//             }

//         }

//         ros::spinOnce();
//         loop_rate.sleep();

//     }

//     return 0;
// }