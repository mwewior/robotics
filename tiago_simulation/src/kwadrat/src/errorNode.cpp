#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>

#include <stdio.h>



using namespace std;



// czy wyświetlać na bieżąco chwilowy błąd
bool show_cur_error = false;


// odometria:
float vel_x;
float pos_x_odom;
float pos_y_odom;


// prędkości zadane
float set_speed_linear;


// symulacja
float pos_x_simulation;
float pos_y_simulation;



// obecny stan
int state = -100;



void odomCallback(const nav_msgs::Odometry& msg){

    vel_x       =   msg.twist.twist.linear.x;
    
    pos_x_odom  =   msg.pose.pose.position.x;
    pos_y_odom  =   msg.pose.pose.position.y;

}


void setSpeedCallback(const geometry_msgs::Twist& msg){

    set_speed_linear = msg.linear.x;

}


void gazeboCallback(const gazebo_msgs::ModelStates& msg){

    pos_x_simulation = msg.pose[1].position.x;
    pos_y_simulation = msg.pose[1].position.y;

}


void statesCallback(const std_msgs::Int16& msg){

    state = msg.data;

}



int main(int argc, char *argv[])
{

    const int rate = 100; // rate w Hz


    ros::init(argc, argv, "errorNode");
    ros::NodeHandle nodeHandler;
    ros::Rate loop_rate(rate);



    // systemowe subskrybcje
    ros::Subscriber odom;
    odom = nodeHandler.subscribe("mobile_base_controller/odom", 1, odomCallback);

    ros::Subscriber set_speed;
    set_speed = nodeHandler.subscribe("key_vel", 1, setSpeedCallback);

    ros::Subscriber gazebo;
    gazebo = nodeHandler.subscribe("gazebo/model_states", 1, gazeboCallback);



    // publisherzy i subscriber na potrzeby komunikacji
    ros::Publisher error_current_pub;
    error_current_pub = nodeHandler.advertise<std_msgs::Float32>("error_current_topic", 1);

    ros::Publisher error_speed_pub;
    error_speed_pub = nodeHandler.advertise<std_msgs::Float32>("error_speed_topic", 1);

    ros::Publisher error_side_pub;
    error_side_pub = nodeHandler.advertise<std_msgs::Float32>("error_side_topic", 1);

    ros::Publisher error_total_pub;
    error_total_pub = nodeHandler.advertise<std_msgs::Float32>("error_total_topic", 1);

    ros::Subscriber states_sub;
    states_sub = nodeHandler.subscribe("state_topic", 1, statesCallback);




    std_msgs::Float32 position_error_current;
    std_msgs::Float32 speed_error_current;
    std_msgs::Float32 side_error;
    std_msgs::Float32 total_error;


    float cur_pose_error = 0.0f;
    float cur_speed_error = 0.0f;
    float side_error_pose = 0.0f;
    float total_error_pose = 0.0f;


    position_error_current.data = cur_pose_error;
    error_current_pub.publish(position_error_current);

    speed_error_current.data = cur_speed_error;
    error_total_pub.publish(speed_error_current);

    side_error.data = side_error_pose;
    error_side_pub.publish(side_error);

    total_error.data = total_error_pose;
    error_total_pub.publish(total_error);


    
    int prev_state = -100;


    float x_error = 0.0f;
    float y_error = 0.0f;


    while (ros::ok())
    {

        x_error = fabs(pos_x_odom - pos_x_simulation);
        y_error = fabs(pos_y_odom - pos_y_simulation);

        cur_pose_error = x_error + y_error;
        cur_speed_error = fabs(vel_x - set_speed_linear);


        if(state != prev_state)
        {

            ROS_INFO("side error of location: %f.\n", side_error_pose);
            side_error_pose = 0;                    // jeden bok

            if(state % 4 == 0)
            {
                ROS_INFO("total error of location: %f,\n", total_error_pose);
                total_error_pose = 0;               // cały kwadrat
            }


            prev_state = state;

        }
        else// (state==prev_state)
        {

            side_error_pose  += cur_pose_error;     // jeden bok

            total_error_pose += cur_pose_error;     // cały kwadrat

        }



        position_error_current.data = cur_pose_error;
        error_current_pub.publish(position_error_current);

        speed_error_current.data = cur_speed_error;
        error_speed_pub.publish(speed_error_current);

        side_error.data = side_error_pose;
        error_side_pub.publish(side_error);

        total_error.data = total_error_pose;
        error_total_pub.publish(total_error);
    


        if(show_cur_error){
            ROS_INFO("current error of position: %f,", cur_pose_error);
            ROS_INFO("current error of speed: %f.\n", cur_speed_error);
        }
        
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;

}
