#include <ros/ros.h>

#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>

#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <yaml-cpp/yaml.h>

#include <stdio.h>
#include <cmath>


using namespace std;



float vel_x;
float ang_z;

float pos_x;
float pos_y;

float orient_x;
float orient_y;
float orient_z;
float orient_w;

double rotation_z;

const float PI = 3.14159;




void odomCallback(const nav_msgs::Odometry& msg){

    vel_x       =   msg.twist.twist.linear.x;
    ang_z       =   msg.twist.twist.angular.z;
    
    pos_x       =   msg.pose.pose.position.x;
    pos_y       =   msg.pose.pose.position.y;

    orient_x    =   msg.pose.pose.orientation.x;
    orient_y    =   msg.pose.pose.orientation.y;
    orient_z    =   msg.pose.pose.orientation.z;
    orient_w    =   msg.pose.pose.orientation.w;

    tf2::Quaternion quat(orient_x, orient_y, orient_z, orient_w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    rotation_z = yaw;

}


void errorCallback(const std_msgs::Float32& msg){}




int main(int argc, char *argv[])
{

    const int rate = 100; // rate w Hz


    ros::init(argc, argv, "kwadrat");
    ros::NodeHandle nodeHandler;
    ros::Rate loop_rate(rate);



    ros::Publisher pub;
    pub = nodeHandler.advertise<geometry_msgs::Twist>("/key_vel", 1);
    geometry_msgs::Twist goal;

    ros::Subscriber odom;
    odom = nodeHandler.subscribe("mobile_base_controller/odom", 1, odomCallback);



    ros::Subscriber error_current;
    error_current = nodeHandler.subscribe("error_current_topic", 1, errorCallback);

    ros::Subscriber error_speed;
    error_speed = nodeHandler.subscribe("error_speed_topic", 1, errorCallback);

    ros::Subscriber error_side;
    error_side = nodeHandler.subscribe("error_side_topic", 1, errorCallback);

    ros::Subscriber error_total;
    error_total = nodeHandler.subscribe("error_total_topic", 1, errorCallback);
    

    ros::Publisher state_machine;
    state_machine = nodeHandler.advertise<std_msgs::Int16>("state_topic", 1);
    std_msgs::Int16 send_state;



    ros::Duration(2).sleep(); 
    ROS_INFO("Initialising...\n\n");



    // zerowanie ruchu robota:
    goal.linear.x = 0.0;
    goal.linear.y = 0.0;
    goal.linear.z = 0.0;

    goal.angular.x = 0.0;
    goal.angular.y = 0.0;
    goal.angular.z = 0.0;



    // wczytywanie parametrów ruchu:
    bool trapezoid_mode;
    nodeHandler.getParam("trapezoid_mode", trapezoid_mode);

    bool clockwise_direction;
    nodeHandler.getParam("clockwise_direction", clockwise_direction);

    float length;
    nodeHandler.getParam("length", length);

    float velocity;
    nodeHandler.getParam("velocity", velocity);

    if (velocity < 0.1) velocity = 0.1;       // minimal value of velocity
    else if (velocity > 1.0) velocity = 1.0;  // maximum value of velocity
    float const vel = velocity;
    
    // float omega;
    // nodeHandler.getParam("omega", omega);
    const float omega = 1.0;


    bool oneThirdAcceleration = false;

    int i = 0;
    float t_max = 2*length/(vel*3);
    float acceleration = vel/t_max;
    float acc_len = length/3;


    if(!oneThirdAcceleration){
        nodeHandler.getParam("acceleration", acceleration);
        acc_len = (vel*vel) / (2*acceleration);
    }

    acceleration = acceleration/rate;
    


    // ustawienie stałych zatrzymania ruchu:
    const float *line_offset_ptr = new float;
    if (vel > 0.45) {
        line_offset_ptr = new float(0.8 * (vel - 0.32));
    }
    else {
        line_offset_ptr = new float(0.3 * (vel - 0.1));
    }

    const float linear_offset = *line_offset_ptr;
    const float angular_offset = 0.259;
    const float stop_vel = 0.05;



    // wyświetlenie parametrów:
    ROS_INFO("Trapezoid change of velocity: %s,", trapezoid_mode?"true":"false");
    ROS_INFO("Direction is clockwise: %s,", clockwise_direction?"true":"false");
    ROS_INFO("Set side lenth: %f,", length);
    ROS_INFO("Maximum velocity: %f,", vel);
    ROS_INFO("Acceleration: %f,", acceleration);
    ROS_INFO("Angular velocity: %f,", omega);
    ROS_INFO("Starting position (x, y): %f, %f.\n\n", pos_x, pos_y);



    /*
    
    OPIS DZIAŁANIA AUTOMATU STANÓW
    
    Ruch:
      -przeciwny do wskazówek zegara
        -automat stanów:
          0 -> 1 -> 10 -> 2 -> 20 -> 3 -> 30 -> 4 -> 40 -> 1 -> ...

      -zgodny z ruchem wskazówek zegara
        -automat stanów:
          0 -> 2 -> 21 -> 1 -> 11 -> 4 -> 41 -> 3 -> 31 -> 2 -> ...
    
    Stany:
      -1, 2, 3, 4 odnoszą się do ruchu w odpowiednim kierunku:
        1) x+
        2) y+
        3) x-
        4) y-
      
      -10, 11, 20, 21, ..., 41
        pary tej samej dziesiątki np. 10 i 11 odnoszą się do tego, po którym stanie są wywoływane,
        liczba jedności 0 oznacza stan, gdy jesteśmy w ruchu przeciwnym do ruchu wskazówek zegara
        a 1 do ruchu zgodnego z ruchem wskazówek (0 odnosi się do flagi False, przy zmiennej
        clockwise_direction, analogicznie 1, gdy jest flaga ta jest True).

    */



    int state = 0;
    int cur_state = -1;
    int prev_state = -1;
    

    while (ros::ok()){


        if (state != prev_state){
            ROS_INFO("current state: %d\n", state);
        }

        switch (state){

            case 0:{

                if (clockwise_direction){

                    prev_state = state;
                    goal.angular.z = omega*0.9;

                    if (rotation_z >= PI/2 - angular_offset){
                        goal.angular.z = 0.01;
                    }

                    if (rotation_z >= PI/2){
                        goal.angular.z = 0.0;
                        state = 2;
                    }

                    pub.publish(goal);
                    break;

                }
                else{

                    state = 1;
                    break;
                }
            }

            case 1: {

                if(prev_state != state){
                    cur_state++;
                    send_state.data = cur_state;
                    state_machine.publish(send_state);
                }
                
                prev_state = state;

                if(!trapezoid_mode){ // skokowy

                    goal.linear.x = vel;
                    if (pos_x >= length - linear_offset){
                        goal.linear.x = 0.01;
                    }
                }
                else{ // trapezowy

                    float cur_pos = pos_x;

                    // przyspieszanie
                    if(cur_pos < acc_len){

                        float cur_vel = acceleration*i;
                        i = 1+i;
                        if (cur_vel <= 0.025){
                            cur_vel = 0.025;
                            
                        }
                        else if (cur_vel >= vel){
                            cur_vel = vel;
                        } 

                        goal.linear.x = cur_vel;
                    }

                    // stała prędkość
                    if(cur_pos >= acc_len && cur_pos <= (length - acc_len)){

                        goal.linear.x = vel;
                        i = 0;

                    }

                    // opóźnienie
                    if(cur_pos > length - acc_len){

                        float cur_vel = vel -  1.1*acceleration*i;
                        i = 1+i;
                        if (cur_vel <= 0.025){
                             cur_vel = 0.025;
                        }
                        else if (cur_vel >= vel){
                            cur_vel = vel;
                        } 

                        goal.linear.x = cur_vel;
                    }
                }

                if (pos_x >= length){
                    i = 0;
                    goal.linear.x = 0.0;

                    if(clockwise_direction)
                        state = 11;
                    else
                        state = 10;
                }

                pub.publish(goal);
                break;
            }

            case 10: { // counter clockwise

                prev_state = state;
                goal.angular.z = omega;

                if (rotation_z >= PI/2 - angular_offset){
                    goal.angular.z = 0.01;
                }

                if (rotation_z >= PI/2){
                    
                    goal.angular.z = 0.0;
                    state = 2;
                }

                pub.publish(goal);
                break;
            }

            case 11: { // clockwise

                prev_state = state;
                goal.angular.z = -omega;
            
                if (rotation_z <= -PI/2 + angular_offset){
                    goal.angular.z = -0.01;
                }
            
                if (rotation_z <= -PI/2){
            
                    goal.angular.z = 0.0;
                    state = 4;
                }
            
                pub.publish(goal);
                break;
            }

            case 2: {

                if(prev_state != state){
                    cur_state++;
                    send_state.data = cur_state;
                    state_machine.publish(send_state);
                }

                prev_state = state;

                if(!trapezoid_mode){

                    goal.linear.x = vel;
                    if (pos_y >= length - linear_offset){//0.54){
                        goal.linear.x = 0.01;
                    }

                }
                else{

                    float cur_pos = pos_y;

                    
                    if(cur_pos < acc_len){

                        float cur_vel = acceleration*i;
                        i = 1+i;
                        if (cur_vel <= 0.025){
                            cur_vel = 0.025;
                        }
                        else if (cur_vel >= vel){
                            cur_vel = vel;
                        } 

                        goal.linear.x = cur_vel;
                    }

                    if(cur_pos >= acc_len && cur_pos <= length - acc_len){
                        i = 0;
                        goal.linear.x = vel;

                    }

                    if(cur_pos > length - acc_len){

                        float cur_vel = vel -  1.1*acceleration*i;
                        i = 1+i;

                        if (cur_vel <= 0.025){
                            cur_vel = 0.025;
                        }
                        else if (cur_vel >= vel){
                            cur_vel = vel;
                        } 

                        goal.linear.x = cur_vel;
                    }
                }

                if (pos_y >= length){
                    i = 0;
                    goal.linear.x = 0.0;

                    if(clockwise_direction)
                        state = 21;
                    else
                        state = 20;
                        
                }

                pub.publish(goal);
                break;
            }

            case 20: { // counter clockwise

                prev_state = state;
                goal.angular.z = omega;

                if (rotation_z >= PI - angular_offset){
                    goal.angular.z = 0.01;
                }

                if (rotation_z >= PI*179/180 || rotation_z <= 0){

                    goal.angular.z = 0.0;
                    state = 3;
                }

                pub.publish(goal);
                break;
            }

            case 21: { // clockwise

                prev_state = state;
                goal.angular.z = -omega;

                if (rotation_z <= angular_offset){
                    goal.angular.z = -0.01;
                }

                if (rotation_z <= PI*1/180 && rotation_z >= -PI*1/180){

                    goal.angular.z = 0.0;
                    state = 1;
                }

                pub.publish(goal);
                break;
            }

            case 3: {
                
                if(prev_state != state){
                    cur_state++;
                    send_state.data = cur_state;
                    state_machine.publish(send_state);
                }

                prev_state = state;

                if(!trapezoid_mode){

                    goal.linear.x = vel;

                    if (pos_x <= linear_offset){// 0.54){
                        goal.linear.x = 0.01;
                    }
                }
                else{

                    float cur_pos = abs(length - pos_x);

                    
                    if(cur_pos < acc_len){

                        float cur_vel = acceleration*i;
                        i = 1+i;
                        if (cur_vel <= 0.025){
                            cur_vel = 0.025;
                        }
                        else if (cur_vel >= vel){
                            cur_vel = vel;
                        } 
                        goal.linear.x = cur_vel;
                    }

                    if(cur_pos >= acc_len && cur_pos <= length - acc_len){
                        i = 0;
                        goal.linear.x = vel;

                    }

                    if(cur_pos > length - acc_len){

                        float cur_vel = vel -  1.1*acceleration*i;
                        i = 1+i;

                        if (cur_vel <= 0.025){
                            cur_vel = 0.025;
                        }
                        else if (cur_vel >= vel){
                            cur_vel = vel;
                        } 

                        goal.linear.x = cur_vel;
                    }
                }

                if (pos_x <= 0.0){
                    i = 0;
                    goal.linear.x = 0.0;

                    if(clockwise_direction)
                        state = 31;
                    else
                        state = 30;
                }
                
                pub.publish(goal);
                break;
            }

            case 30: { // counter clockwise

                prev_state = state;
                goal.angular.z = omega;
                
                if (rotation_z >= -PI/2 - angular_offset){
                    goal.angular.z = 0.01;
                }

                if (rotation_z >= -1*PI*1/2 && rotation_z <= 0){

                    goal.angular.z = 0.0;
                    state = 4;
                }

                pub.publish(goal);
                break;
            }

            case 31: { // clockwise

                prev_state = state;
                goal.angular.z = -omega;

                if (rotation_z <= PI/2 + angular_offset && rotation_z > 0){
                    goal.angular.z = -0.01;
                }

                if (rotation_z <= PI/2 && rotation_z > 0){
                    
                    goal.angular.z = 0.0;
                    state = 2;
                }
                
                pub.publish(goal);
                break;
            }
            

            case 4: {

                if(prev_state != state){
                    cur_state++;
                    send_state.data = cur_state;
                    state_machine.publish(send_state);
                }

                prev_state = state;

                if(!trapezoid_mode){

                    goal.linear.x = vel;

                    if (pos_y <= linear_offset){// 0.54){
                        goal.linear.x = 0.01;
                    }
                }
                else{

                    float cur_pos = abs(length - pos_y);

                    // przyspieszanie
                    if(cur_pos < acc_len){

                        float cur_vel = acceleration*i;
                        i = 1+i;

                        if (cur_vel <= 0.025){
                            cur_vel = 0.025;
                        }
                        else if (cur_vel >= vel){
                            cur_vel = vel;
                        } 

                        goal.linear.x = cur_vel;
                    }

                    if(cur_pos >= acc_len && cur_pos <= length - acc_len){
                        i = 0;
                        goal.linear.x = vel;

                    }

                    if(cur_pos > length - acc_len){

                        float cur_vel = vel -  1.1*acceleration*i;
                        i = 1+i;
                        
                        if (cur_vel <= 0.025){
                            cur_vel = 0.025;
                        }
                        else if (cur_vel >= vel){
                            cur_vel = vel;
                        } 
                        
                        goal.linear.x = cur_vel;
                    }
                }
                
                if (pos_y <= 0.0){
                    i = 0;
                    goal.linear.x = 0.0;
                    
                    if(clockwise_direction)
                        state = 41;
                    else
                        state = 40;
                }
                
                pub.publish(goal);
                break;
            }

            case 40: { //counter clockwise

                prev_state = state;
                goal.angular.z = omega;

                if (rotation_z >= -angular_offset){
                    goal.angular.z = 0.01;
                }

                if (rotation_z >= -1*PI*1/180 && rotation_z <= PI*1/360){
                    
                    goal.angular.z = 0.0;
                    state = 1;
                }

                pub.publish(goal);
                break;
            }  

            
            case 41: { // clockwise

                prev_state = state;
                goal.angular.z = -omega;

                if (rotation_z <= -PI + angular_offset){
                    goal.angular.z = -0.01;
                }

                if (rotation_z <= -PI*179/180 || rotation_z >= PI*179/180){

                    goal.angular.z = 0.0;
                    state = 3;
                }

                pub.publish(goal);
                break;
            }          
        }


        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
