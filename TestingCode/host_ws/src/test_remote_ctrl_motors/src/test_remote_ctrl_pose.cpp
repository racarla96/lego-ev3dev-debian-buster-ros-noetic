#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h" // http://wiki.ros.org/std_msgs

#include <iostream>
#include <math.h>

using namespace std;

// Robot Constants
float b = 0.068; // Half Track Width (m)
float r = 0.028; // Wheel Radius (m)
float T = 0.1; // Sample Time (s)
float eps = 0.15; // Epsilon

float ang_speed_left, ang_speed_right = 0;
float x_V_left, x_V_right = 0;
float x_V, x_W, x_X, x_Y, x_Psi = 0; // Robot State
float s_V, s_W, s_X, s_Y, s_Psi = 0; // Robot Setpoint
float k_V = 0.2;
float k_W = 0.6;
float d; // distance

float sets_X[5] = {0.6, 0.6, 1.2, 1.2, 1.8};
float sets_Y[5] = {0,   0.6, 0.6,   0,   0};
int counter = 0;

void motorsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (msg->data.size() == 2){
        ang_speed_left = msg->data[0];
        ang_speed_right = msg->data[1];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "host");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("get_ang_vel", 1, motorsCallback);

    ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("set_ang_vel", 1);
    ros::Rate loop_rate(10);

    std_msgs::Float32MultiArray msg; // Crear el objeto del mensaje fuera del bucle

    float w_r, w_l = 0;

    while (ros::ok())
    {
        x_V_left = r * ang_speed_left;
        x_V_right = r * ang_speed_right;
        x_V = (x_V_right + x_V_left) / 2.0f;
        x_W = (x_V_right - x_V_left) / (2.0f * b);
        x_X = x_X + x_V * T * cos(x_Psi + x_W * T);
        x_Y = x_Y + x_V * T * sin(x_Psi + x_W * T);
        x_Psi = x_Psi + x_W * T;

        s_X = sets_X[counter];
        s_Y = sets_Y[counter];
        s_Psi = atan2(s_Y-x_Y, s_X-x_X);
        s_W = k_W * (x_Psi - s_Psi);
        d = sqrt(pow(s_X-x_X, 2) + pow(s_Y-x_Y, 2));
        s_V = k_V * d;

        ROS_INFO("State [%f, %f, %f]", x_X, x_Y, x_Psi);
        ROS_INFO("Objective [%f, %f]", s_X, s_Y);
        ROS_INFO("Distance [%f]", d);
        if (d < eps) 
        {
            counter++;
            if (counter >= 5){
                s_V = 0;
                s_W = 0;
                counter = 4;
            }
            //ROS_INFO("Estoy dentro?");
        }

        s_V = min(0.1f, s_V);

        s_W = min(0.2f, s_W);

        w_r = ((s_V + s_W * b) / r);
        w_l = ((s_V - s_W * b) / r);

        ROS_INFO("Reference velocities [%f, %f]", w_r, w_l);

        // Llenar el mensaje con los nuevos datos que deseas enviar
        msg.data.clear(); // Limpiar los datos anteriores (opcional)
        msg.data.push_back(w_r);
        msg.data.push_back(w_l);
        pub.publish(msg); // Publicar el mensaje en el tópico "get_ang_vel"

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
