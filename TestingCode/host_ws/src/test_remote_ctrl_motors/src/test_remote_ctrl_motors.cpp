#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h" // http://wiki.ros.org/std_msgs

#include <iostream>
#include <math.h>

using namespace std;

void motorsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (msg->data.size() == 2){
        float left_speed = msg->data[0];
        float right_speed = msg->data[1];
        std::cout << "Left speed: " << left_speed << ", Right speed: " << right_speed << std::endl;                 
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

    float V = 0.1; // Linear Speed (m/s)
    float W = 0;   // Angular Speed (rad/s)
    float b = 0.068; // Half Track Width (m)
    float r = 0.028; // Wheel Radius (m)
    float w_r, w_l = 0;

    while (ros::ok())
    {
        w_r = ((V + W * b) / r);
        w_l = ((V - W * b) / r);

        // Llenar el mensaje con los nuevos datos que deseas enviar
        msg.data.clear(); // Limpiar los datos anteriores (opcional)
        msg.data.push_back(w_r);
        msg.data.push_back(w_l);

        pub.publish(msg); // Publicar el mensaje en el tópico "get_ang_vel"

        ros::spinOnce();

        loop_rate.sleep();
    }

    msg.data.clear(); // Limpiar los datos anteriores (opcional)
    msg.data.push_back(0);
    msg.data.push_back(0);

    pub.publish(msg); // Publicar el mensaje en el tópico "get_ang_vel"

    ros::spinOnce();

        msg.data.clear(); // Limpiar los datos anteriores (opcional)
    msg.data.push_back(0);
    msg.data.push_back(0);

    pub.publish(msg); // Publicar el mensaje en el tópico "get_ang_vel"

    ros::spinOnce();

    return 0;
}
