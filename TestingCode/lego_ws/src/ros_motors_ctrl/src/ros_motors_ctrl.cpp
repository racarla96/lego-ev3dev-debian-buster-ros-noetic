// https://ev3dev-lang.readthedocs.io/en/latest/
#include "ev3dev.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h" // http://wiki.ros.org/std_msgs

#include <iostream>
#include <chrono>

using namespace std;
using namespace ev3dev;

ev3dev::large_motor _motor_left(ev3dev::OUTPUT_A);
ev3dev::large_motor _motor_right(ev3dev::OUTPUT_B);
float set_ang_speed_motor_left, set_ang_speed_motor_right = 0;

void motorsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (msg->data.size() == 2){
        set_ang_speed_motor_left = msg->data[0];
        set_ang_speed_motor_right = msg->data[1];
    }
}

int main(int argc, char **argv)
{
    if (!(_motor_left.connected() && _motor_right.connected())){
        std::cout << "Motors not found" << std::endl;
        return -1;
    }

    ros::init(argc, argv, "robot");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("set_ang_vel", 1, motorsCallback);

    ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("get_ang_vel", 1);
    ros::Rate loop_rate(10);

    std_msgs::Float32MultiArray msg; // Crear el objeto del mensaje fuera del bucle

    float cpr = 360; // Counts Per Revolution
    float T = 0.1; // Sample Time (s)
    float scale_factor = 2; // No se porque, pero va a la mitad
    float get_ang_speed_motor_left, get_ang_speed_motor_right = 0;
    int counts_motor_left, counts_motor_right = 0;
    int ref_counts_motor_left, ref_counts_motor_right = 0;

    while (ros::ok())
    {
        counts_motor_left = _motor_left.speed();
        counts_motor_right = _motor_right.speed();

        get_ang_speed_motor_left = ((float) counts_motor_left) / (cpr * T * scale_factor);
        get_ang_speed_motor_right = ((float) counts_motor_right)  / (cpr * T * scale_factor);

        ref_counts_motor_left = (int) (set_ang_speed_motor_left * cpr * T * scale_factor); 
        ref_counts_motor_right = (int) (set_ang_speed_motor_right * cpr * T * scale_factor);

        _motor_left.set_speed_sp(ref_counts_motor_left);
        _motor_right.set_speed_sp(ref_counts_motor_right); 
        _motor_left.run_forever();
        _motor_right.run_forever();

        // Llenar el mensaje con los nuevos datos que deseas enviar
        msg.data.clear(); // Limpiar los datos anteriores (opcional)
        msg.data.push_back(get_ang_speed_motor_left);
        msg.data.push_back(get_ang_speed_motor_right);
        pub.publish(msg); // Publicar el mensaje en el tópico "get_ang_vel"

        ros::spinOnce();

        loop_rate.sleep();
    }

    _motor_left.set_speed_sp(0);
    _motor_right.set_speed_sp(0); 
    _motor_left.run_forever();
    _motor_right.run_forever();

    return 0;
}
