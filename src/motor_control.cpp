#include <sstream>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <ctime>
#include <pthread.h>
#include <std_msgs/Float32.h>
#include <cmath>


using namespace std;

serial::Serial ser; //声明串口对象
float speed,angle;
int is_cmd_valid = 1;  //指令是否有效标志

ros::Publisher* motor_pub_;


void receiveSpeed(geometry_msgs::Twist twist)
{
    speed = twist.linear.x ;
    angle = -twist.angular.z ;
}

void writeToPort( const float angle, const float speed )
{
    // Generate low-layer instructions
    unsigned char send[10];
    send[0] = 0xff;
    send[1] = 0xfe;

    // Speed control
    const int *p = (int *)&speed;
    send[5] = (*p>>24)&0xff;
    send[4] = (*p>>16)&0xff;
    send[3] = (*p>>8)&0xff;
    send[2] = (*p>>0)&0xff;

    // Angle control
    p = (int *)&angle;
    send[9] = (*p>>24)&0xff;
    send[8] = (*p>>16)&0xff;
    send[7] = (*p>>8)&0xff;
    send[6] = (*p>>0)&0xff;

    // Send to port
    ser.write(send,10);
}


/*
 *  手柄消息的回调函数，紧急停车保护指令
 */
void protect(std_msgs::Float32 stop)
{
    if (stop.data == 1)
        is_cmd_valid = 0;  //指令有效标志置0，车轮锁止
    else
        is_cmd_valid = 1;  //指令有效标志置１
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;
    ros::NodeHandle nh_param("~");

    ros::Subscriber speed_sub = n.subscribe<geometry_msgs::Twist>("smooth_cmd_vel",1, &receiveSpeed);
    //ros::Subscriber speed_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel",1, &receiveSpeed);
    ros::Subscriber is_cmd_valid_sub = n.subscribe<std_msgs::Float32>("stop",1, &protect);
    //ros::Subscriber joy_sub = n.subscribe<std_msgs::Float32>("cmd",1,&change_pwm);
  
    ros::Rate loop_rate(100);
  
    bool serial_error = false;
    //bool serial_open = false;
  
    try
    {
        //设置串口属性，并打开串口
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        //ser.setParity(serial::parity_even);
        //ser.setStopbits(serial::stopbits_one);
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        serial_error = true;
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    //检测串口是否已经打开，并给出提示信息
    if(ser.isOpen())
    {
        serial_error = false;
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }
    //test
    //timer1 = n.createTimer(ros::Duration(10),&callback1);
    //ring_ban();

    while (ros::ok())
    {
        if(is_cmd_valid  == 0)  //指令无效时，立即制动
        {
            //hard_braking();
        }
        else
        {
            writeToPort( angle, speed );
        }


        loop_rate.sleep(); //发送端保证频率
        ros::spinOnce();   //spinOnce函数调用的频率太少，就会导致队列溢出，一些callback函数就会被挤掉，导致没被执行到。
    }

    return 0;
}
