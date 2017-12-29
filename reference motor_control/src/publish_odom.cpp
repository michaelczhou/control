
#include <sstream>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <motor_control/readDataAll.h>
#include <ctime>
#include <pthread.h>
#include <nav_msgs/Odometry.h>
#include<boost/thread.hpp>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/thread/condition_variable.hpp>
#include <limits>

#include <stdlib.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#define Thread_num 4
//#include <iomanip>
using namespace std;
serial::Serial ser[4]; //声明串口对象
serial::Serial ser_port;

motor_control::readDataAll odom_data;

ros::Publisher* odom_pub_;
ros::Publisher* pulse_pub_;
ros::Publisher* motor_pub_;
boost::thread* my_thread;

tf::TransformBroadcaster* odom_broadcaster_;
//double period = 0.0;
double scale = 0.000766;
double a = 0.25,b = 0.255;

double vel_[4];
int update_freq_ = 20;
//memset(cmd_, 0, sizeof cmd_);

bool init=false;

double pose_x = 0.0;
double pose_y = 0.0;
double pose_th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

double x=0.0;
double y=0.0;
double th=0.0;
double last_data[4]={0.0};
double updated_;


static const unsigned char aucCRCHi[] = {

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,

    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,

    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40

};

static const unsigned char aucCRCLo[] = {

    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,

0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,

    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,

    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,

    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,

    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,

    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,

    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,

    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,

    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,

    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,

    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,

    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,

    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,

    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,

    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,

    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,

    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,

    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,

    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,

    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,

    0x41, 0x81, 0x80, 0x40

};

// Protocols

int usMBCRC16(unsigned char * pucFrame, int usLen )

{

    unsigned char ucCRCHi = 0xFF;

    unsigned char ucCRCLo = 0xFF;

    int iIndex;

    while( usLen-- )

    {

        iIndex = ucCRCLo ^ *( pucFrame++ );

        ucCRCLo = ( unsigned char )( ucCRCHi ^ aucCRCHi[iIndex] );

        ucCRCHi = aucCRCLo[iIndex];

    }
//    cout << "0x" << hex << int(ucCRCLo) << " ";
//    cout << "0x" << hex << int(ucCRCHi) << endl;
//    cout << "0x" << hex << ( int )( ucCRCHi << 8 | ucCRCLo );
    return ( int )( ucCRCHi << 8 | ucCRCLo );

}


void Get_Odom(int ser_id)
{
    //int ser_id = *((int*)thread);
    if (ser_id >= 0 && ser_id <4)
    {
        while(1)
        {
        uint8_t send[8];
        int check;
        int check_temp;
        send[0] = 0x01;
        send[1] = 0x03;
        send[2] = 0x00;
        send[3] = 0x24;
        send[4] = 0x00;
        send[5] = 0x02;
        check = usMBCRC16(send,6);
        send[6] = check&0xff;
        send[7] = (check>>8)&0xff;
        //cout << "0x" << hex << int(send[6]) << " ";
        //cout << "0x" << hex << int(send[7]) << endl;

        ser[ser_id].write(send,8);

        vector<uint8_t> receive;
        ser[ser_id].read(receive,9); //每次发送完读命令，需要立刻读串口数据，否则会丢失

        uint8_t temp[9];
        int i;
        for(i = 0;i<7;i++)
        {
            temp[i] = receive[i];
        }
        check_temp = usMBCRC16(temp,7);

        if (receive[0] == 0x01 && receive[1] == 0x03)
        {
            if (receive.size() == 9)
            {
                //check_temp = usMBCRC16(receive,7);
                if (check_temp == receive[7]<<8|receive[8])
                {
                    if (ser_id == 0)
                        odom_data.odom1 =  (((receive[3]) << 24)|((receive[4]) << 16)|((receive[5]) << 8)|receive[6])*scale;
                    else if(ser_id == 1)
                        odom_data.odom2 =  (((receive[3]) << 24)|((receive[4]) << 16)|((receive[5]) << 8)|receive[6])*scale;
                    else if(ser_id == 2)
                        odom_data.odom3 =  (((receive[3]) << 24)|((receive[4]) << 16)|((receive[5]) << 8)|receive[6])*scale;
                    else
                        odom_data.odom4 =  (((receive[3]) << 24)|((receive[4]) << 16)|((receive[5]) << 8)|receive[6])*scale;

                    //long odom_temp;
                    //odom_temp = ((receive[3]) << 24)|((receive[4]) << 16)|((receive[5]) << 8)|receive[6];
                    //odom_pub_->publish(odom_data);
                    //odom = ((receive[3]) << 24)|((receive[4]) << 16)|((receive[5]) << 8)|receive[6];
                    //cout << "The odom is :" << odom_temp;
                }
            }
        }
//        nav_msgs::Odometry odom;
//        odom.header.stamp = updated_;
//        odom.header.frame_id = "odom";

//        odom.pose.pose.position.x = x;
//        odom.pose.pose.position.y = y;
//        odom.pose.pose.position.z = 0.0;
//        odom.pose.pose.orientation = odom_quat;

//        odom.child_frame_id = "base_link";
//        odom.twist.twist.linear.x = vx;
//        odom.twist.twist.linear.y = vy;
//        odom.twist.twist.angular.z = vth;
        }
    }
}

void pwm_output(int ser_id,float p)
{
    if (ser_id >=0 && ser_id <4)
    {
        if (p>= -1 and p<= 1)
        {
            int p_t;
            p_t = p*1000;
            uint8_t send_buff[8];
            int check;
            //wheel 1
            uint8_t send[8];
            send[0] = 0x01;
            send[1] = 0x06;
            send[2] = 0x00;
            send[3] = 0x42;
            send[4] = (p_t>>8)&0xff;
            send[5] = (p_t&0xff);
            check = usMBCRC16(send,6);
            send[6] = check&0xff;
            send[7] = (check>>8)&0xff;

            ser[ser_id].write(send,8);

            vector<uint8_t> receive;
            ser[ser_id].read(receive,9); //每次发送完读命令，需要立刻读串口数据，否则会丢失

            uint8_t temp[8];
            int check_temp;
            int i;
            bool state;
            for(i = 0;i<6;i++)
            {
                temp[i] = receive[i];
            }
            check_temp = usMBCRC16(temp,6);

            if (receive[0] == 0x01 && receive[1] == 0x06)
            {
                if (receive.size() == 8)
                {
                    state = true;
                }
            }
        }
    }
}

void motor(int id1,float p1,int id2,float p2,int id3,float p3,int id4,float p4)
{
    boost::thread send_thread1(&pwm_output,id1,p1);
    boost::thread send_thread2(&pwm_output,id2,p2);
    boost::thread send_thread3(&pwm_output,id3,p3);
    boost::thread send_thread4(&pwm_output,id4,p4);

    send_thread1.join();
    send_thread2.join();
    send_thread3.join();
    send_thread4.join();
}


void odom_test()
{
    uint8_t send[8];
    int check;
    int check_temp;
    send[0] = 0x01;
    send[1] = 0x03;
    send[2] = 0x00;
    send[3] = 0x24;
    send[4] = 0x00;
    send[5] = 0x02;
    check = usMBCRC16(send,6);
    send[6] = check&0xff;
    send[7] = (check>>8)&0xff;
    //cout << "0x" << hex << int(send[6]) << " ";
    //cout << "0x" << hex << int(send[7]) << endl;

    ser_port.write(send,8);

    vector<uint8_t> receive;
    ser_port.read(receive,9); //每次发送完读命令，需要立刻读串口数据，否则会丢失
    //cout << "receive" << " "<<  receive.size() << endl;


    uint8_t temp[9];
    int i;
    for(i = 0;i<7;i++)
    {
        temp[i] = receive[i];
    }
    check_temp = usMBCRC16(temp,7);

    if (receive[0] == 0x01 && receive[1] == 0x03)
    {
        if (receive.size() == 9)
        {
            //check_temp = usMBCRC16(receive,7);
            if (check_temp == receive[7]<<8|receive[8])
            {
                //long odom;
                odom_data.odom1 =  ((receive[3]) << 24)|((receive[4]) << 16)|((receive[5]) << 8)|receive[6];
                //cout << odom <<endl;
                odom_pub_->publish(odom_data);
                //odom = ((receive[3]) << 24)|((receive[4]) << 16)|((receive[5]) << 8)|receive[6];
            }
        }
    }
}


void hello(int id)
{
    if (id == 1)
    {
        std::cout <<
        "Hello world, I''m a thread!"
        << std::endl;
    }
    else if (id == 2)
    {

        std::cout << "I am the second thread~" << std::endl;
    }

    else
        std::cout << "invalid thread!" << std::endl;
}

void odom_publish()
{
//    boost::thread read_thread2(&(Get_Odom(1)));
//    boost::thread read_thread3(&(Get_Odom(2)));
//    boost::thread read_thread4(&(Get_Odom(3)));

//    read_thread1.join();
//    read_thread2.join();
//    read_thread3.join();
//    read_thread4.join();
//    Get_Odom(0);
//    Get_Odom(1);
//    Get_Odom(2);
//    Get_Odom(3);

    //pulse_pub_->publish(odom_data);

    if(!init)
    {
        last_data[0]=odom_data.odom1;
        last_data[1]=odom_data.odom2;
        last_data[2]=odom_data.odom3;
        last_data[3]=odom_data.odom4;

        updated_ = ros::Time::now().toSec();
        init=true;
        return;
    }

    double period = ros::Time::now().toSec() - updated_;
    updated_ = ros::Time::now().toSec();

    double vel1 =-(double)((double)(odom_data.odom1-last_data[0])/(double)period);
    double vel2 =-(double)((double)(odom_data.odom2-last_data[1])/(double)period);
    double vel3 =(double)((double)(odom_data.odom3-last_data[2])/(double)period);
    double vel4 =(double)((double)(odom_data.odom4-last_data[3])/(double)period);

//    cout << "vel1:" << vel1 << " ";
//    cout << "vel2:" << vel2 << " ";
//    cout << "vel3:" << vel3 << " ";
//    cout << "vel4:" << vel4 << endl;

    last_data[0]=(double)odom_data.odom1;
    last_data[1]=(double)odom_data.odom2;
    last_data[2]=(double)odom_data.odom3;
    last_data[3]=(double)odom_data.odom4;

    vel_[0] = vel1;
    vel_[1] = vel3;
    vel_[2] = vel4;
    vel_[3] = vel2;

    vx=(vel_[0]+vel_[1]+vel_[2]+vel_[3])/4.0;
    vy=0.0;
    vth=((vel_[0]-vel_[2])/2.0/(a+b)+(vel_[3]-vel_[1])/2.0/(a+b))/2.0;
    //vth=((vel_[0]-vel_[1])/2.0/(a+b)+(vel_[3]-vel_[2])/2.0/(a+b))/2.0;

    double delta_x = (vx * cos(th) - vy * sin(th))*period;
    double delta_y = (vx * sin(th) + vy * cos(th))*period;
    double delta_th = vth*period;

    //cout << "delta_th:" << delta_th << endl;

    pose_x += (double)delta_x;
    pose_y += (double)delta_y;
    pose_th += (double)delta_th;

    x+=(double)delta_x;
    y+=(double)delta_y;
    th = th + (double)delta_th;

    //cout << "th:" << th << endl;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster_->sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //odom_pub.publish(odom);
    odom_pub_->publish(odom);

}

//void change_pwm(std_msgs::Float32 joy)
//{
//    static float p  = 0;
//    if (joy.data == 1.0)
//    {
//        p += 0.1;
//        //(*my_thread)(&pwm_output,0,p);
//        my_thread(&hello,2);
//    }
//}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  //ros::Subscriber speed_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel",1, &receiveSpeed);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom",1,true);
  //ros::Publisher pulse_pub = n.advertise<motor_control::readDataAll>("/pulse_count",1,true);
  //ros::Publisher motor_pub = n.advertise<std_msgs::Bool>("motor",1,true);
  //ros::Subscriber speed_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel",1, &receiveSpeed);
  //ros::Subscriber joy_sub = n.subscribe<std_msgs::Float32>("cmd",1,&change_pwm);
  tf::TransformBroadcaster odom_broadcaster;
  odom_pub_ = &odom_pub;
  //pulse_pub_ = &pulse_pub;
  //motor_pub_ = &motor_pub;
  odom_broadcaster_ = &odom_broadcaster;
  ros::Rate loop_rate(100);

//  try
//  {
//  //设置串口属性，并打开串口
//      ser_port.setPort("/dev/ttyUSB0");
//      //ser[init_count].setPort("/dev/ttyUSB3");
//      ser_port.setBaudrate(115200);
//      ser_port.setParity(serial::parity_even);
//      ser_port.setStopbits(serial::stopbits_one);
//      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
//      ser_port.setTimeout(to);
//      ser_port.open();
//  }
//  catch (serial::IOException& e)
//  {
//      ROS_ERROR_STREAM("Unable to open port ");
//      return -1;
//  }
//  //检测串口是否已经打开，并给出提示信息
//  if(ser_port.isOpen())
//  {
//      ROS_INFO_STREAM("Serial Port initialized");
//  }
//  else
//  {
//      return -1;
//  }
  //serial::Serial ser; //声明串口对象


  int init_count;
  for(init_count =0 ;init_count < 4;init_count++)
  {
      try
      {
      //设置串口属性，并打开串口
          if (init_count == 0)
              ser[init_count].setPort("/dev/ttyUSB0");
          else if (init_count == 1)
              ser[init_count].setPort("/dev/ttyUSB1");
          else if (init_count == 2)
              ser[init_count].setPort("/dev/ttyUSB2");
          else
              ser[init_count].setPort("/dev/ttyUSB3");
          ser[init_count].setBaudrate(9600);
          ser[init_count].setParity(serial::parity_even);
          ser[init_count].setStopbits(serial::stopbits_one);
          serial::Timeout to = serial::Timeout::simpleTimeout(1000);
          ser[init_count].setTimeout(to);
          ser[init_count].open();
      }
      catch (serial::IOException& e)
      {
          ROS_ERROR_STREAM("Unable to open port ");
          return -1;
      }
      //检测串口是否已经打开，并给出提示信息
      if(ser[init_count].isOpen())
      {
          ROS_INFO_STREAM("Serial Port initialized");
      }
      else
      {
          return -1;
      }
  }

    boost::thread read_thread1(&Get_Odom,0);
    boost::thread read_thread2(&Get_Odom,1);
    boost::thread read_thread3(&Get_Odom,2);
    boost::thread read_thread4(&Get_Odom,3);

    //boost::thread send_thread1(&pwm_output,0,0.5);
    //my_thread = &send_thread1;

//      boost::thread send_thread2(&pwm_output,1,0.5);
//      boost::thread send_thread3(&pwm_output,2,0.5);
//      boost::thread send_thread4(&pwm_output,3,0.5);

    //boost::thread test_thread1(&hello,2);
    //my_thread = &test_thread1;
    //boost::thread test_thread2(&hello,2);

    //test_thread1.join();
  //pwm_output(0,0.5);
  //motor(0,0.2,1,0.4,2,0.6,3,0.8);
  while (ros::ok())
  {
      //odom_test();
      //time_t now_time;
//      Get_Odom(0);
////      now_time = time(NULL);
////      cout << "Now_time is " << now_time << endl;
//      Get_Odom(1);
//      Get_Odom(2);
//      Get_Odom(3);

      //odom_pub.publish(odom_data);
      //cout << "-------------------------------------------------------------enter--------------------------------------------------" << endl;
      odom_publish();
      loop_rate.sleep();
      ros::spinOnce();
  }

  //test_thread1.join();
  //test_thread1.join();

//  read_thread1.join();
//  read_thread2.join();
//  read_thread3.join();
//  read_thread4.join();

   //pthread_exit(NULL);
  return 0;
}
