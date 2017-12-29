#include <motor_control/readDataAll.h>
#include <sstream>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <ctime>
#include <pthread.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/thread/condition_variable.hpp>
#include <limits>

#include <stdlib.h>
#include <std_msgs/Float32.h>
#include <motor_control/readCurrent.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <cmath>

#define Thread_num 4

//#include <iomanip>
using namespace std;

serial::Serial ser; //声明串口对象


ros::Publisher* motor_pub_;


tf::TransformBroadcaster* odom_broadcaster_;

double scale = 0.6/548;//1:36减速

double a  = 0.322;//1:36减速
double b  = 0.215;//1:36减速

double vel_[4];
int update_freq_ = 20;


int is_cmd_valid = 1;  //指令是否有效标志

double pose_x = 0.0;
double pose_y = 0.0;
double pose_th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

double x = 0.0;
double y = 0.0;
double th = 0.0;
double last_data[4] = {0.0};
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

/* 
 * CRC16校验 
 */
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
//  cout << "0x" << hex << int(ucCRCLo) << " ";
//  cout << "0x" << hex << int(ucCRCHi) << endl;
//  cout << "0x" << hex << ( int )( ucCRCHi << 8 | ucCRCLo );
    return ( int )( ucCRCHi << 8 | ucCRCLo );
}

/*
 *  十进制转十六进制
 */
int DectoHex(short dec, unsigned char *hex, int length)
{
    int i;

    if (dec < 0)
    {
        dec = -dec;
        for (i = length - 1; i >= 0; i--)
        {
            hex[i] = (dec % 256) & 0xFF;
            dec /= 256;
        }

        hex[0] = (((~((hex[0] << 8)|hex[1])) +1)>>8)&0xff;
        hex[1] = ((~((hex[0] << 8)|hex[1])) +1)&0xff;
    }
    else
    {
        for (i = length-1; i >= 0; i--)
        {
            hex[i] = (dec % 256) & 0xFF;
            dec /= 256;
        }
    }

    return 0;
}

/*
 *  读取电机换向脉冲数
 */
void Get_Odom(int ser_id)
{
    //int ser_id = *((int*)thread);
    if (ser_id >= 0 && ser_id <4)
    {
        //cout << "ser_id:         " << ser_id << endl;
        uint8_t send[8];
        int check;
        int check_temp;
        send[0] = (uint8_t)(ser_id + 1);
        send[1] = 0x03;
        send[2] = 0x00;
        send[3] = 0x24;
        send[4] = 0x00;
        send[5] = 0x02;  //连续读取2个寄存器，一共4个字节

        check = usMBCRC16(send,6);
        send[6] = check & 0xff;
        send[7] = (check>>8) & 0xff;

        ser.write(send,8);

        vector<uint8_t> receive;
        ser.read(receive,9);  //每次发送完读命令，需要立刻读串口数据，否则会丢失

        uint8_t temp[9];
	}
	if (receive.size() == 9)
	{
            for(int i=0; i<7; i++)
            {
                temp[i] = receive[i];
            }
            check_temp = usMBCRC16(temp,7);  //7个字节计算CRC校验


            if ((receive[7]+(receive[8]<<8)) == check_temp)  //若CRC一致
            {
                switch(receive[0])
                {
                case 1:
                    odom_data.odom1 =  (((receive[3]) << 24)|((receive[4]) << 16)|((receive[5]) << 8)|receive[6])*scale;
                    odom_data_.odom1 = (((receive[3]) << 24)|((receive[4]) << 16)|((receive[5]) << 8)|receive[6]);
                    //cout << "get_odom1-------------:" <<odom_data_.odom1<< endl;
                    break;
                case 2:
                    odom_data.odom2 =  (((receive[3]) << 24)|((receive[4]) << 16)|((receive[5]) << 8)|receive[6])*scale;
                    //odom_data_.odom2 = (((receive[3]) << 24)|((receive[4]) << 16)|((receive[5]) << 8)|receive[6]);
                    //cout << "get_odom2-------------" << endl;
                    break;
                case 3:
                    odom_data.odom3 =  (((receive[3]) << 24)|((receive[4]) << 16)|((receive[5]) << 8)|receive[6])*scale;
                    //odom_data_.odom3 = (((receive[3]) << 24)|((receive[4]) << 16)|((receive[5]) << 8)|receive[6]);
                    //cout << "get_odom3-------------" << endl;
                    break;
                case 4:
                    odom_data.odom4 =  (((receive[3]) << 24)|((receive[4]) << 16)|((receive[5]) << 8)|receive[6])*scale;
                    //odom_data_.odom4 = (((receive[3]) << 24)|((receive[4]) << 16)|((receive[5]) << 8)|receive[6]);
                    //cout << "get_odom4-------------" << endl;
                    break;
                default:
                    break;
                }
            }
         }
        else
        {
            ROS_WARN("The number received in Get_Odom by USB%d less than 9,receive %d data",ser_id,receive.size());
        }
    }
}

/*
 *  读取实时电流
 */
void Get_Current(int ser_id)
{
    if (ser_id >= 0 && ser_id <4)
    {
        uint8_t send[8];
        int check;
        int check_temp;

        send[0] = (uint8_t)(ser_id + 1);
        send[1] = 0x03;
        send[2] = 0x00;
        send[3] = 0x21;
        send[4] = 0x00;  //实时电流寄存器
        send[5] = 0x01;
        check = usMBCRC16(send,6);
        send[6] = check & 0xff;
        send[7] = (check>>8) & 0xff;
        //cout << "0x" << hex << int(send[6]) << " ";
        //cout << "0x" << hex << int(send[7]) << endl;

        ser.write(send,8);

        vector<uint8_t> receive;
        ser.read(receive,7);  //每次发送完读命令，需要立刻读串口数据，否则会丢失

        if (receive.size() == 5)  //当从站接收错误时,从站回送5个字节
        {
            ROS_INFO("Received Error:%02X",int(receive[2]));  //输出错误码
        }
        else if(receive.size() == 7)  //正常情况下读回７个字节
        {
            uint8_t temp[7];

            for(int i=0; i<5; i++)
            {
                temp[i] = receive[i];
            }

            check_temp = usMBCRC16(temp,5);  //5个字节计算CRC

            if( (receive[5]+(receive[6]<<8)) == check_temp )
            {
                switch((int)receive[0])
                {
                case 1:
                    current_data.current1 = (((receive[3])<<8)|receive[4])*0.01;
                    break;
                case 2:
                    current_data.current2 = (((receive[3])<<8)|receive[4])*0.01;
                    break;
                case 3:
                    current_data.current3 = (((receive[3])<<8)|receive[4])*0.01;
                    break;
                case 4:
                    current_data.current4 = (((receive[3])<<8)|receive[4])*0.01;
                    break;
                default:
                    break;
                }
            }
            else
            {
                ROS_INFO("接受电流数据错误！");
            }
        }
    }
}

/*
 *  读取电机实时换向频率（转速）
 */
void current_frequence(int ser_id)
{
    if (ser_id >= 0 && ser_id < 4)
    {
        uint8_t send[8];
        int check;
        int check_temp;

        send[0] = (uint8_t)(ser_id + 1);
        send[1] = 0x03;
        send[2] = 0x00;
        send[3] = 0x22;
        send[4] = 0x00;
        send[5] = 0x01;
        check = usMBCRC16(send,6);	//CRC16
        send[6] = check&0xff;
        send[7] = (check>>8)&0xff;
        //cout << "0x" << hex << int(send[6]) << " ";
        //cout << "0x" << hex << int(send[7]) << endl;

        ser.write(send,8);

        vector<uint8_t> receive;
        ser.read(receive,7); //每次发送完读命令，需要立刻读串口数据，否则会丢失

        if (receive.size() == 5)  //当从站接收错误时,从站回送5个字节
        {
            ROS_INFO("Received Error:%02X",int(receive[2]));  //输出错误码
        }
        else if(receive.size() == 7)
        {
            uint8_t temp[7];

            for(int i=0; i<5; i++)
            {
                temp[i] = receive[i];
            }
            check_temp = usMBCRC16(temp,5);

            if (check_temp == receive[5]<<8|receive[6]) 	//校验成功
            {
                switch(receive[0])
                {
                case 1:
                    odom_data_.odom1 =  (((receive[3]) << 8)|receive[4])*0.1;
                    break;
                case 2:
                    odom_data_.odom2 = (((receive[3]) << 8)|receive[4])*0.1;
                    break;
                case 3:
                    odom_data_.odom3 =  (((receive[3]) << 8)|receive[4])*0.1;
                    break;
                case 4:
                    odom_data_.odom4 = (((receive[3]) << 8)|receive[4])*0.1;
                    break;
                default:
                    break;
                }
            }
        }
    }
}

/*
 *  设定占空比
 */
void pwm_output(int ser_id,float p)
{
    if (ser_id >=0 && ser_id <4)
    {
        if (p>= -1 && p<= 1)
        {
            int p_t;
            p_t = p*1000;
            uint8_t send_buff[8];
            int check;
	    
            //wheel 1
            uint8_t send[8];
            send[0] = (uint8_t)(ser_id + 1);
            send[1] = 0x06;
            send[2] = 0x00;
            send[3] = 0x42;
            
            if(ser_id == 0 || ser_id == 1)
                 p_t = -p_t;
	    
            send[4] = (p_t>>8)&0xff;
            send[5] = (p_t&0xff);
            check = usMBCRC16(send,6);
            send[6] = check&0xff;
            send[7] = (check>>8)&0xff;

            ser.write(send,8);

            vector<uint8_t> receive;
            ser.read(receive,8); //每次发送完读命令，需要立刻读串口数据，否则会丢失

            uint8_t temp[8];
            int check_temp;
            int i;
            bool state;
	    
            if (receive.size() == 5)
                cout << "0x" << hex << int(receive[2]) << endl;

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
                    //exit(1);
                }
            }
        }
    }
}

/*
 * 指令设置速度闭环控制目标速度（换向频率）
 */
bool set_speed(int ser_id,short p)
{
    bool state = false;

    if (ser_id >= 0 && ser_id < 4)
    {
        int check;
        uint8_t send[8];

        if(p < 100 && p > -100)
        {
            //写单个寄存器
            send[0] = (uint8_t)(ser_id + 1);    //设定驱动器地址
            send[1] = 0x06;
            send[2] = 0x00;
            send[3] = 0x40;

            send[4] = 0x00;
            send[5] = 0x02;  //自由停止模式

            check = usMBCRC16(send,6);  //计算CRC16校验和
            send[6] = check & 0xff;
            send[7] = (check>>8) & 0xff;

            ser.write(send,8);

            vector<uint8_t> receive;
            //ser.read(receive,8); //每次发送完读命令，需要立刻读串口数据，否则会丢失
            if(ser.waitReadable())
            {
              ser.read(receive,8);
              //cout << "Read data successfully" << endl;
              //ser.waitByteTimes(9);
              error_cnt=0;
            }
            else
            {
              error_cnt++;
              ROS_WARN("USB%d set_speed Receive fail",ser_id);
              if(error_cnt > 5)
              {
                //is_cmd_valid = 0;
                error_cnt = 0;
                ROS_ERROR("Stop");
              }
              return 0;
            }


            uint8_t temp[8];
            int check_temp;
            bool state = false;

            if (receive.size() == 5)  //当从站接收错误时,从站回送5个字节
            {
                ROS_INFO("Received Error:%02X",int(receive[2]));  //输出错误码
            }
            else if(receive.size() == 8)  //正常返回８个字节
            {
                for(int i=0; i<6; i++)
                {
                    temp[i] = receive[i];
                }
                check_temp = usMBCRC16(temp,6);

                if ( (receive[6]+(receive[7]<<8)) == check_temp )  //若校验和一致
                {
                    state = true;
                    //exit(1);
                }
            }
            else
            {
               ROS_ERROR("设置速度:USB%d receive data error,receive %d data",ser_id,receive.size());
            }
        }
        else if (p >= -65535 && p<= 65535)  //最大换向评论800Hz?
        {
            unsigned char hex_bff[2] = " ";
            //p_t = p*1000;

            //写单个寄存器，设定速度闭环控制目标速度(换向频率)
            send[0] = (uint8_t)(ser_id + 1);    //设定驱动器地址
            send[1] = 0x06;
            send[2] = 0x00;
            send[3] = 0x43;
	    
            if(ser_id == 0 || ser_id == 1)
                 p = -p;

            DectoHex(p,hex_bff,2);  //10参数进制转换为16进制指令
            send[4] = hex_bff[0];
            send[5] = hex_bff[1];

            check = usMBCRC16(send,6);  //计算前6个字节的校验和
            send[6] = check & 0xff;
            send[7] = (check>>8) & 0xff;

            ser.write(send,8);

            vector<uint8_t> receive;
            //ser.read(receive,8);   //每次发送完读命令，需要立刻读串口数据，否则会丢失

            if(ser.waitReadable())
            {
              ser.read(receive,8);
              //cout << "Read data successfully" << endl;
              //ser.waitByteTimes(9);
              error_cnt = 0;
            }
            else
            {
                error_cnt++;
                ROS_WARN("USB%d set_speed Receive fail",ser_id);
                if(error_cnt > 5)
                {
                  //is_cmd_valid = 0;
                  error_cnt = 0;
                  ROS_ERROR("Stop");
                }
              return 0;
            }
            uint8_t temp[8];
            int check_temp;
            //bool state;

            if(receive.size() == 5)  //从站接收错误时，返回5个字节
            {
                ROS_INFO("Received Error:%02X",int(receive[2]));  //异常码
            }
            else if(receive.size() == 8)  //正常情况下返回8个字节
            {
                for(int i=0; i<6; i++)
                {
                    temp[i] = receive[i];
                }
                check_temp = usMBCRC16(temp,6);  //计算CRC16校验和

                if( (receive[6]+(receive[7]<<8)) == check_temp )  //若校验和一致
                {
                    state = true;
                    //exit(1);
                }
            }
            else
            {
               ROS_ERROR("设置速度:USB%d receive data error,receive %d data",ser_id,receive.size());
            }

        }
    }

    return state;
}

/*
 *  紧急刹车
 */
bool hard_braking()
{
    bool state = false;
    int check;
    uint8_t send[8];

    for(uint8_t ser_id = 1; ser_id <= 4; ser_id++)
    {
        //写单个寄存器
        send[0] = (uint8_t)(ser_id);    //设定驱动器地址
        send[1] = 0x06;
        send[2] = 0x00;
        send[3] = 0x40;
        send[4] = 0x00;
        send[5] = 0x01;  //紧急制动模式

        check = usMBCRC16(send,6);  //计算CRC16校验和
        send[6] = check & 0xff;
        send[7] = (check>>8) & 0xff;

        ser.write(send,8);

        vector<uint8_t> receive;
        ser.read(receive,8);  //每次发送完读命令，需要立刻读串口数据，否则会丢失

        uint8_t temp[8];
        int check_temp;
        bool state = false;

        if (receive.size() == 5)  //当从站接收错误时,从站回送5个字节
        {
            ROS_INFO("Received Error:%02X",int(receive[2]));  //输出错误码
        }
        else if(receive.size() == 8)  //正常返回８个字节
        {
            for(int i=0; i<6; i++)
            {
                temp[i] = receive[i];
            }
            check_temp = usMBCRC16(temp,6);

            if ( (receive[6]+(receive[7]<<8)) == check_temp )  //若校验和一致
            {
                state = true;
                //exit(1);
            }
        }
        else
        {
           ROS_ERROR("紧急刹车:USB%d receive data error,receive %d data",ser_id,receive.size());
        }
    }

    return state;
}

void odom_test()
{
    uint8_t send[8];
    int check;
    int check_temp;
    
    //电机实时位置
    send[0] = 0X01;
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

    ser.write(send,8);

    vector<uint8_t> receive;
    ser.read(receive,9); //每次发送完读命令，需要立刻读串口数据，否则会丢失
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

/*
 *  里程计数据发布
 */
void odom_publish()
{
    static int init = 0;

    if(++init < 6)
    {
        last_data[0]=odom_data.odom1;
        last_data[1]=odom_data.odom2;
        last_data[2]=odom_data.odom3;
        last_data[3]=odom_data.odom4;
//      cout << "--------------------------------" << endl;
//      cout << " odom_data.odom1" << odom_data.odom1;
//      cout << " odom_data.odom2" << odom_data.odom2;
//      cout << " odom_data.odom3" << odom_data.odom3;
//      cout << " odom_data.odom4" << odom_data.odom4 << endl;

        //更新时间
        updated_ = ros::Time::now().toSec();
        //init=true;
        return;
    }

    double period = ros::Time::now().toSec() - updated_;
    updated_ = ros::Time::now().toSec();
    //cout << "period"<< period << endl;

    //注意电源关闭之后，驱动器并没有马上掉电
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

//    cout << " odom_data.odom1" << odom_data.odom1;
//    cout << " odom_data.odom2" << odom_data.odom2;
//    cout << " odom_data.odom3" << odom_data.odom3;
//    cout << " odom_data.odom4" << odom_data.odom4 << endl;

    vel_[0] = vel1;
    vel_[1] = vel3;
    vel_[2] = vel4;
    vel_[3] = vel2;

    vx=(vel_[0]+vel_[1]+vel_[2]+vel_[3])/4.0;
    vy=0.0;
    vth=(((vel_[0]-vel_[2])/2.0/(a+b)+(vel_[3]-vel_[1])/2.0/(a+b))/2.0)*(2*M_PI/5.72);  //5.18498
    //*(2*M_PI/5.18498)
//    cout << "vel_[0]"<< vel_[0] << endl;
//        cout << "vel_[1]"<< vel_[1] << endl;
//            cout << "vel_[2]"<< vel_[2] << endl;
//                cout << "vel_[3]"<< vel_[3] << endl;
//                    cout << "a"<< a << endl;
//                    cout << "b"<< b << endl;
//    cout << "vel_[0]" << vel_[0] << endl;
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

    //发布tf变换
    //odom_broadcaster_->sendTransform(odom_trans);

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
    odom_pub_->publish(odom);  //里程计数据发布
}

/*
 *  电流数据发布
 */
void current_publish()
{
    current_pub_->publish(current_data);
}

/*
 * 手柄消息的回调函数，转化为车轮速度指令
 */
void receiveSpeed(geometry_msgs::Twist twist)
{
    float x,angle;

    if(bMotorTypeIs50_1)
    {
      x = twist.linear.x * 2000*6.62 ;  //线速度;
      angle = twist.angular.z * 2000*3.268 ;  //角速度;
    }
    else
    {
      x = twist.linear.x * 2000*6.62 / 50 * 36;  //线速度;
      angle = twist.angular.z * 2000*3.268 / 50 * 36;  //角速度;
    }
    //x = twist.linear.x * 0.667;//线速度对应的pwm占空比
    //angle = twist.angular.z * 0.328;//角速度对应的pwm占空比

    p1 = (short)(x + angle);
    p2 = p1;
    p3 = (short)(x - angle);
    p4 = p3;
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

/*
 * 读取错误状态寄存器
 */
void error_state(int ser_id)
{
    if (ser_id >= 0 && ser_id <4)
    {
        uint8_t send[8];
        int check;
        int check_temp;
        send[0] = (uint8_t)(ser_id + 1);
        send[1] = 0x03;
        send[2] = 0x00;
        send[3] = 0x33;
        send[4] = 0x00;
        send[5] = 0x01;
        check = usMBCRC16(send,6);
        send[6] = check&0xff;
        send[7] = (check>>8)&0xff;
        //cout << "0x" << hex << int(send[6]) << " ";
        //cout << "0x" << hex << int(send[7]) << endl;

        ser.write(send,8);

        vector<uint8_t> receive;
        ser.read(receive,7); //每次发送完读命令，需要立刻读串口数据，否则会丢失

        uint8_t temp[7];
        int i;
        if (receive.size() == 7)
        {
            for(i = 0;i<5;i++)
            {
                temp[i] = receive[i];
            }
            check_temp = usMBCRC16(temp,5);

            if (receive[0] == (uint8_t)(ser_id + 1) && receive[1] == 0x03)
            {
                if (check_temp == receive[5]<<8|receive[6])
                {
                  //if(int(receive[4]) == 4)
                    cout << "ser_id" << ser_id<< " error_state: " << int(receive[4]) << endl;
                }
            }
        }
    }
}

/* 禁用报警 */
void ring_ban()
{
    for(int ser_id = 0;ser_id < 4;ser_id++)
    {
        //p_t = p*1000;
        uint8_t send_buff[8];
        int check;
        //wheel 1
        uint8_t send[8];
        send[0] = (uint8_t)(ser_id + 1);
        send[1] = 0x06;
        send[2] = 0x00;
        send[3] = 0x99;
        //cout << " pre_p : " << p;
        send[4] = 0x00;
        send[5] = 0x01;
        check = usMBCRC16(send,6);
        send[6] = check&0xff;
        send[7] = (check>>8)&0xff;

        ser.write(send,8);

        vector<uint8_t> receive;
        ser.read(receive,8);  //每次发送完读命令，需要立刻读串口数据，否则会丢失

        uint8_t temp[8];
        int check_temp;
        int i;
        bool state;
        if (receive.size() == 5)
	{
          cout << "0x" << hex << int(receive[2]) << endl;
	  return;
	}

        if (receive.size() == 8)
        {
            for(i = 0;i<6;i++)
            {
              temp[i] = receive[i];
            }
            check_temp = usMBCRC16(temp,6);

            if (receive[0] == 0x01 && receive[1] == 0x06)
            {
                    state = true;
                    //exit(1);
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;
    ros::NodeHandle nh_param("~");

    nh_param.param<bool>("MotorTypeIs50_1", bMotorTypeIs50_1, true);
    if(bMotorTypeIs50_1)
    {
      double a = 0.25;//1:50减速箱
      double b = 0.255;//1:50减速箱
      scale = 0.000766*1.2052*0.6904;  //1:50减速箱
    }
     else
    {
        double a  = 0.322;//1:36减速
        double b  = 0.215;//1:36减速
        scale = 0.6/548;//1:36减速
    }

    // ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom",1,true);
    // ros::Publisher current_pub = n.advertise<motor_control::readCurrent>("current",1,true);
    // ros::Publisher pulse_pub = n.advertise<motor_control::readDataAll>("/pulse_count",1,true);
    //ros::Publisher motor_pub = n.advertise<std_msgs::Bool>("motor",1,true);
    ros::Subscriber speed_sub = n.subscribe<geometry_msgs::Twist>("/smooth_cmd_vel",1, &receiveSpeed);
    //ros::Subscriber speed_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel",1, &receiveSpeed);
    ros::Subscriber is_cmd_valid_sub = n.subscribe<std_msgs::Float32>("stop",1, &protect);
    //ros::Subscriber joy_sub = n.subscribe<std_msgs::Float32>("cmd",1,&change_pwm);

    // tf::TransformBroadcaster odom_broadcaster;

    // odom_pub_ = &odom_pub;
    // pulse_pub_ = &pulse_pub;
    // current_pub_ = &current_pub;
    //motor_pub_ = &motor_pub;
    // odom_broadcaster_ = &odom_broadcaster;
  
    ros::Rate loop_rate(100);
  
    bool serial_error = false;
    //bool serial_open = false;
  
    try
    {
        //设置串口属性，并打开串口
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        ser.setParity(serial::parity_even);
        ser.setStopbits(serial::stopbits_one);
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

    int count = 0;

    while (ros::ok())
    {
        if(is_cmd_valid == 0)  //指令无效时，立即制动
        {
            //cout<< "$$$$$$$$$$$$$$$$$$$" << endl;
            hard_braking();
        }
        else
        {
            set_speed(0, p1);
            set_speed(1, p2);
            set_speed(2, p3);
            set_speed(3, p4);
        }

       //error_state(3);
        for(int i=0; i<4; i++)
        {
            Get_Odom(i);    //读取里程计

            //error_state(i);
            //current_frequence(i);   //读取转速
        }

        odom_publish();  //里程计发布

        count++;
        if(count == 15)
        {
            count = 0;

            for(int i=0;i<4;i++)
            {
                Get_Current(i);
            }
            current_publish();  //电流发布
        }

        //pulse_pub.publish(odom_data_);

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
