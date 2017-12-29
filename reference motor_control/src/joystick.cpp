#include <stack>
#include <vector>
#include <math.h>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include <std_msgs/Float32.h>


using namespace std;

ofstream outfile;
string fileName="/home/csc105/catkin_ws/src/tcp2ros/NodeMap/Node.txt";



class TeleopJoy{
public:
  TeleopJoy();
  void pub_cmd();
private:
  void callBack(const sensor_msgs::Joy::ConstPtr& joy);
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Publisher stop_pub;
  ros::Subscriber sub;
  int i_velLinear, i_velAngular;
  geometry_msgs::Twist vel;
  std_msgs::Float32 stop;
};

TeleopJoy::TeleopJoy()
{    
    n.param("axis_linear",i_velLinear,1);
    n.param("axis_angular",i_velAngular,0);
    
    pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    
    stop_pub = n.advertise<std_msgs::Float32>("stop",1);
    sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::callBack, this);
}

void TeleopJoy::pub_cmd()
{
  pub.publish(vel);
  stop_pub.publish(stop);
}


void TeleopJoy::callBack(const sensor_msgs::Joy::ConstPtr& joy)
{
    if (joy->axes[2] < -0.1)
    {

        vel.angular.z = 0;
        vel.linear.x = 0;
        pub.publish(vel);      

        stop.data = 1;
        stop_pub.publish(stop);
    }
    else if(joy->axes[2] >= -0.1)
    {
      if (joy->axes[5] < 0.5)
      {
          vel.angular.z = joy->axes[3];
          vel.linear.x = joy->axes[1];
          pub.publish(vel);
      }
        stop.data = 0;
        stop_pub.publish(stop);
    }

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleopJoy");
	ros::NodeHandle ph;

	TeleopJoy teleop_turtle;
  ros::Rate loop_rate(20);
	

  while (ros::ok())
  {
    teleop_turtle.pub_cmd();


    loop_rate.sleep();
    ros::spinOnce();

  }
}
