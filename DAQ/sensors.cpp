#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <stdio.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

using namespace std;

class sensores
{
public:
  double z; // altura do drone baseado no sensor ultrassonico
  double mx, my, mz; //resposta do magnetometro
  double yaw //yaw vindo da visão (line follower)

void callback_sonar(const sensor_msgs::Range &h)
  {
    z = h.range;
  }
void callback_magnetic(const geometry_msgs::Vector3 &angulo)
  {
    mx = angulo.y;
    my = angulo.x;
    mz = angulo.z;
  }
 void callback_linefollower(const geometry_msgs::Pose2D &drone_pose)
  {
    yaw = drone_pose.theta;
  }
};

int main(int argc, char **argv)
{
  
  // Init ROS node
  sensores vars;

  ros::init(argc, argv, "sensores");
//  ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
//  ros::Publisher pub_centralizeQRcode = nh.advertise<std_msgs::Bool>("controle/centralizeQRcode", 10);
  ros::Subscriber sub_sonar = nh.subscribe("/sonar_height", 10, &sensores::callback_sonar, &vars);
  ros::Subscriber sub_mag = nh.subscribe("/magnetic", 10, &sensores::callback_magnetic, &vars);
  ros::Subscriber sub_linefollower = nh.subscribe("/line_follower", 10, &controle::callback_linefollower, &vars);
//  ros::Subscriber sub_qrcode = nh.subscribe("/qrcode/posicao", 10, &controle::callback_QRcode, &vars);
//  ros::Subscriber sub_vel = nh.subscribe("cmd_vel", 10, &controle::callback_twist, &vars);
  ros::Rate loop_rate(1000);

  geometry_msgs::Twist twist;
  std_msgs::Bool centralizeQRcode;
  
}
