#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <stdio.h>
#include <std_msgs/Float32.h>

using namespace std;

// Init variables
float altura = 0;
float yaw = 0;
void callback_yaw(const std_msgs::Float32 &delta)
{
  yaw = delta.data;
}
void callback_sonar(const sensor_msgs::Range &h)
{
  altura = h.range;
  cout << altura << endl;
}
int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "controle");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber sub_sonar = nh.subscribe("/sonar_height", 10, callback_sonar);
  ros::Subscriber sub_yaw = nh.subscribe("/yaw_rads", 10, callback_yaw);
  ros::Rate loop_rate(100);
  geometry_msgs::Twist twist;

  while (ros::ok())
  {
    twist.linear.z = 2 * (0.6 - altura);
    //twist.linear.x=0.15;
    twist.angular.z = (0.4 * (-1 * yaw));
    if (abs(yaw) < 0.01)
    {
      twist.linear.x = 0.15;
    }
    pub.publish(twist);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}