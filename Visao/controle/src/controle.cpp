#include<ros/ros.h>
#include <geometry_msgs/Twist.h>
#include<sensor_msgs/Range.h>
#include <stdio.h>

using namespace std;

// Init variables
float altura=0;

void callback(const sensor_msgs::Range& h){
    altura = h.range;
    cout<<altura<<endl;
    
}
int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "controle");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber sub= nh.subscribe("/sonar_height",10,callback);

  ros::Rate loop_rate(100);
  geometry_msgs::Twist twist;

    while (ros::ok())
    {
        twist.linear.z=2*(1-altura);
        twist.linear.x=0.3;
        pub.publish(twist);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
  
    
  return 0;
}