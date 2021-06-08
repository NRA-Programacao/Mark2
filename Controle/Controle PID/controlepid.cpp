#include <iostream>
#include <time.h>
#include <math.h>
#include "marcos.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <stdio.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

using namespace std;
class controle
{
public:
  float altura;
  float yaw, x, y;
  bool detectLine = false;
  float posicaoxQRcode, posicaoyQRcode;
  bool detectQRcode = false;
  string flight_mode;

  void callback_QRcode(const geometry_msgs::Pose2D &posicao)
  {
    //eixo x do drone eh o y do opencv
    posicaoxQRcode = posicao.y;
    posicaoyQRcode = posicao.x;
    detectQRcode = true;
  }
  
  
	void callback_twist(const geometry_msgs::Twist &drone_pose)
  {
  	float *l= drone_pose.linear.z; //velocidade em z (?)
  }
  	
  void callback_linefollower(const geometry_msgs::Pose2D &drone_pose)
  {
    yaw = drone_pose.theta;
    x = drone_pose.x;
    y = drone_pose.y;
    detectLine = true;
  }

  void callback_sonar(const sensor_msgs::Range &h)
  {
    altura = h.range;
  }
  void callback_flight_mode(const std_msgs::String &modo_voo)
  {
    flight_mode = modo_voo.data;
  }
};

void P2Controller();

int main(int argc, char **argv)
{
  
  // Init ROS node
  controle vars;

  ros::init(argc, argv, "controle");
  ros::NodeHandle nh;
  ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Publisher pub_centralizeQRcode = nh.advertise<std_msgs::Bool>("controle/centralizeQRcode", 10);
  ros::Subscriber sub_sonar = nh.subscribe("/sonar_height", 10, &controle::callback_sonar, &vars);
  ros::Subscriber sub_linefollower = nh.subscribe("/line_follower", 10, &controle::callback_linefollower, &vars);
  ros::Subscriber sub_qrcode = nh.subscribe("/qrcode/posicao", 10, &controle::callback_QRcode, &vars);
  ros::Subscriber sub_flight_mode = nh.subscribe("/master/fligth_mode", 10, &controle::callback_flight_mode, &vars);
  ros::Subscriber sub_vel = nh.subscribe("cmd_vel", 10, &controle::callback_twist, &vars);
  ros::Rate loop_rate(1000);

  geometry_msgs::Twist twist;
  std_msgs::Bool centralizeQRcode;

  float kpy = 0.09, kdy = 0.07, kiy = 0.03;
  float py, iy, dy, previousErroy;

  float kpxQR = 0.001, kdxQR = 0.000, kixQR = 0;
  float pxQR, ixQR, dxQR, previousErroxQR = 0;
  float kpyQR = 0.001, kdyQR = 0.000, kiyQR = 0.;
  float pyQR, iyQR, dyQR, previousErroyQR = 0;

  while (ros::ok())
  {
    //eixo x do drone e para frente, então tive que mudar os erros em x para y e vice-versa
//    py = -(vars.x);
//    iy = iy + py;
//    dy = dy - previousErroy;
//    previousErroy = py;

//    pxQR = (360 - vars.posicaoxQRcode);
//    ixQR = ixQR + pxQR;
//    dxQR = pxQR - previousErroxQR;
//    previousErroxQR = pxQR;
//
//    pyQR = (640 - vars.posicaoyQRcode);
//    iyQR = iyQR + pyQR;
//    dyQR = pyQR - previousErroyQR;
//    previousErroyQR = pyQR;
    
    //~ -- Horizontal control: 
    float *posError = -(vars.x);//getObjectPosition(targetObj, drone_pos);
    py = posError;
    float alphaE = p2tau[0];
    float betaE = p2tau[1];
    float alphaCorr = attKp*alphaE + attKd*(alphaE-pAlphaE);
    float betaCorr = attKp*betaE + attKd*(betaE-pBetaE);
    pAlphaE=alphaE;
    pBetaE=betaE;
    
    integradorX = integradorX + ((posError[0] + posErrorX_last)/2)*Ts;
    integradorY = integradorY + ((posError[1] + posErrorY_last)/2)*Ts;
    
    alphaCorr= alphaCorr + posKp*posError[1] + posKd*(posError[1]-posErrorY_last) + posKi*integradorY;
    betaCorr = betaCorr - (posKp*posError[0] + posKd*(posError[0]-posErrorX_last) + posKi*integradorX);
    posErrorY_last = posError[1];
    posErrorX_last = posError[0];
    
    //~ -- P2-Controller
    P2Controller();
	
	//~ -- Rotational control:
    // float *yaw_target = getObjectOrientation(targetObj, dummy_array);
    float yaw_error = -vars.yaw; // isso é erro ou é yaw?
								 //yaw_quad - yaw_target[2];
    float rotCorr = yaw_error*0.1 + 2*(yaw_error - yaw_error_last);
    yaw_error_last = yaw_error;

    //controle de altura
    //twist.linear.z = 2 * (0.9 - vars.altura);
    
    //~ -- Vertical control:
    //float dummy_array[3] = {0.0, 0.0, 0.0};
    //float vel_z, vel_roll, vel_pitch, vel_yaw;
    //targetPos = getObjectPosition(targetObj, dummy_array);
    //~ --pos=sim.getObjectPosition(d,-1)
    //float *l; //= getVelocity(heli);//problema da daq
    float e = 1 - vars.altura; //= (targetPos[2] - gps_xyz[2]); //vars.altura é a altura do drone
    float cumul = cumul + e;
    float pv = pParam*e;
    float thrust = 5.335 + pv + iParam*cumul + dParam*(e-lastE) + l[2]*vParam;
    lastE = e;
    twist.linear.z = thrust;

//    //centalizar no QR code
//    if (vars.flight_mode == "qrcode")
//    {
//      vars.detectQRcode = false;
//      twist.linear.x = kpxQR * (pxQR) + kdxQR * dxQR + kixQR * ixQR;
//      twist.linear.y = (kpyQR * (pyQR) + kdyQR * dyQR + kiyQR * iyQR);
//    }
//    if (pxQR < 5 and pyQR < 5)
//    {
//      centralizeQRcode.data = true;
//      pub_centralizeQRcode.publish(centralizeQRcode);
//    }
//    else if (std::abs(pxQR + pyQR) > 900)
//    {
//      centralizeQRcode.data = false;
//      pub_centralizeQRcode.publish(centralizeQRcode);
//    }
//    else
//    {
//      centralizeQRcode.data = false;
//      pub_centralizeQRcode.publish(centralizeQRcode);
//    }

    //seguidor de linha
    if (vars.flight_mode == "line_follower")
    {
//      if (abs(py) < 0.05)
//      {
//        twist.linear.x = 0.3;
//      }
//      else
//      {
//        twist.linear.x = 0.05;
//      }
      twist,linear.x = alphaCorr;
      twist.linear.y = betaCorr;//kpy * py;
      //+kdy *dy + kiy *iy;
      cout << twist.linear.y << endl;
      twist.angular.z = rotCorr; //(0.7 * (-vars.yaw));
      vars.detectLine = false;
    }

    pub_vel.publish(twist);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void P2Controller()
{
    //~ -- Full Quaternion Based Attitude Control for a Quadrotor, Emil Fresk and George Nikolakopoulos,
    //~ -- European Control Conference, 2013
    float q0 = qs[0];
    float q1 = qs[1];
    float q2 = qs[2];
    float q3 = qs[3];
    float qErr[3] = {0,0,0};
    
    q1 = -q1;
    q2 = -q2;
    q3 = -q3;
    //~ -- Product: q_ref (x) *q_m, eq. 19:
    float qErr0 = p0*q0 - p1*q1 - p2*q2 - p3*q3;
    qErr[0] = p0*q1 + p1*q0 + p2*q3 - p3*q2;
    qErr[1] = p0*q2 - p1*q3 + p2*q0 + p3*q1;
    qErr[2] = p0*q3 + p1*q2 - p2*q1 + p3*q0;
    //~ -- eq. 20:
    if (qErr0 < 0)
    {
        qErr[0] = -qErr[0];
        qErr[1] = -qErr[1];
        qErr[2] = -qErr[2];
    }
    //~ -- eq. 21:
    //erro de velocidade angular
    for(int i=0; i<3; i++)
    {
        p2tau[i] = -KwP2Controller*angVel[i] - KqP2Controller*qErr[i];
    }

}
