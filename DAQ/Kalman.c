#include <stdio.h>
#include <math.h>

double t; //intervalo de tempo do filtro de Kalman
int alfa = 1e-3, ki = 0, beta = 2;

double X_m0 = {0 0 0 0 0 0 0 0 0 0 0 0}; //estados medidos k-1
double X_m1 = {0 0 0 0 0 0 0 0 0 0 0 0}; //estados medidos k
double a = [[ax], [ay], [az]]; //sensores
double g = [gx gy gz]; 
double m = [mx my mz]; 
double R = [[cos(mx)cos(mz) sin(mx)sin(my)cos(mz)-cos(mx)sin(mz) cos(mx)sin(my)cos(mz)+sin(mx)sin(mz)] [sin(my)sin(mz) sin(mx)sin(my)sin(mz)+cos(mx)cos(mz) cos(mx)sin(my)sin(mz)-sin(mx)cos(mz)] [-sin(my) sin(mx)cos(my) cos(mx)cos(my)]]; //Earth-fixed to body frame
double gr = [0 0 -9.81]; //vetor gravidade 
double M = [[t*t/2 0 0], [0 t*t/2 0], [0 0 t*t/2], [0 0 0], [0 0 0], [0 0 0], [0 0 0], [0 0 0], [0 0 0], [0 0 0], [0 0 0], [0 0 0]]; //matriz acelerômetro
double N = [[t 0 0] [0 t 0] [0 0 t]]; //matriz giroscópio
            
double X = [x y z r p ya vx vy vz vr vp vya] = [0 0 0 0 0 0 0 0 0 0 0 0]; //vetor de estados
double A = [[1 0 0 0 0 0 t 0 0 0 0 0], [0 1 0 0 0 0 0 t 0 0 0 0], [0 0 1 0 0 0 0 0 t 0 0 0], [0 0 0 1 0 0 0 0 0 t 0 0 ], [0 0 0 0 1 0 0 0 0 0 t 0], [0 0 0 0 0 1 0 0 0 0 0 t], [0 0 0 0 0 0 1 0 0 0 0 0], [0 0 0 0 0 0 0 1 0 0 0 0], [0 0 0 0 0 0 0 0 1 0 0 0], [0 0 0 0 0 0 0 0 0 1 0 0], [0 0 0 0 0 0 0 0 0 0 1 0], [0 0 0 0 0 0 0 0 0 0 0 1]]; //matriz de estados
double u = [accx accy accz accr accp accya]; //vetor de controle
double B = [[t*t/2 0 0 0 0 0], [0 t*t/2 0 0 0 0], [0 0 t*t/2 0 0 0], [0 0 0 t*t/2 0 0], [0 0 0 0 t*t/2 0], [0 0 0 0 0 t*t/2], [t 0 0 0 0 0], [0 t 0 0 0 0], [0 0 t 0 0 0], [0 0 0 t 0 0], [0 0 0 0 t 0], [0 0 0 0 0 t]]; //matriz de controle

int main ()
{
 X = X*A + B*u;
 a = R*(a-gr);
 M_m1 = X_m0 + M*a
  
}
/*
roll_quad  = math.atan(2*(q2*q3 + q0*q1), q0^2 - q1^2 - q2^2 - q3^2) -- Roll
pitch_quad = math.asin(-2*(q1*q3 - q0*q2))                           -- Pitch
yaw_quad   = math.atan(2*(q1*q2 + q0*q3), q0^2 + q1^2 - q2^2 - q3^2) -- Yaw
*/
