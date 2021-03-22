#include <stdio.h>
#include <math.h>

double [ax ay az gx gy gz]; //medidas
double t //intervalo de tempo
double [x y z r p ya vx vy vz vr vp vya]; //vetor de estados
double A = [[1 0 0 0 0 0 t 0 0 0 0 0],[0 1 0 0 0 0 0 t 0 0 0 0], [0 0 1 0 0 0 0 0 t 0 0 0], [0 0 0 1 0 0 0 0 0 t 0 0 ], [0 0 0 0 1 0 0 0 0 0 t 0], [0 0 0 0 0 1 0 0 0 0 0 t], [0 0 0 0 0 0 1 0 0 0 0 0], [0 0 0 0 0 0 0 1 0 0 0 0], [0 0 0 0 0 0 0 0 1 0 0 0], [0 0 0 0 0 0 0 0 0 1 0 0], [0 0 0 0 0 0 0 0 0 0 1 0], [0 0 0 0 0 0 0 0 0 0 0 1]];
double B = [[t*t/2 0 0 0 0 0], [0 t*t/2 0 0 0 0], [0 0 t*t/2 0 0 0], [0 0 0 t*t/2 0 0], [0 0 0 0 t*t/2 0], [0 0 0 0 0 t*t/2], [t 0 0 0 0 0], [0 t 0 0 0 0], [0 0 t 0 0 0], [0 0 0 t 0 0], [0 0 0 0 t 0], [0 0 0 0 0 t]];






roll_quad  = math.atan(2*(q2*q3 + q0*q1), q0^2 - q1^2 - q2^2 - q3^2) -- Roll
pitch_quad = math.asin(-2*(q1*q3 - q0*q2))                           -- Pitch
yaw_quad   = math.atan(2*(q1*q2 + q0*q3), q0^2 + q1^2 - q2^2 - q3^2) -- Yaw
