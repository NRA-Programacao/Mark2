#ifndef MARCOS_BIB
#define MARCOS_BIB

float targetObj[3]; //x, y, z
float particlesTargetVelocities[4] = {0, 0, 0, 0}; //velocidades dos motores

//Parametros PID
int pParam=2;
int iParam=0;
int dParam=0;
int vParam=-2;

float attKp = 0.25;
float attKd = 2.10;
float posKp = 0.005;
float posKd = 1.000;
float posKi = 0.003;
// fim dos parametros PID

//Erros
float cumul=0;
float lastE=0;
float pAlphaE=0;
float pBetaE=0;
float posErrorY_last=0;
float posErrorX_last=0;

float yaw_quad = 0;
float yaw_error_last = 0;

float integradorX = 0;
float integradorY = 0;

float KqP2Controller = 2.05;
float KwP2Controller = 0.4;
float p2tau[3] = {0,0,0};
float qs[4] = {1,0,0,0};

//quaternion ref, r,p,y zero
float p0=1, p1=0, p2=0, p3=0; 

float Ts; // tempo de simulação
//filtro de kalman: orient, pos, vel
//sens: accel, ang vel

/*  dados vindos da DAQ */
float gps_xyz[3];
float drone_pos[3];//orientacao + pos (?)
float angVel[3]; //na DAQ: gx, gy, gz
/*  fim dados vindos da DAQ */

void P2Controller();

float *getObjectPosition(float *veta, float *vetb)
{
	float *vetc = (float*)malloc(3*sizeof(float));
	for(int i=0; i<3; i++)
	{
		vetc[i] = veta[i]-vetb[i];
	}
	
	return vetc;
}

float *getObjectOrientation(float *veta, float *vetb)
{
	float *vetc = (float*)malloc(3*sizeof(float));
	for(int i=0; i<3; i++)
	{
		vetc[i] = veta[i]-vetb[i];
	}
	
	return vetc;
}

#endif
