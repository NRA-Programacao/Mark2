#ifndef MARCOS_BIB
#define MARCOS_BIB

float targetObj[3];//x, y, z

/*  dados vindos da DAQ */
float gps_xyz[3]; //posição
float drone_pos[3];//orientacao + pos (?)
float angVel[3];
/*  fim dados vindos da DAQ */

float particlesTargetVelocities[4] = {0, 0, 0, 0}; //força do motor

// Constantes de PID //
int pParam=2;
int iParam=0;
int dParam=0;
int vParam=-2;
//------------------//
float attKp = 0.25;
float attKd = 2.10;
float posKp = 0.005;
float posKd = 1.000;
float posKi = 0.003;
// Fim das constantes de PID //

//Variaveis de erro e integradores//
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
//Fim das variaveis de erro e integradores//

float KqP2Controller = 2.05;
float KwP2Controller = 0.4;
float p2tau[3] = {0,0,0};
float qs[4] = {1,0,0,0}; //quaternion de referencia

float p0=1, p1=0, p2=0, p3=0; //quaternion ref, r,p,y zero

float Ts;

float *getObjectPosition(float *veta, float *vetb) //retorna o erro de posição entre o drone e o target
{
	float *vetc = (float*)malloc(3*sizeof(float));
	for(int i=0; i<3; i++)
	{
		vetc[i] = veta[i]-vetb[i];
	}
	
	return vetc;
}

float *getObjectOrientation(float *veta, float *vetb) //retorna o erro de orientação entre o drone e o target
{
	float *vetc = (float*)malloc(3*sizeof(float));
	for(int i=0; i<3; i++)
	{
		vetc[i] = veta[i]-vetb[i];
	}
	
	return vetc;
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
    for(int i=0; i<3; i++)
    {
        p2tau[i] = -KwP2Controller*angVel[i] - KqP2Controller*qErr[i];
    }

}

#endif
