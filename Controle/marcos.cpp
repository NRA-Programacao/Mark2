#include <iostream>
//~ #include <chrono>
#include <time.h>
#include <math.h>
#include "marcos.h"

//~ using namespace std::chrono;
using namespace std;
//~ int main(void)
//~ {

  //~ high_resolution_clock::time_point t1 = high_resolution_clock::now();

  //~ cout << "printing out 100 stars...\n";
  //~ for (int i=0; i<100; ++i) std::cout << "*";
  //~ cout << std::endl;

  //~ high_resolution_clock::time_point t2 = high_resolution_clock::now();

  //~ duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  //~ cout << "It took me " << time_span.count() << " seconds.";
  //~ cout << std::endl;

  //~ return 0;

//~ }


// Detatch the manipulation sphere:
int main()
{
	
	clock_t start, end;
	
	//targetObj=sim.getObjectHandle('Quadricopter_target')

	//~ for(int i=0; i<3;i++)
	//~ {
		//~ targetObj[i] = setObjectParent()
	//~ }
	//sim.setObjectParent(targetObj,-1,true)

	//~ d=sim.getObjectHandle('Quadricopter_base')

	//~ propellerScripts={-1,-1,-1,-1}
	//~ for i=1,4,1 do
		//~ propellerScripts[i]=sim.getScriptHandle('Quadricopter_propeller_respondable'..i)
	//~ end
	//~ heli=sim.getObjectAssociatedWithScript(sim.handle_self)

	//~ high_resolution_clock::time_point t1 = high_resolution_clock::now();//inicio contagem tempo
	start = clock();
	

	//~ gyroCommunicationTube=sim.tubeOpen(0,'gyroData'..sim.getNameSuffix(nil),1) -- put this in the initialization phase
	//~ angVel = {0.0034,0.0022,0.0018}

	//~ attQuaternionTube = sim.tubeOpen(0,'attQuaternion'..sim.getNameSuffix(nil),1)

	//~ gpsCommunicationTube=sim.tubeOpen(0,'gpsData'..sim.getNameSuffix(nil),1) -- put this in the initialization phase
	//~ gps_xyz = sim.getObjectPosition(d,-1) -- initialize with initial position

	//~ attLog = assert(io.open("att_control.txt","w"))

	//~ Ts  = sim.getSimulationTimeStep()
	//~ high_resolution_clock::time_point t2 = high_resolution_clock::now();
	//~ duration<double> Ts = duration_cast<duration<double>>(t2 - t1);
	
	end = clock();
	double duration_sec = double(end-start)/CLOCKS_PER_SEC;
	

	//~ sim.addStatusbarMessage(string.format("Att Kp: %.4f Kd: %.4f\nPos Kp: %.4f Kd: %.4f", attKp, attKd, posKp, posKd))
	//~ end

	//~ function sysCall_cleanup() 
		//~ attLog:close()
	//~ end

	//~ function sysCall_sensing()
		// Sensors:
		//~ gyro_data=sim.tubeRead(gyroCommunicationTube)
		//~ if (gyro_data) then angVel = sim.unpackFloatTable(gyro_data) end
		//~ gps_data=sim.tubeRead(gpsCommunicationTube)
		//~ if (gps_data) then gps_xyz=sim.unpackFloatTable(gps_data) end
		//~ attQuat = sim.tubeRead(attQuaternionTube)
		//~ if (attQuat) then qs = sim.unpackFloatTable(attQuat) end

		//~ yaw_quad = sim.getFloatSignal("ZCompassLP")
	//~ end
}

void sysCall_actuation()
{
	
	float *targetPos;
    //~ -- Attitude 'Sensing' from Compass and qASGD scripts:
    //~ --yaw_quad = sim.getFloatSignal("ZCompassLP")  -- in radians, [-pi,pi], Yaw from ZCompassLP
    //~ --pitch_quad = sim.getFloatSignal("Pitch_qASGD") -- in rad
    //~ --roll_quad = sim.getFloatSignal("Roll_qASGD")   -- in rad

    //~ -- Vertical control:
    float dummy_array[3] = {0.0, 0.0, 0.0};
    targetPos = getObjectPosition(targetObj, dummy_array);
    //~ --pos=sim.getObjectPosition(d,-1)
    float *l; //= getVelocity(heli);//problema da daq
    float e = (targetPos[2] - gps_xyz[2]);
    float cumul = cumul + e;
    float pv = pParam*e;
    float thrust = 5.335 + pv + iParam*cumul + dParam*(e-lastE) + l[2]*vParam;
    lastE = e;


    //~ -- P2-Controller
    P2Controller();
	
	//~ -- Rotational control:
    float *yaw_target = getObjectOrientation(targetObj, dummy_array);
    float yaw_error = yaw_quad - yaw_target[2];
    float rotCorr = yaw_error*0.1 + 2*(yaw_error - yaw_error_last);
    yaw_error_last = yaw_error;

//~ -- Horizontal control: 
    float *posError = getObjectPosition(targetObj, drone_pos);
    
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
    
    
    //~ -- Actuation: Decide of the motor velocities:
    particlesTargetVelocities[0]=thrust*(1-alphaCorr+betaCorr+rotCorr);
    particlesTargetVelocities[1]=thrust*(1-alphaCorr-betaCorr-rotCorr);
    particlesTargetVelocities[2]=thrust*(1+alphaCorr-betaCorr+rotCorr);
    particlesTargetVelocities[3]=thrust*(1+alphaCorr+betaCorr-rotCorr);
    
    //~ -- Send the desired motor velocities to the 4 rotors:
    //~ for i=1,4,1 do
        //~ sim.setScriptSimulationParameter(propellerScripts[i],'particleVelocity',particlesTargetVelocities[i])
    //~ end
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

void modelagem()//depois melhora o nome ou divide em mais funções
{
	float massa;
	float inercia;
	
	float rho_a = 1.225;//densidade do ar [kg/m^3]
	
	float F, V, Theta_double_dot, Phi_double_dot, Psi_double_dot, a_z;
	
	F = 2*rho_a*A*pow(ceil((f*eta*K_p)/K_b), 2)*pow(V, 2);//impulso e voltagem
	
	V = (J*Z*Omega_dot)/K_q + K_e*Omega + (Z*D*pow(Omega, 2))/K_q;//velocidade angular e voltagem
	
	Theta_double_dot = ((2*l*rho_s*A)/I_xx)*pow((f*eta*K_t)/K_q, 2)*(pow(V_2, 2) - pow(V_4, 2));//roll
	
	Phi_double_dot = ((2*l*rho_s*A)/I_yy)*pow((f*eta*K_t)/K_q, 2)*(pow(V_3, 2) - pow(V_1, 2));//pitch
	
	Psi_double_dot = (J*(Omega_dot_1 + Omega_dot_3 - Omega_dot_2 - Omega_dot_4) + D*(pow(Omega_dot_1, 2) + pow(Omega_dot_3, 2) - pow(Omega_dot_2, 2) - pow(Omega_dot_4, 2)))/I_zz;//yaw
	
	a_z = (2*rho_s*A)/M*pow((f*eta*K_p)/K_q)*(pow(V_1, 2) + pow(V_2, 2) + pow(V_3, 2) + pow(V_4, 2))*cos(Theta)*cos(Phi) - g;//aceleração vertical
}

void modelagem()//depois melhora o nome ou divide em mais funções
{
float F_m[4], F[3] = {0,0,0};
// vz = w
float vz, p, q, xin[4]; // p e q é velocidade angular em pitch e roll
float U_h, U_phi, U_theta, U_psi;
float accx, accy, accz, acc_psi, acc_theta, acc_phi;
float phi, psi, theta;
float CT2s = -1.3077*pow(10,-2);
float CT1s = -2.5224*pow(10,-4);
float CT0s = 1.5393579917*pow(10,-5);
float v_1[4] = {0,0,0,0};
float l_m = 0.275; // Braço do drone
float g = 9.81;
float m = 1.477; //massa
float m_M = 0.072;                                                    // [kg]      Masse Motor + Propeller 
float m_R = 0.044;                                                    // [kg]      Masse motor
float m_C = m - 4*m_M - 2*m_R;                                        // [kg]      Masse corpo (nucleo)
float I_x = 2*m_M*pow(l_m,2) + 1/12*m_C*(2*pow(0.12,2)) + 1/12*m_R*pow((2*l_m),2);    
float I_y = I_x;                                                      
float I_z = 4*m_M*pow(l_m,2) + 1/12*m_C*(2*pow(0.12,2)) + 2*1/12*m_R*pow((2*l_m),2);; 


v_1[0] = - vz + l_m*q;
v_1[1] = - vz - l_m*p;
v_1[2] = - vz - l_m*q;
v_1[3] = - vz + l_m*p;

//força dos motores
for(int i=0; i<4; i++) 
{
// if the flow speed at infinity is negative
    if (v_1[i] < 0){
        F_m[i] = CT2s*pow(v_1[i],2) + CT1s*v_1[i]*xin[i] + CT0s*pow(xin[i],2);}
        //if the flow speed at infinity is positive
    else{
        F_m[i] = -CT2s*pow(v_1[i],2) + CT1s*v_1[i]*xin[i] + CT0s*pow(xin[i],2);
    }
    //sum up all rotor forces
    F[2] = F[2] + F_m[i];
  }
U_h = F[2];
U_phi = (F_m[1] - F_m[3])*l_m;
U_theta = (F_m[2] - F_m[0])*l_m;
U_psi = F_m[0] + F_m[2] - F_m[1] - F_m[3];
	
//acelerações
accx = U_h*(sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi))/m;
accy = U_h*(sin(psi)*sin(theta)*cos(phi) -cos(psi)*sin(phi))/m;
accz = U_h*(cos(phi)*cos(theta))/m - g;
acc_phi = U_phi/I_x;
acc_theta = U_theta/I_y;
acc_psi = U_psi/I_z;

}
	
