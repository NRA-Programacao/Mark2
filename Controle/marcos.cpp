#include <iostream>
//~ #include <chrono>
#include <time.h>
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
	void sysCall_actuation()
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

void sysCall_actuation() // em loop
{
	
	float *targetPos;
    //~ -- Attitude 'Sensing' from Compass and qASGD scripts:
    //~ --yaw_quad = sim.getFloatSignal("ZCompassLP")  -- in radians, [-pi,pi], Yaw from ZCompassLP
    //~ --pitch_quad = sim.getFloatSignal("Pitch_qASGD") -- in rad
    //~ --roll_quad = sim.getFloatSignal("Roll_qASGD")   -- in rad

    //~ -- Vertical control:
    float dummy_array[3] = {0, 0, 0}; //referencia
    targetPos = getObjectPosition(targetObj, dummy_array);
    //~ --pos=sim.getObjectPosition(d,-1)
    float *l; //= getVelocity(heli);//problema da daq 
    float e = (targetPos[2] - gps_xyz[2]); //erro de posição
    //float//
cumul = cumul + e;
    float pv = pParam*e;
    float thrust = 5.335 + pv + iParam*cumul + dParam*(e-lastE) + l[2]*vParam;
    lastE = e;


    //~ -- P2-Controller
    P2Controller();
	
	//~ -- Rotational control:
    float *yaw_target = getObjectOrientation(targetObj, dummy_array);
    float yaw_error = yaw_quad - yaw_target[2]; //em z
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
