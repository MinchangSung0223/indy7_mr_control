#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "SharedMemory/PhysicsClientSharedMemory_C_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3HashMap.h"
#include <Eigen/Dense>
#include "Utils/b3Clock.h"
#include "../lib/ModernRobotics.h"

#include <vector>
#include <iostream>


#include "Indy7.h"
using namespace std;
using namespace Eigen;
using namespace mr;

extern const int CONTROL_RATE;
const int CONTROL_RATE = 2000;
const b3Scalar FIXED_TIMESTEP = 1.0 / ((b3Scalar)CONTROL_RATE);
b3SharedMemoryCommandHandle command;
int statusType, ret;


int main()
{




    b3PhysicsClientHandle client = b3ConnectSharedMemory(SHARED_MEMORY_KEY);
    if (!b3CanSubmitCommand(client))
    {
        printf("Not connected, start a PyBullet server first, using python -m pybullet_utils.runServer\n");
        exit(0);
    }
    b3RobotSimulatorClientAPI_InternalData data;
    data.m_physicsClientHandle = client;
    data.m_guiHelper = 0;
    b3RobotSimulatorClientAPI_NoDirect sim;
    sim.setInternalData(&data);
    

    sim.resetSimulation();
    sim.setGravity( btVector3(0 , 0 ,-9.8));
    //int plane = sim.loadURDF("plane.urdf");
    int robotId = sim.loadURDF("model/indy7.urdf");    
    Indy7 indy7(&sim,robotId);
    


    sim.setTimeStep(FIXED_TIMESTEP);
    double t = 0;
    Eigen::VectorXd torques(6,1);
    Eigen::VectorXd MAX_TORQUES(6,1);    
    torques<<0.0,0.0,0.0,0.0,0.0,0.0;
    MAX_TORQUES<<431.97,431.97,197.23,79.79,79.79,79.79;

    double jointAccelerations[6]={0,0,0,0,0,0};
    double jointForcesOutput[6]={0,0,0,0,0,0};
    MatrixXd M = indy7.getM();
    MatrixXd Slist = indy7.getSlist();
    MatrixXd Blist = indy7.getBlist();
    vector<MatrixXd> Mlist = indy7.getMlist();
    vector<MatrixXd> Glist = indy7.getGlist();
    VectorXd g(3,1);
	g<<0.0,0.0,-9.8;
    VectorXd Ftip(6,1);
    	Ftip<<0,0,0,0,0,0;
    int m = indy7.getActuatedJointNum();    	
    VectorXd qddot=  VectorXd::Zero(m);
    VectorXd eint=  VectorXd::Zero(m);
        
    VectorXd dq=  VectorXd::Zero(m);
    VectorXd dqdot=  VectorXd::Zero(m);  
    VectorXd e =        VectorXd::Zero(m);  
    double dt= FIXED_TIMESTEP;
    double scale = 100;
    VectorXd Kp=  VectorXd::Zero(m);
    Kp<<70,70,40,25,25,18;

    VectorXd Ki=  VectorXd::Zero(m);
    Ki<<1,1,0.5,0.2,0.2,0.1;    
    VectorXd Kd=  VectorXd::Zero(m);    
    Kd<<55,55,30,15,15,3;
    
    MatrixXd matKp = scale*Kp.asDiagonal();
    MatrixXd matKi = scale*Ki.asDiagonal();
    MatrixXd matKd = scale*Kd.asDiagonal();        
    while(1){

	VectorXd q = indy7.getQ( &sim);
	VectorXd qdot = indy7.getQdot( &sim);	
	MatrixXd T = indy7.getEEFPose(&sim); //bullet3
	MatrixXd Tsb = FKinSpace(M,Slist,q);

	//bool ret =  sim.calculateInverseDynamics(robotId, q.data(), qdot.data(), jointAccelerations, jointForcesOutput);	
	MatrixXd Mmat = MassMatrix(q,Mlist,Glist,Slist);
	VectorXd Cmat = VelQuadraticForces(q,qdot,Mlist,Glist,Slist);
	VectorXd Gmat = GravityForces(q,g,Mlist,Glist,Slist);	
	e  = dq-q;
	eint += dt * e;
	VectorXd tau_invedrsedyn = InverseDynamics(q,qdot, qddot,g,Ftip,Mlist,Glist,Slist);
	VectorXd torque = Mmat*(matKp*e+matKi*(eint+e) + matKd*(dqdot-qdot))+tau_invedrsedyn;
	indy7.setTorques(&sim,torque,MAX_TORQUES);
	
	sim.stepSimulation();	
	b3Clock::usleep(1000. * 1000. * FIXED_TIMESTEP);
	t = t+FIXED_TIMESTEP;	
    }
    
    
    
}
