#ifndef INDY7_SETUP_H
#define INDY7_SETUP_H

#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "Bullet3Common/b3HashMap.h"
#include "../lib/ModernRobotics.h"
#include <vector>
#include <Eigen/Dense>
#include <iostream>
using namespace Eigen;
using namespace std;

class Indy7
{
	int robotId;
	int actuated_joint_num;
	int eef_num;
	vector<int> actuated_joint_id;
	vector<string> actuated_joint_name;
	MatrixXd Slist;
	MatrixXd Blist;	
	vector<MatrixXd> Glist;	
	vector<MatrixXd> Mlist;			
	MatrixXd M;				
	
public:
	Indy7(class b3RobotSimulatorClientAPI_NoDirect* sim,int robotId);
	void setTorques(class b3RobotSimulatorClientAPI_NoDirect* sim,VectorXd  torques ,VectorXd  max_torques );
	VectorXd getQ(class b3RobotSimulatorClientAPI_NoDirect* sim);	
	VectorXd getQdot(class b3RobotSimulatorClientAPI_NoDirect* sim);	
	MatrixXd getEEFPose(class b3RobotSimulatorClientAPI_NoDirect* sim);	
	void MRSetup();
	int getActuatedJointNum(){
		return this->actuated_joint_num;
	};
	MatrixXd getM();
	MatrixXd getSlist();
	MatrixXd getBlist();
	vector<MatrixXd> getMlist();
	vector<MatrixXd> getGlist();
	virtual ~Indy7();

};
#endif  //INDY7_SETUP_H
