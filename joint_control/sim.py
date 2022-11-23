import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import pybullet as p
import time
import pybullet_data
import numpy as np
np.set_printoptions(precision=4, suppress=True)
import math
import modern_robotics as mr
from mr_urdf_loader import loadURDF
from functions.utils import *


urdf_name = "../model/indy7.urdf"

## MR setup
MR=loadURDF(urdf_name)
M  = np.array(MR["M"])
Slist  = np.array(MR["Slist"])
Mlist  = np.array(MR["Mlist"])
Glist  = np.array(MR["Glist"])
Blist = np.array(MR["Blist"])
jointNum = MR["actuated_joints_num"]


## pybullet env setup
p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
CONTROL_FREQ = 240.0
timeStep = 1/CONTROL_FREQ
#p.setTimeStep(timeStep)
p.setPhysicsEngineParameter(fixedTimeStep = timeStep)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
#p.setRealTimeSimulation(True)
## robot setup
robotId = p.loadURDF(urdf_name, [0, 0, 0.0],[0, 0, 0, 1],flags=p.URDF_USE_SELF_COLLISION)
p.resetBasePositionAndOrientation(robotId, [0, 0, 0], [0, 0, 0, 1])
active_joints_name,active_joints_num,eef_num=getActiveJointList(robotId)
initializeActiveJoints(robotId,active_joints_num)
## intialize 
g = np.array([0, 0,-9.8])
q = np.array([0,0,0,0,0,0])
qdot = np.array([0 ,0, 0, 0 ,0 ,0]);
qddot= np.array([0 ,0, 0, 0 ,0 ,0]);
MAX_TORQUES = np.array([431.97,431.97,197.23,79.79,79.79,79.79]) # unit [N]
torques = np.array([0,0,0,0,0,0])
Ftip = np.array([0,0,0,0,0,0])

dq = np.array([1.57,1.57,0,0,0,0]) # desired q
dqdot = np.array([0.0 ,0.0, 0.0, 0.0 ,0.0 ,0.0]); # desired qdot
dqddot = np.array([0.0 ,0.0, 0.0, 0.0 ,0.0 ,0.0]); # desired qddot

Kp = np.diag([70,70,40,25,25,18]);
Kv= np.diag([55,55,30,15,15,3]);


dataLogger = []

t = 0;
while p.isConnected():
	t = t+timeStep
	q,qdot = getJointStates(robotId,active_joints_num)
	MassMatrix =mr.MassMatrix(q,Mlist,  Glist, Slist)
	MRID =mr.InverseDynamics(q, qdot, qddot, g, Ftip, Mlist,  Glist, Slist) #inverse dynamics ,ID =  C(qdot)* qdot + G(q)

	e = dq-q
	edot = dqdot-qdot
	qddot_ref = dqddot+Kv @  edot + Kp @ e
	torques =MassMatrix @  qddot_ref + MRID
	#print(e)
	setTorques(robotId,active_joints_num,torques,MAX_TORQUES)
	dataLogger.append([t,q,qdot,e,torques])
	p.stepSimulation()
	
	

