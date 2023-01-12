import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import pybullet as p
import time
from math import *

import modern_robotics_smc as mr_smc
from mr_urdf_loader import *
from Indy7  import *
#--------------------------------------------
## Env Setup
CONTROL_FREQ = 1000.0
endTime = 20;
dt = 1/CONTROL_FREQ;
urdf_name = "../../model/indy7.urdf"
indy7 = Indy7(p,urdf_name = urdf_name, CONTROL_FREQ=CONTROL_FREQ)
indy7.resetJointStates(indy7.HomePos);
p.loadURDF("plane.urdf")
#--------------------------------------------
## Modern Robotics setup
MR=loadURDF(urdf_name)
M  = MR["M"]
Slist  = MR["Slist"]
Mlist  = MR["Mlist"]
Glist  = MR["Glist"]
Blist  = MR["Blist"]
#--------------------------------------------

qddot = np.zeros([6,1])
g = np.array([0,0,-9.8])
Ftip = [0,0,0,0,0,0]

for t in np.arange(0,endTime,dt):
	q,qdot = indy7.getJointStates();
	T = FKinSpace(M,Slist,q);
	Js,Jb,Ja,pinvJs,pinvJb, pinvJa = indy7.getJacobian(M,Slist,Blist,q);
	gravity_torques =InverseDynamics(q, qdot, qddot, g, Ftip, Mlist,  Glist, Slist,eef_mass = 0)	
	torques = gravity_torques;
	indy7.setTorques(gravity_torques)
	indy7.drawEEF();
	indy7.step()
