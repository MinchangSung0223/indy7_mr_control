import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import pybullet as p
import time
from math import *

#import modern_robotics_smc as mr_smc
from modern_robotics_smc import *
from mr_urdf_loader import *
from Indy7  import *


#--------------------------------------------
## Env Setup
CONTROL_FREQ = 240.0
endTime = 4;
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
q,qdot = indy7.getJointStates();
Xstart = FKinSpace(M,Slist,q);
Rstart,pstart=TransToRp(Xstart)
print(Xstart)

theta = pi/3.0
Xend = RpToTrans(Xstart[0:3,0:3] @ EulerXYZ(theta,theta,-theta), pstart+np.array([0.1,0.1,0.1]))
print(Xend)
Xd_list = CartesianTrajectory(Xstart, Xend, endTime, int(endTime/dt), 5,type=0)
Vd_list = CartesianTrajectory(Xstart, Xend, endTime, int(endTime/dt), 5,type=1)
dVd_list = CartesianTrajectory(Xstart, Xend, endTime, int(endTime/dt), 5,type=2)

data={}
data["Xd_list"]=[]
data["Vd_list"]=[]
data["Xd_list"]=np.array(Xd_list).tolist();
data["Vd_list"]=np.array(Vd_list).tolist();
data["dVd_list"]=np.array(dVd_list).tolist();

with open("trajectory.json", 'w') as outfile:
    json.dump(data, outfile)

idx= 0;
qddot = np.zeros([6,1])

Kp =2000.0;
Kd = Kp/30;
Ki = Kp/100;
orn_scale = 1;
indy7.drawTrajectory(Xd_list,50)
prevXd=Xd_list[0]
diffXd_list = []
q,qdot = indy7.getJointStates();
Js,Jb,Ja,pinvJs,pinvJb, pinvJa = indy7.getJacobian(M,Slist,Blist,q);
prevVb = Jb @ qdot;
prevVa = Ja @ qdot;
for t in np.arange(0,endTime,dt):
	Xd = Xd_list[idx];
	Vd = Vd_list[idx];
	dVd = dVd_list[idx];
	q,qdot = indy7.getJointStates();
	T = FKinSpace(M,Slist,q);
	R,p = TransToRp(T);
	Rd,pd = TransToRp(Xd);
	Js,Jb,Ja,pinvJs,pinvJb, pinvJa = indy7.getJacobian(M,Slist,Blist,q);
	Vb = Jb @ qdot;
	Va = Ja @ qdot;
	dVa = (Va-prevVa)/dt
	dVb = (Vb-prevVb)/dt
	Xe= np.r_[so3ToVec(MatrixLog3(R.T@Rd)),np.array(pd-p).T]
	diffXd =se3ToVec(MatrixLog6(TransInv(prevXd)@ Xd))
	diffXd_list.append(diffXd);
	#Vd[3:6] = R.T@Vd[3:6]  # analytic twist -> body twist
	qdot = pinvJa@Vd*10 + Kp*pinvJa@Xe
	q,qdot = EulerStep(q,qdot,qddot,dt)
	prevXd = Xd;
	prevVb = Vb;
	prevVa = Va;
	idx= idx+1
	indy7.setData(t,q,T,Xd,diffXd/dt,Vb,np.array([dVb[0],dVb[1],dVb[2], dVa[3], dVa[4], dVa[5]]).T,dVd)
	indy7.setJointStates(q)
	indy7.drawEEF();
	indy7.step()
indy7.saveData();
indy7.plotData();
