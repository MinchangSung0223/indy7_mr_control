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
Xd_list = ScrewTrajectory(Xstart, Xend, endTime, int(endTime/dt), 5,type=0)
Vd_list = ScrewTrajectory(Xstart, Xend, endTime, int(endTime/dt), 5,type=1)

data={}
data["Xd_list"]=[]
data["Vd_list"]=[]
data["Xd_list"]=np.array(Xd_list).tolist();
data["Vd_list"]=np.array(Vd_list).tolist();
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
for t in np.arange(0,endTime,dt):
	Xd = Xd_list[idx];
	Vd = Vd_list[idx];
	q,qdot = indy7.getJointStates();
	T = FKinSpace(M,Slist,q);
	R,p = TransToRp(T);
	Rd,pd = TransToRp(Xd);
	Js,Jb,Ja,pinvJs,pinvJb, pinvJa = indy7.getJacobian(M,Slist,Blist,q);
	Vb = Jb @ qdot;
	Va = Ja @ qdot;
	diffXd =se3ToVec(MatrixLog6(TransInv(prevXd)@ Xd))
	diffXd_list.append(diffXd);
	Xe = se3ToVec(MatrixLog6(TransInv(T)@ Xd))
	qdot = pinvJb@Adjoint(TransInv(T)@Xd)@Vd*10+Kp*pinvJb@Xe
	q,qdot = EulerStep(q,qdot,qddot,dt)
	prevXd = Xd;
	idx= idx+1
	indy7.setData(t,q,T,Xd,diffXd/dt,Vb)
	indy7.setJointStates(q)
	indy7.drawEEF();
	indy7.step()
indy7.saveData();
indy7.plotData();
'''
import matplotlib.pyplot as plt
diffXd_list =np.array(diffXd_list);
Vd_list = np.array(Vd_list)
print(diffXd_list.shape)
print(Vd_list.shape)
t_list = np.linspace(0,endTime,int(endTime/dt))
print(Xstart)
print(Xend)
for i in range(0,6):
	plt.subplot(6,1,i+1)
	plt.plot(t_list,Vd_list[:,i],"r--")
	plt.plot(t_list,diffXd_list[:,i],"b:")
plt.show()
'''