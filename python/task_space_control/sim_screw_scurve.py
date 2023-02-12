import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import pybullet as p
import time
from math import *
from pyScurveGenerator import *
#import modern_robotics_smc as mr_smc
from modern_robotics_smc import *
from mr_urdf_loader import *
from Indy7  import *
#-----------Scurve Setup------------------
dof = 1
dt = 0.001
so = 0.0
sf = 1.0
vo = 0.0
vf = 0.5
ao = 0.0
af = 2.5
vmax = 1.294
amax = 5
dmax = 5
j =100
traj1_list = [Trajectory(),Trajectory(),Trajectory(),Trajectory(),Trajectory(),Trajectory()]
traj2_list = [Trajectory(),Trajectory(),Trajectory(),Trajectory(),Trajectory(),Trajectory()]
for i in range(0,6):
	traj1_list[i].so = so;
	traj1_list[i].sf = sf;
	traj1_list[i].vo = vo;
	traj1_list[i].vf = vf;
	traj1_list[i].ao = ao;
	traj1_list[i].af = af;
	traj1_list[i].vmax = vmax;
	traj1_list[i].amax = amax;
	traj1_list[i].dmax = dmax;
	traj1_list[i].j = j;
sg1 = ScurveGenerator(traj1_list)
time.sleep()
traj2 = Trajectory()
traj2.so = so;
traj2.sf = sf;
traj2.vo = vf;
traj2.vf = vo;
traj2.ao = af;
traj2.af = ao;
traj2.vmax = vmax;
traj2.amax = amax;
traj2.dmax = dmax;
traj2.j = j;
sg2 = ScurveGenerator([traj2])
time.sleep(1.0)


#--------------------------------------------
## Env Setup
CONTROL_FREQ = 1000.0
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
Rend,pend=TransToRp(Xend)

theta = pi/6.0
Xend2 = RpToTrans(Xend[0:3,0:3] @ EulerXYZ(theta,theta,-theta), pend+np.array([-0.1,-0.1,-0.1]))

Xd1_list = ScrewScurveTrajectory(Xstart, Xend,sg1,dt,type=0)
Vd1_list = ScrewScurveTrajectory(Xstart, Xend,sg1,dt,type=1)
dVd1_list = ScrewScurveTrajectory(Xstart, Xend,sg1,dt,type=2)
print(Vd1_list[-1])
print(se3ToVec(MatrixLog6(TransInv(Xstart)@Xend))*vf)
print(se3ToVec(MatrixLog6(TransInv(Xstart)@Xend))*vf)
v_end =se3ToVec(MatrixLog6(TransInv(Xstart)@Xend))*traj1.vf;
v_start = se3ToVec(MatrixLog6(TransInv(Xend)@Xend2))

a_end =se3ToVec(MatrixLog6(TransInv(Xstart)@Xend))*traj1.af;
a_start = se3ToVec(MatrixLog6(TransInv(Xend)@Xend2))



print(v_end)
print(v_start)
print(v_end[-1]/v_start[-1])
idx = 2
traj2 = Trajectory()
traj2.so = so;
traj2.sf = sf;
traj2.vo = v_end[idx]/v_start[idx];
traj2.vf = vo;
traj2.ao = a_end[idx]/a_start[idx];
traj2.af = ao;
traj2.vmax = vmax;
traj2.amax = amax;
traj2.dmax = dmax;
traj2.j = j;
sg2 = ScurveGenerator([traj2])




Xd2_list = ScrewScurveTrajectory(Xend, Xend2,sg2,dt,type=0)
Vd2_list = ScrewScurveTrajectory(Xend, Xend2,sg2,dt,type=1)
dVd2_list = ScrewScurveTrajectory(Xend, Xend2,sg2,dt,type=2)
print(se3ToVec(MatrixLog6(TransInv(Xend)@Xend2))*traj2.vf)
print(Vd2_list[-1])
print(se3ToVec(MatrixLog6(TransInv(Xend)@Xend2))*traj2.vo)
print(Vd2_list[0])

#time.sleep(100.0)
s_traj1 = sg1.getTraj(0);
s_traj2 = sg2.getTraj(0);
Tf = s_traj1.tt+s_traj2.tt
N = int(Tf/dt)
endTime = Tf;


Xd_list = []
Vd_list = []
dVd_list = []
for i in range(0,len(Xd1_list)):
	Xd_list.append(Xd1_list[i])
	Vd_list.append(Vd1_list[i])
	dVd_list.append(dVd1_list[i])
for i in range(0,len(Xd2_list)):
	Xd_list.append(Xd2_list[i])
	Vd_list.append(Vd2_list[i])
	dVd_list.append(dVd2_list[i])


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
q,qdot = indy7.getJointStates();
Js,Jb,Ja,pinvJs,pinvJb, pinvJa = indy7.getJacobian(M,Slist,Blist,q);
Vb = Jb @ qdot;
prevVb = Vb;
prevdVb = Vb-prevVb;
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
	dVb = (Vb-prevVb)/dt
	dVb = 0.95*prevdVb + 0.05*dVb 
	diffXd =se3ToVec(MatrixLog6(TransInv(prevXd)@ Xd))
	diffXd_list.append(diffXd);
	Xe = se3ToVec(MatrixLog6(TransInv(T)@ Xd))
	qdot = pinvJb@Adjoint(TransInv(T)@Xd)@Vd*10+Kp*pinvJb@Xe
	q,qdot = EulerStep(q,qdot,qddot,dt)
	prevXd = Xd;
	prevVb = Vb;
	prevdVb = dVb;

	if idx+1 < len(Xd_list):
			idx= idx+1 

	indy7.setData(t,q,T,Xd,Vb,Vd,dVb,dVd)
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