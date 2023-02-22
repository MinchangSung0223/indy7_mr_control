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
endTime = 1;
dt = 1/CONTROL_FREQ;
urdf_name = "../../model/indy7.urdf"
indy7 = Indy7(p,urdf_name = urdf_name, CONTROL_FREQ=CONTROL_FREQ)
indy7.resetJointStates(np.array([ 0.   ,  -0.8893, -2.2512 , 0.0003, -0.0005 ,-0.0003]).T);
p.loadURDF("../../model/plane.urdf")
p.loadURDF("../../model/axis.urdf")
#--------------------------------------------
## Modern Robotics setup
MR=loadURDF(urdf_name)
M  = MR["M"]
Slist  = MR["Slist"]
Mlist  = MR["Mlist"]
Glist  = MR["Glist"]
Blist  = MR["Blist"]


#--------------------------------------------
def computeImpedanceControlTorq(des_pos,des_vel, q, qdot,Xe,pinvJb):
	tauImp = np.array([0,0,0,0,0,0]).T
	pos = np.array(T[0:3,3]).T
	err = des_pos-pos;
	err_dot = np.array([0,0,0]).T;
	Kp = np.diag([70,70,25,25,25,15])
	Kd = np.diag([5,5,5,5,5,0.1])
	#print(pos.shape)
	pinvJb_pos = pinvJb[:,3:6]
	tauImp = Kp.dot(pinvJb_pos.dot(err))-Kd.dot(qdot);
	return tauImp

qddot = np.zeros([6,1])
g = np.array([0,0,-9.8])
Ftip = [0,0,0,0,0,0]
q,qdot = indy7.getJointStates();
des_q = indy7.HomePos;
#des_q = np.array([ 0.   ,  -0.8893, -2.2512 , 0.0003, -0.0005 ,-0.0003]).T;

K= np.diag([1.0,1.0,1.0,1.0,1.0,1.0]);
#K= np.array([5.0,5.0,5.0,5.0,5.0,5.0]);
KC=np.diag([80,80,40,25,25,25])*0.01;
KD=np.diag([55,55,30,15,15,1])*0.01;

q_nom = np.array([q[0],q[1],q[2],q[3],q[4],q[5]]).T;
qdot_nom = np.array([qdot[0],qdot[1],qdot[2],qdot[3],qdot[4],qdot[5]]).T;
qddot_nom = qddot;
des_pos = np.array([0.35,-0.2,0.1]).T
des_vel = np.array([0,0,0]).T
targetT = np.array([[-1.     , 0.0001 , 0.0006  ,0.3499],
 		[ 0.0001,  1.   ,   0.     ,-0.1865],
 [-0.0006 , 0.   ,  -1.   ,   0.5 ],
 [ 0.  ,    0. ,     0.    ,  1.    ]])
def DerivativeJacobianBody_(Blist,thetalist):
    n = len(thetalist)
    dvecJb = np.zeros([6*n,n])
    Jb = JacobianBody(Blist,thetalist);
    for i in range(0,n):
            for j in range(0,n):    
                if(i>j):
                    Jb_j = Jb[:,j]
                    Jb_i = Jb[:,i]      
                    dvecJb[6*j:6*j+6,i] = ad(Jb_j)@Jb_i              
    return dvecJb
for t in np.arange(0,endTime,dt):
	q,qdot = indy7.getJointStates();
	T = FKinSpace(M,Slist,q);
	T_nom = FKinSpace(M,Slist,q_nom);
	Xe = se3ToVec(MatrixLog6(TransInv(T)@targetT))
	Js,Jb,Ja,pinvJs,pinvJb, pinvJa = indy7.getJacobian(M,Slist,Blist,q);
	Jbdot = DerivativeJacobianBody_(Blist,q)
	MassMat = MassMatrix(q,Mlist,Glist,Slist);
	#MassMat_nom = MassMatrix(q_nom,Mlist,Glist,Slist);
	
	#Coriolis =  VelQuadraticForces(q, qdot, Mlist, Glist, Slist)
	#Coriolis_nom =  VelQuadraticForces(q_nom, qdot_nom, Mlist, Glist, Slist)
	
	#tauGrav = GravityForces(q, g, Mlist, Glist, Slist);
	#tauGrav_nom = GravityForces(q_nom, g, Mlist, Glist, Slist);

	H = InverseDynamics(q, qdot, qddot, g, Ftip, Mlist, \
                    Glist, Slist)

	#indy7.applyForce([0,0,-100])
	tauExt  = indy7.getFT();
	#print(tauExt)
	#tauImp = computeImpedanceControlTorq(des_pos,des_vel, q, qdot,Xe,pinvJb)
	qddot_nom = np.array([0,0,0,0,0,0]).T
	#qddot_nom = np.linalg.inv(MassMat_nom)@(-(Coriolis_nom)+K.dot(tauImp+tauExt))
	q_nom,qdot_nom = EulerStep(q_nom,qdot_nom,qddot_nom,dt)
	err = q_nom-q
	err_dot = qdot_nom-qdot
	#print(err)
	Lambda = pinvJb.T @ MassMat @ pinvJb;
	Vb = Jb@qdot
	#print(H.shape)
	eta = pinvJb.T @H 
	#Xe[0:3] = Xe[0:3] /30.0
	#Vb[0:3] = Vb[0:3] /30.0
	torques = Jb.T @ (Lambda@(Xe*500.0-150.0*Vb)+tauExt)+H
	#print(torques)
	indy7.setTorques(torques)
	#indy7.setJointStates(des_q)

	indy7.drawEEF();	
	indy7.step()
