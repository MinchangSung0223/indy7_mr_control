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
def JointVelTrajectory(thetastart, thetaend, Tf, N, method):
    """Computes a straight-line trajectory in joint space
    :param thetastart: The initial joint variables
    :param thetaend: The final joint variables
    :param Tf: Total time of the motion in seconds from rest to rest
    :param N: The number of points N > 1 (Start and stop) in the discrete
              representation of the trajectory
    :param method: The time-scaling method, where 3 indicates cubic (third-
                   order polynomial) time scaling and 5 indicates quintic
                   (fifth-order polynomial) time scaling
    :return: A trajectory as an N x n matrix, where each row is an n-vector
             of joint variables at an instant in time. The first row is
             thetastart and the Nth row is thetaend . The elapsed time
             between each row is Tf / (N - 1)
    Example Input:
        thetastart = np.array([1, 0, 0, 1, 1, 0.2, 0,1])
        thetaend = np.array([1.2, 0.5, 0.6, 1.1, 2, 2, 0.9, 1])
        Tf = 4
        N = 6
        method = 3
    Output:
        np.array([[     1,     0,      0,      1,     1,    0.2,      0, 1]
                  [1.0208, 0.052, 0.0624, 1.0104, 1.104, 0.3872, 0.0936, 1]
                  [1.0704, 0.176, 0.2112, 1.0352, 1.352, 0.8336, 0.3168, 1]
                  [1.1296, 0.324, 0.3888, 1.0648, 1.648, 1.3664, 0.5832, 1]
                  [1.1792, 0.448, 0.5376, 1.0896, 1.896, 1.8128, 0.8064, 1]
                  [   1.2,   0.5,    0.6,    1.1,     2,      2,    0.9, 1]])
    """
    N = int(N)
    timegap = Tf / (N - 1.0)
    traj = np.zeros((len(thetastart), N))
    for i in range(N):
        if method == 3:
            sdot = CubicTimeScalingDot(Tf, timegap * i)
        else:
            sdot = QuinticTimeScalingDot(Tf, timegap * i)
        traj[:, i] = sdot * np.array(thetaend) - sdot * np.array(thetastart)
    traj = np.array(traj).T
    return traj

def computeImpedanceControlTorq(des_q,des_qdot,q,qdot):
	n = len(q)
	Kp=np.diag([70,70,25,25,25,5])*200
	Kd=np.diag([5*40,5*40,3*20,3*10,3*10,0.5])
	tauImp = Kp.dot(des_q-q)+Kd.dot(des_qdot-qdot);
	return tauImp
def computeCTMControlTorq(robotID,des_q,des_qdot,q,qdot,Mmat,C,tauGrav):
	#Kv=np.diag([70,70,40,25,25,18])*0.1
	#Kp=np.diag([55,55,30,15,15,3])*0.1
	Kp = np.diagflat([70,70,10,10,10,2])*500
	Kv = np.diagflat([55,55,1,1,1,0.1])*30
	e = des_q-q
	edot = des_qdot-qdot;
	qddot_ref=Kv.dot(edot)+Kp.dot(e)
	#M = np.array(p.calculateMassMatrix(robotID, [q[0],q[1],q[2],q[3],q[4],q[5]]))
	tauCTM = Mmat.dot( qddot_ref) +C+tauGrav;	
	return tauCTM	
qddot = np.zeros([6,1])
g = np.array([0,0,-9.8])
Ftip = [0,0,0,0,0,0]
q,qdot = indy7.getJointStates();
des_q = indy7.HomePos;

print(q)
print(des_q)
des_q_traj = JointTrajectory(q, des_q, endTime, int(endTime/dt), 5)
des_qdot_traj = JointVelTrajectory(q, des_q, endTime, int(endTime/dt), 5)

#des_q = np.array([ 0.   ,  -0.8893, -2.2512 , 0.0003, -0.0005 ,-0.0003]).T;

K= np.diag([1.0,1.0,1.0,1.0,1.0,1.0]);
#K= np.array([5.0,5.0,5.0,5.0,5.0,5.0]);
KC=np.diag([80,80,40,25,25,25])*0.01;
KD=np.diag([55,55,30,15,15,1])*0.01;

q_norm = np.array([q[0],q[1],q[2],q[3],q[4],q[5]]).T;
qdot_norm = np.array([qdot[0],qdot[1],qdot[2],qdot[3],qdot[4],qdot[5]]).T;
qddot_norm = qddot;
idx = 0;
des_q = np.array(des_q_traj[0,:]).T;
des_qdot = np.array(des_qdot_traj[0,:]).T;
for t in np.arange(0,endTime,dt):
	#des_q = np.array(des_q_traj[idx,:]).T;
	#des_qdot = np.array(des_qdot_traj[idx,:]).T;
	q,qdot = indy7.getJointStates();
	T = FKinSpace(M,Slist,q);
	T_norm = FKinSpace(M,Slist,q_norm);
	Js,Jb,Ja,pinvJs,pinvJb, pinvJa = indy7.getJacobian(M,Slist,Blist,q);
	tauGrav =InverseDynamics(q, qdot, qddot, g, Ftip, Mlist,  Glist, Slist,eef_mass = 0)	
	MassMat = MassMatrix(q,Mlist,Glist,Slist);
	tauGrav = GravityForces(q, g, Mlist, Glist, Slist);
	#M_ = np.array(p.calculateMassMatrix(indy7.robotId, [q[0],q[1],q[2],q[3],q[4],q[5]]))

	Coriolis_norm =  VelQuadraticForces(q_norm, qdot_norm, Mlist, Glist, Slist)
	Coriolis =  VelQuadraticForces(q, qdot, Mlist, Glist, Slist)
	Mhat = MassMatrix(q_norm,Mlist,Glist,Slist);

	indy7.applyForce([0,0,2000])
	tauExt  = indy7.getFT();
	print(tauExt)
	#tauExt = np.array([0,0,0,0,0,0]).T
	tauImp = computeImpedanceControlTorq(des_q,des_qdot,q,qdot)
	D = 0.0
	invMhat = np.linalg.pinv(Mhat);
	#print(invMhat)
	qddot_norm = invMhat.dot(-Coriolis_norm+K.dot(tauImp +tauExt))
	qddot_norm[-1]=0
	#print(qddot_norm)
	#Mhat * qddot_nom+Cqdot = (-( 3.0*D)*_robotNom->qdot() + K.cwiseProduct(tauImp + tauExt));
	#print(qddot_norm.shape)
	#print(qddot_norm)
	q_norm,qdot_norm=EulerStep(q_norm,qdot_norm,qddot_norm,dt);
	err = q_norm - q;
	err_dot = qdot_norm - qdot;
	
	#print(err)
	tauAux = KC.dot(err) + KD.dot(err_dot);
	#print(tauAux)
	#print(np.linalg.inv(Jb)@(tauGrav- 0.1*tauExt+K@(tauImp+tauAux+tauExt*0.1)))
	torques = tauGrav- tauExt+K@(tauImp+tauAux+tauExt);

	tauCTM = computeCTMControlTorq(indy7.robotId,des_q,des_qdot,q,qdot,MassMat,Coriolis,tauGrav)
	#torques = tauGrav+tauImp;
	
	
	#indy7.setTorques(torques)

	indy7.setTorques(tauImp)
	#indy7.setJointStates(des_q)
	idx = idx+1;
	indy7.drawEEF();	
	indy7.step()
