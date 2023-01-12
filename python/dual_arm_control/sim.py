import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import pybullet as p
import time
from math import *
from modern_robotics import *
from Indy7  import Indy7

CONTROL_FREQ = 240.0
endTime = 20;
dt = 1/CONTROL_FREQ;

urdf_name = "../../model/indy7.urdf"
indy_r = Indy7(p,urdf_name = urdf_name, CONTROL_FREQ=CONTROL_FREQ,isconnect = False ,baseXYZ = [0 ,0.15634, 0.37722],baseRPY=[-1.047197333333333 ,0 ,0])
indy_l = Indy7(p,urdf_name = urdf_name, CONTROL_FREQ=CONTROL_FREQ,isconnect = True ,baseXYZ = [0 ,-0.15634 ,0.37722],baseRPY=[-1.047197333333333 ,0 ,3.141592])
bodyId = p.loadURDF("../../model/body.urdf")
planeId = p.loadURDF("plane.urdf",[0, 0, -0.52])
boxId = p.loadURDF("../../model/box.urdf",[0.4, 0, 0.1])


baseSlist,baseBlist,baseM,relSlist,relBlist,relM,Tbr,Tbl,Mlist,Glist=indy_r.MRSetup("./MR_info.json");

indy_r.resetJointStates(np.array( [-0.3532, -1.1644 ,-1.2237, -0.7882, -0.7229,  0.0695]).T)
indy_l.resetJointStates(np.array( [ 0.3531 , 1.1645  ,1.2237 , 0.788  , 0.723 , -0.0696]).T)

Orn_Kp = 20.0/30.0;
Orn_Kd = Orn_Kp/50.0;
Orn_Ki = Orn_Kp/100.0;

Pos_Kp = 20.0
Pos_Kd = Pos_Kp/50.0;
Pos_Ki = Pos_Kp/100.0;
Xe_rel_sum = np.array([0,0,0,0,0,0]).T

for t in np.arange(0,endTime,dt):

	q_r,qdot_r = indy_r.getJointStates();
	q_l,qdot_l = indy_l.getJointStates();
	q_rel = np.r_[-np.flip(q_r),q_l]
	qdot_rel = np.r_[-np.flip(qdot_r),qdot_l]

	Js_r,Jb_r,Ja_r,pinvJs_r,pinvJb_r,pinvJa_r = indy_r.getJacobian(baseM,baseSlist,baseBlist,q_r)
	Js_l,Jb_l,Ja_l,pinvJs_l,pinvJb_l,pinvJa_l = indy_l.getJacobian(baseM,baseSlist,baseBlist,q_l)
	Js_rel,Jb_rel,Ja_rel,pinvJs_rel,pinvJb_rel,pinvJa_rel = indy_r.getJacobian(relM,relSlist,relBlist,q_rel)
	J_pb_r =  p.calculateJacobian(indy_r.robotId,indy_r.eef_num,[0,0,0],[q_r[0],q_r[1],q_r[2],q_r[3],q_r[4],q_r[5]],[qdot_r[0],qdot_r[1],qdot_r[2],qdot_r[3],qdot_r[4],qdot_r[5]],[0,0,0,0,0,0])
	J_pb_r = np.r_[np.array(J_pb_r[0]),np.array(J_pb_r[1])]

	T_r = Tbr@ FKinBody(baseM,baseBlist,q_r);
	T_l = Tbl@ FKinBody(baseM,baseBlist,q_l);
	T_rel = FKinBody(relM,relBlist,q_rel);

	Vb_r = Jb_r @ qdot_r;
	Vb_l = Jb_l @ qdot_l;
	Vs_r = Js_r @ qdot_r;
	Vs_l = Js_l @ qdot_l;
	Vs_rel = Js_rel @ qdot_rel;
	Vb_rel = Jb_rel @ qdot_rel;

	Xd = np.array([[-1 ,0 ,0 ,0],[0,1 ,0 ,0],[0 ,0 ,-1 ,0.5*exp(-t)],[0 ,0 ,0 ,1]])
	Xe_rel = se3ToVec(MatrixLog6(TransInv(T_rel)@ Xd))
	V_rel = Jb_rel@qdot_rel
	Xedot_rel = Adjoint(TransInv(T_rel)@ Xd) @ np.array([0,0,0,0,0,0]).T -V_rel
	Xe_rel_sum = Xe_rel_sum+Xe_rel*dt;

	Kp_Xe_rel = Xe_rel
	Kp_Xe_rel[0:3] = Orn_Kp*Xe_rel[0:3] 
	Kp_Xe_rel[3:6] = Pos_Kp*Xe_rel[3:6] 
	Ki_Xe_rel = Xe_rel_sum
	Ki_Xe_rel[0:3] = Orn_Ki*Xe_rel_sum[0:3] 
	Ki_Xe_rel[3:6] = Pos_Ki*Xe_rel_sum[3:6] 
	Kd_Xe_rel = Xedot_rel
	Kd_Xe_rel[0:3] = Orn_Kd*Xedot_rel[0:3] 
	Kd_Xe_rel[3:6] = Pos_Kd*Xedot_rel[3:6] 

	Xe_rell_All = Kp_Xe_rel+Kd_Xe_rel+np.tanh(Ki_Xe_rel)
	qdot_rel = 20*pinvJb_rel @ Xe_rell_All*0
	q_rel,qdot_rel=EulerStep(q_rel,qdot_rel,np.zeros((12,1)),dt)	

	F_r = indy_r.getFT();
	F_l = indy_l.getFT();
	#print("F_r : ",F_r)
	#print("F_l : ",F_l)
	F_r_ = np.array([F_r[3],F_r[4],F_r[5],F_r[0],F_r[1],F_r[2]]).T
	F_l_ = np.array([F_l[3],F_l[4],F_l[5],F_l[0],F_l[1],F_l[2]]).T

	indy_r.applyForce([0,0,0])
	indy_l.applyForce([0,0,0])

	tau_r_pb_ID= np.array(p.calculateInverseDynamics(indy_r.robotId,[q_r[0],q_r[1],q_r[2],q_r[3],q_r[4],q_r[5] ],[qdot_r[0],qdot_r[1],qdot_r[2],qdot_r[3],qdot_r[4],qdot_r[5] ] ,[0,0,0,0,0,0] ))
	tau_l_pb_ID= np.array(p.calculateInverseDynamics(indy_l.robotId,[q_l[0],q_l[1],q_l[2],q_l[3],q_l[4],q_l[5] ],[qdot_l[0],qdot_l[1],qdot_l[2],qdot_l[3],qdot_l[4],qdot_l[5] ] ,[0,0,0,0,0,0] ))

	tau_r_ID =InverseDynamics(q_r, qdot_r, np.array([0,0,0,0,0,0]).T,np.array([0,8.495709,-4.905000]), np.array([0,0,0,0,0,0]).T, Mlist,  Glist, baseSlist);
	tau_l_ID =InverseDynamics(q_l, qdot_l, np.array([0,0,0,0,0,0]).T,np.array([0,8.495709,-4.905000]), np.array([0,0,0,0,0,0]).T, Mlist,  Glist, baseSlist);

	tau_r= indy_r.getActuatedTorques();
	tau_l= indy_l.getActuatedTorques();

	q_r = -np.flip(q_rel[0:6])
	q_l = q_rel[6:12]
	#print("q_r:",q_r)
	#print("q_l:",q_l)
	indy_r.drawEEF();
	indy_l.drawEEF();
	F_r  = np.array([0,0,0.001*cos(t),0,0,10]).T
	F_l  = np.array([0,0,-0.001*cos(t),0,0,10]).T
	tau_r = tau_r_ID+Jb_r.T@F_r;
	tau_l = tau_l_ID+Jb_l.T@F_l;

	#indy_r.setJointStates(q_r);
	#indy_l.setJointStates(q_l);
	#tau_r= indy_r.getActuatedTorques();
	#tau_l= indy_l.getActuatedTorques();
	indy_r.setTorques(tau_r);
	indy_l.setTorques(tau_l);

	#indy_r.step()
	indy_l.step()

