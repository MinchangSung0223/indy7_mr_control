import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import pybullet as p
import time
from math import *
from modern_robotics import *
import modern_robotics_smc as mr_smc

from mr_urdf_loader import *
from Indy7  import *
pi = np.pi

CONTROL_FREQ = 240.0
dt = 1.0/CONTROL_FREQ
endTime = 50
force_scaling_factor = 0.05
eul_x = 0.0
eul_y = 0.0
eul_z = 0.0
urdf_name = "../../model/indy7.urdf"
indy7 = Indy7(p,urdf_name = urdf_name, CONTROL_FREQ=CONTROL_FREQ,baseXYZ = [0 ,2,0])
indy7.resetJointStates(indy7.HomePos);
MR=loadURDF(urdf_name)
M  = MR["M"]
Slist  = MR["Slist"]
Mlist  = MR["Mlist"]
Glist  = MR["Glist"]
Blist  = MR["Blist"]


#p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER,1)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS,0)
p.resetDebugVisualizerCamera(1.0,90,0,[0,0,0.0])
#p.setTimeStep(dt)
#p.setPhysicsEngineParameter(dt)
#p.setGravity(0, 0, -9.8)

quat = p.getQuaternionFromEuler([eul_x,eul_y,eul_z])
d = -0.25
R = p.getMatrixFromQuaternion(quat)
R = np.reshape(R,[3,3])
n = R@np.array([0,0,1]).T
nx = R@np.array([1,0,0]).T
ny = R@np.array([0,1,0]).T
nz = R@np.array([0,0,1]).T
boxId = p.loadURDF("box.urdf",[0.0, 0, 0.0],quat,useFixedBase=True)


#boxId = p.loadURDF("sphere.urdf",[0.0, 0, 0.0],quat,useFixedBase=True)
#boxId = p.loadURDF("cylinder.urdf",[0.0, 0, 0.0],quat,useFixedBase=True)
box2Id = p.loadURDF("object.urdf",[0.0, 0, 0.5],quat)

p.enableJointForceTorqueSensor(boxId,0,True)


x_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0],
                            lineToXYZ=[0, 0, 0],
                            lineColorRGB=[1,0,0],
                            lineWidth=3,
                            lifeTime=dt)
y_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0],
                            lineToXYZ=[0, 0, 0],
                            lineColorRGB=[0,1,0],
                            lineWidth=3,
                            lifeTime=dt)
z_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0],
                            lineToXYZ=[0, 0, 0],
                            lineColorRGB=[0,1,0],
                            lineWidth=3,
                            lifeTime=dt) 
pos_x_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0],
                            lineToXYZ=[0, 0, 0],
                            lineColorRGB=[1,0,0],
                            lineWidth=3,
                            lifeTime=dt)
pos_y_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0],
                            lineToXYZ=[0, 0, 0],
                            lineColorRGB=[0,1,0],
                            lineWidth=3,
                            lifeTime=dt)
pos_z_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0],
                            lineToXYZ=[0, 0, 0],
                            lineColorRGB=[0,1,0],
                            lineWidth=3,
                            lifeTime=dt) 



p.stepSimulation()
jointState = np.array(p.getJointState(boxId,0)[2]).T
initial_Wrench= np.array([jointState[3],jointState[4],jointState[5],jointState[0],jointState[1],jointState[2]]).T  
r = [0,0,0]

qddot = np.zeros([6,1])
g = np.array([0,0,-9.8])
Ftip = [0,0,0,0,0,0]
Kp = np.diag([100,100,100,100,100,100])

Kv = np.diag([0.1,0.1,0.1,0.1,0.1,0.1])

qd = indy7.HomePos
for t in np.arange(0,endTime,dt):
	jointState = np.array(p.getJointState(boxId,0)[2]).T
	Wrench= np.array([jointState[3],jointState[4],jointState[5],jointState[0],jointState[1],jointState[2]]).T -initial_Wrench
	Wrench = Adjoint(RpToTrans(R,np.array([0,0,0]).T))@Wrench
	f = np.array([Wrench[3],Wrench[4],Wrench[5]])
	norm_f= np.linalg.norm(f)
	u = f/norm_f;
	f_nx = np.dot(u,nx)*nx*norm_f*force_scaling_factor
	f_ny = np.dot(u,ny)*ny*norm_f*force_scaling_factor
	f_nz = np.dot(u,nz)*nz*norm_f*force_scaling_factor

	U = np.array([[0 ,-u[2] ,u[1]],[u[2] ,0 ,-u[0]],[-u[1], u[0], 0],[n[0], n[1], n[2]]])
	try:
		r = np.linalg.inv(U.T @ U)@U.T @np.array([-Wrench[0]/norm_f,-Wrench[1]/norm_f,-Wrench[2]/norm_f,-d ])
	except:
		r = [0,0,0]
		
	uf_nx = f_nx/np.linalg.norm(f_nx)
	uf_ny = f_ny/np.linalg.norm(f_ny)
	uf_nz = f_nz/np.linalg.norm(f_nz)
	p.addUserDebugLine([r[0]+10*uf_nx[0],r[1]+10*uf_nx[1],r[2]+10*uf_nx[2]],[r[0]-10*uf_nx[0],r[1]-10*uf_nx[1],r[2]-10*uf_nx[2]],[1,0,0],1,dt*10,replaceItemUniqueId=indy7.x_lineId)
	p.addUserDebugLine([r[0]+10*uf_ny[0],r[1]+10*uf_ny[1],r[2]+10*uf_ny[2]],[r[0]-10*uf_ny[0],r[1]-10*uf_ny[1],r[2]-10*uf_ny[2]],[0,1,0],1,dt*10,replaceItemUniqueId=indy7.y_lineId)
	p.addUserDebugLine([r[0]+10*uf_nz[0],r[1]+10*uf_nz[1],r[2]+10*uf_nz[2]],[r[0]-10*uf_nz[0],r[1]-10*uf_nz[1],r[2]-10*uf_nz[2]],[0,0,1],1,dt*10,replaceItemUniqueId=indy7.z_lineId)	

	#p.addUserDebugLine([r[0],r[1],r[2]],[r[0]-f_nx[0],r[1]-f_nx[1],r[2]-f_nx[2]],[1,0,0],5,dt*50,replaceItemUniqueId=pos_x_lineId)
	#p.addUserDebugLine([r[0],r[1],r[2]],[r[0]-f_ny[0],r[1]-f_ny[1],r[2]-f_ny[2]],[0,1,0],5,dt*50,replaceItemUniqueId=pos_y_lineId)
	#p.addUserDebugLine([r[0],r[1],r[2]],[r[0]-f_nz[0],r[1]-f_nz[1],r[2]-f_nz[2]],[0,0,1],5,dt*50,replaceItemUniqueId=pos_z_lineId)
	q,qdot = indy7.getJointStates();
	Js,Jb,Ja,pinvJs,pinvJb,pinvJa = indy7.getJacobian(M,Slist,Blist,q)
	tauGrav = GravityForces(q, g, Mlist, Glist, Slist);
	MassMat = MassMatrix(q, Mlist, Glist, Slist)
	tauExt = Jb.T@Wrench/100.0
	torques =tauGrav+tauExt;
	indy7.setTorques(tauGrav+tauExt)
	indy7.step();
	


