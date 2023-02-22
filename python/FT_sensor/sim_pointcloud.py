import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import pybullet as p
import time
from math import *
from modern_robotics import *
import scipy.io
import numpy as np
np.set_printoptions(precision=5,suppress=True)

mat = scipy.io.loadmat('cylinder.mat')
Cylinder_X = mat['X']
Cylinder_Y = mat['Y']
Cylinder_Z = mat['Z']
Cylinder_Nx = mat['Nx']
Cylinder_Ny = mat['Ny']
Cylinder_Nz = mat['Nz']


pi = np.pi

CONTROL_FREQ = 240.0
dt = 1.0/CONTROL_FREQ
endTime = 50
force_scaling_factor = 0.05
eul_x = 0. 
eul_y = 0.0
eul_z = 0.0


p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER,0)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS,1)
p.resetDebugVisualizerCamera(1.0,90,0,[0,0,0.0])
p.setTimeStep(dt)
p.setPhysicsEngineParameter(dt)
p.setGravity(0, 0, 0)

quat = p.getQuaternionFromEuler([eul_x,eul_y,eul_z])
d = -0.25
R = p.getMatrixFromQuaternion(quat)
R = np.reshape(R,[3,3])
n = R@np.array([0,0,1]).T
nx = R@np.array([1,0,0]).T
ny = R@np.array([0,1,0]).T
nz = R@np.array([0,0,1]).T
#boxId = p.loadURDF("box.urdf",[0.0, 0, 0.0],quat,useFixedBase=True)
#boxId = p.loadURDF("sphere.urdf",[0.0, 0, 0.0],quat,useFixedBase=True)
#boxId = p.loadURDF("monkey.urdf",[0.0, 0, 0.0],quat,useFixedBase=True)
boxId = p.loadURDF("cylinder.urdf",[0.0, 0, 0.0],quat,useFixedBase=True)
box2Id = p.loadURDF("object.urdf",[0.0, 0, 2],quat)
planeId = p.loadURDF("plane.urdf",[0.0, 0, 0.0])
p.enableJointForceTorqueSensor(boxId,0,True)


x_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0],
                            lineToXYZ=[0, 0, 10],
                            lineColorRGB=[1,0,0],
                            lineWidth=3,
                            lifeTime=0)
y_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0],
                            lineToXYZ=[0, 0, 0],
                            lineColorRGB=[0,1,0],
                            lineWidth=3,
                            lifeTime=0)
z_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0],
                            lineToXYZ=[0, 0, 0],
                            lineColorRGB=[0,1,0],
                            lineWidth=3,
                            lifeTime=0) 
pa_x_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0],
                            lineToXYZ=[0, 0, 10],
                            lineColorRGB=[1,0,0],
                            lineWidth=3,
                            lifeTime=0)
pa_y_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0],
                            lineToXYZ=[0, 0, 0],
                            lineColorRGB=[0,1,0],
                            lineWidth=3,
                            lifeTime=0)
pa_z_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0],
                            lineToXYZ=[0, 0, 0],
                            lineColorRGB=[0,1,0],
                            lineWidth=3,
                            lifeTime=0) 
lineId_list = []
distId_list=[]
for j in range(0,100):
	lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0],
                            lineToXYZ=[0, 0, 0],
                            lineColorRGB=[0,0,0],
                            lineWidth=3,
                            lifeTime=0)
	distId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0],
                            lineToXYZ=[0, 0, 0],
                            lineColorRGB=[0,0,0],
                            lineWidth=3,
                            lifeTime=0)
	lineId_list.append(lineId)
	distId_list.append(distId)



p.stepSimulation()
jointState = np.array(p.getJointState(boxId,0)[2]).T
initial_Wrench= np.array([jointState[3],jointState[4],jointState[5],jointState[0],jointState[1],jointState[2]]).T  
p.stepSimulation()
jointState = np.array(p.getJointState(boxId,0)[2]).T
initial_Wrench= np.array([jointState[3],jointState[4],jointState[5],jointState[0],jointState[1],jointState[2]]).T  
p.stepSimulation()
jointState = np.array(p.getJointState(boxId,0)[2]).T
initial_Wrench= np.array([jointState[3],jointState[4],jointState[5],jointState[0],jointState[1],jointState[2]]).T  
r = [0,0,0]
test_idx=0
isStart = 0;
rand_count = 0;
rand_idx = int(np.random.rand()*len(Cylinder_X))
fa =[np.random.rand()*10-5,np.random.rand()*10-5,np.random.rand()*10-5]
for t in np.arange(0,endTime,dt):

	if rand_count>100:
		fa = [-Cylinder_Nx[rand_idx],-Cylinder_Ny[rand_idx],-Cylinder_Nz[rand_idx]]
		rand_idx = int(np.random.rand()*len(Cylinder_X))
		rand_count = 0;
	rand_count = rand_count+1
	
	pa = [Cylinder_X[rand_idx],Cylinder_Y[rand_idx],Cylinder_Z[rand_idx]]
	p.applyExternalForce(boxId,1,fa,pa,p.LINK_FRAME)
	jointState = np.array(p.getJointState(boxId,0)[2]).T
	Wrench= np.array([jointState[3],jointState[4],jointState[5],jointState[0],jointState[1],jointState[2]]).T -initial_Wrench
	Wrench = Adjoint(RpToTrans(R,np.array([0,0,0]).T))@Wrench
	#print(Wrench)
	f = np.array([Wrench[3],Wrench[4],Wrench[5]])
	m = np.array([Wrench[0],Wrench[1],Wrench[2]]).T
	norm_f= np.linalg.norm(f)
	if norm_f <=0.000000001:
		p.stepSimulation(); 
		time.sleep(dt)
		continue;

	
	u = f/norm_f;
	f_nx = np.dot(u,nx)*nx*norm_f*force_scaling_factor
	f_ny = np.dot(u,ny)*ny*norm_f*force_scaling_factor
	f_nz = np.dot(u,nz)*nz*norm_f*force_scaling_factor
	dist_m=np.ones([len(Cylinder_X)])
	d_list=np.ones([len(Cylinder_X)])
	U= np.array([[0 ,-u[2] ,u[1]],[u[2] ,0 ,-u[0]],[-u[1], u[0], 0]])
	cylinder_r = np.array([Cylinder_X ,Cylinder_Y ,Cylinder_Z ])
	cylinder_r = np.reshape(cylinder_r,(3,-1))
	cylinder_n = np.array([Cylinder_Nx ,Cylinder_Ny ,Cylinder_Nz ])
	cylinder_n = np.reshape(cylinder_n,(3,-1))
	est_m = -U@cylinder_r*norm_f
	repmat_m = np.tile(np.array([m[0],m[1],m[2]]), [cylinder_r.shape[1],1]).T
	repmat_u = np.tile(np.array([u[0],u[1],u[2]]), [cylinder_r.shape[1],1]).T
	
	dist_m = np.linalg.norm(est_m-repmat_m,axis=0)
	dist_d = np.linalg.norm(cylinder_n-repmat_u,axis=0)



	#print(dist_d.shape)
	#time.sleep(10)
	angle_m = np.arccos(dist_d/np.linalg.norm(est_m,axis=0)/np.linalg.norm(repmat_m,axis=0))
	pi_angle_m=np.arccos(dist_d/np.linalg.norm(est_m,axis=0)/np.linalg.norm(repmat_m,axis=0))-pi
	angle_m = np.minimum(angle_m, pi_angle_m)

	try:
		r = np.linalg.inv(U.T @ U)@U.T @np.array([-Wrench[0]/norm_f,-Wrench[1]/norm_f,-Wrench[2]/norm_f,-d ])
	except:
		r = [0,0,0]
	min_idx=np.argsort(dist_m)[1:100]

	r = np.round(np.array([Cylinder_X[min_idx[0]][0],Cylinder_Y[min_idx[0]][0],Cylinder_Z[min_idx[0]][0]]),5)

	uf_nx = f_nx/np.linalg.norm(f_nx)
	uf_ny = f_ny/np.linalg.norm(f_ny)
	uf_nz = f_nz/np.linalg.norm(f_nz)
	p.addUserDebugLine([r[0],r[1],r[2]],[r[0]-f_nx[0],r[1]-f_nx[1],r[2]-f_nx[2]],[1,0,0],3,0,replaceItemUniqueId=x_lineId)	
	p.addUserDebugLine([r[0],r[1],r[2]],[r[0]-f_ny[0],r[1]-f_ny[1],r[2]-f_ny[2]],[0,1,0],3,0,replaceItemUniqueId=y_lineId)
	p.addUserDebugLine([r[0],r[1],r[2]],[r[0]-f_nz[0],r[1]-f_nz[1],r[2]-f_nz[2]],[0,0,1],3,0,replaceItemUniqueId=z_lineId)
	for j in range(0,len(min_idx)):
		rj = np.round(np.array([Cylinder_X[min_idx[j]][0],Cylinder_Y[min_idx[j]][0],Cylinder_Z[min_idx[j]][0]]),5)
		rn = np.round(np.array([Cylinder_Nx[min_idx[j]][0],Cylinder_Ny[min_idx[j]][0],Cylinder_Nz[min_idx[j]][0]]),5)
		dist = np.dot(rj,rn)
		
		p.addUserDebugLine([rj[0],rj[1],rj[2]],[rj[0]+0.01,rj[1]+0.01,rj[2]+0.01],[1*(100-j)/100,0,1*(100-j)/100],1,0,replaceItemUniqueId=lineId_list[j])	
		#p.addUserDebugLine([0,0,0],[rn[0]*dist,rn[1]*dist,rn[2] *dist],replaceItemUniqueId=distId_list[j])	
	
	
	

	ua = np.array(fa)/np.linalg.norm(fa);
	p.addUserDebugLine([pa[0],pa[1],pa[2]],[pa[0]+ua[0],pa[1]+ua[1],pa[2]+ua[2]],[0,0,0],1,0,replaceItemUniqueId=pa_x_lineId)	
	p.addUserDebugLine([pa[0],pa[1],pa[2]],[pa[0]+ua[0]/100,pa[1]+ua[1]/100,pa[2]+ua[2]/100],[0,0,0],5,0,replaceItemUniqueId=pa_y_lineId)	
	p.addUserDebugLine([pa[0],pa[1],pa[2]],[pa[0]+ua[0]/100,pa[1]+ua[1]/100,pa[2]+ua[2]/100],[0,0,0],5,0,replaceItemUniqueId=pa_z_lineId)	

	ua = np.array([ua[0],ua[1],ua[2]]).T
	norm_fa=np.linalg.norm(fa)
	fa_nx = np.dot(ua,nx)*nx*norm_fa*force_scaling_factor
	fa_ny = np.dot(ua,ny)*ny*norm_fa*force_scaling_factor
	fa_nz = np.dot(ua,nz)*nz*norm_fa*force_scaling_factor

	p.stepSimulation(); 
	time.sleep(dt)


