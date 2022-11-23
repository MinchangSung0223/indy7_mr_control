import pybullet as p
import numpy as np
import matplotlib.pyplot as plt


	
def getJointStates(robotId,active_joints_num):
	jointState = np.array(p.getJointStates(robotId,active_joints_num))
	q = np.array(jointState[:,0])
	qdot = np.array(jointState[:,1])
	return q,qdot
def getEEFPose(robotId,eef_num):
	Teef = np.eye(4);
	ret = p.getLinkState(robotId,eef_num,1,1)
	pos = ret[0]
	orn = ret[1]	
	Teef[0:3,3] = np.array(pos).T
	Teef[0:3,0:3] = np.reshape(p.getMatrixFromQuaternion(orn),(3,3))
	return Teef	
def getActiveJointList(robotId):
	active_joint_list=[]
	active_joint_num_list=[]	
	numJoint = p.getNumJoints(robotId)
	print("============================JOINT_INFO====================================")
	for i in range(0,numJoint):
		info = p.getJointInfo(robotId,i)
		joint_type = info[2]
		joint_type_name =""
		if joint_type == p.JOINT_FIXED:
			joint_type_name = "JOINT_FIXED"
		elif joint_type == p.JOINT_REVOLUTE:
			joint_type_name = "JOINT_REVOLUTE"
		elif joint_type == p.JOINT_PRISMATIC:
			joint_type_name = "JOINT_PRISMATIC"
		elif joint_type == p.JOINT_SPHERICAL:
			joint_type_name = "JOINT_SPHERICAL"
		elif joint_type == p.JOINT_PLANAR:
			joint_type_name = "JOINT_PLANAR"
											
		print("Joint Num : ", info[0] , "\t Joint Name : ", info[1], "\t Joint Type : " , joint_type_name)
		if info[2] !=  p.JOINT_FIXED:
			active_joint_list.append(info[1])
			active_joint_num_list.append(info[0])

	print("=========================================================================")		
	eef_num = numJoint-1;
	return	active_joint_list,active_joint_num_list,eef_num

def initializeActiveJoints(robotId,active_joints_num):
	for i in active_joints_num:
		p.setJointMotorControl2(robotId, i, p.VELOCITY_CONTROL, targetVelocity=0, force=0)	
def setTorques(robotId, active_joints_num,torques,MAX_TORQUES):
	idx =0
	def saturation(x,max_x):
		if x>= 0 : return np.min([x,max_x])
		else : return np.max([x,-max_x])
	actuated_torques = []
	for i in active_joints_num:
		p.setJointMotorControl2(robotId, i, p.TORQUE_CONTROL, force=saturation(torques[idx],MAX_TORQUES[idx]))	
		actuated_torques.append(saturation(torques[idx],MAX_TORQUES[idx]))
		
		idx=idx+1
	#print("input torques : ", torques)
	#print("actuated_torques : ", actuated_torques)
			
