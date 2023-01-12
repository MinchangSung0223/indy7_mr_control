import pybullet as p
import numpy as np
np.set_printoptions(precision=4, suppress=True)
from numpy.linalg import pinv
import matplotlib.pyplot as plt
from modern_robotics_smc import *
import matplotlib.pyplot as plt
from math import *
import json
import pybullet_data
D2R = np.pi/180.0

class Indy7:
    """
    Indy7 class
    """
    def __init__(self,p,urdf_name = "../model/indy7.urdf", CONTROL_FREQ = 240.0 , isconnect = False,baseXYZ = [0 ,0,0],baseRPY=[0 ,0,0]):
        self.MAX_TORQUES = np.array([431.97,431.97,197.23,79.79,79.79,79.79]) # unit [N]
        self.HomePos = np.array([0,0,-pi/2.0,0,-pi/2.0,0]).T
        self.p = p
        if isconnect==False:
            self.p.connect(self.p.GUI)
        self.p.setGravity(0, 0, -9.8)
        self.p.setRealTimeSimulation(0)
        self.timeStep = 1/CONTROL_FREQ;
        self.p.setTimeStep(self.timeStep)
        #self.p.setPhysicsEngineParameter(fixedTimeStep = self.timeStep)
        self.p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robotId = self.p.loadURDF(urdf_name, [0, 0, 0.0],[0, 0, 0, 1],flags=self.p.URDF_USE_SELF_COLLISION,useFixedBase=True)
        self.p.configureDebugVisualizer(self.p.COV_ENABLE_GUI,0)
        self.p.resetDebugVisualizerCamera(2,90,-15,[0,0,0.5])
        self.p.resetBasePositionAndOrientation(self.robotId, baseXYZ, self.p.getQuaternionFromEuler(baseRPY))

        self.active_joint_list,self.active_joint_num_list,self.eef_num=self.getActiveJointList(self.robotId)
        self.p.enableJointForceTorqueSensor(self.robotId,self.eef_num,True)
        ## initialize all joint
        self.initializeActiveJoints(self.robotId,self.active_joint_num_list)

        self.data={}
        self.data["t"]=[]
        self.data["q_rel"]=[]
        self.data["T_rel"]=[]
        self.data["Xd"]=[]
        self.lineWidth = 4;
        self.x_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0],
                                    lineToXYZ=[0, 0, 0],
                                    lineColorRGB=[1,0,0],
                                    lineWidth=self.lineWidth,
                                    lifeTime=0)
        self.y_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0],
                                    lineToXYZ=[0, 0, 0],
                                    lineColorRGB=[0,1,0],
                                    lineWidth=self.lineWidth,
                                    lifeTime=0)
        self.z_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0],
                                    lineToXYZ=[0, 0, 0],
                                    lineColorRGB=[0,1,0],
                                    lineWidth=self.lineWidth,
                                    lifeTime=0)                                            

        dt = 1.0/CONTROL_FREQ

    def getActiveJointList(self,robotId):
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
        print(active_joint_list)
        return  active_joint_list,active_joint_num_list,eef_num        
    def initializeActiveJoints(sel,robotId,active_joints_num):
        for i in active_joints_num:
            p.setJointMotorControl2(robotId, i, p.VELOCITY_CONTROL, targetVelocity=0, force=0)              
    def MRSetup(sel,file_path):
        ##MRdata load
        data={}
        with open(file_path, 'r') as file:
            data = json.load(file)
        relSlist = np.array(data['relSlist'])
        relBlist = np.array(data['relBlist'])
        relM = np.array(data['relM'])
        baseSlist = np.array(data['baseSlist'])
        baseM = np.array(data['baseM'])
        baseBlist = np.array(data['baseBlist'])
        Tbr = np.array(data['Tbr'])
        Tbl = np.array(data['Tbl'])
        Mlist = np.array(data['Mlist'])
        Glist = np.array(data['Glist'])
        return baseSlist,baseBlist,baseM,relSlist,relBlist,relM,Tbr,Tbl,Mlist,Glist   
    def step(self):
        self.p.stepSimulation();         
    def resetJointStates(self,q):
        for i in range(0,len(q)):
            self.p.resetJointState(self.robotId,self.active_joint_num_list[i],q[i])

    def setJointStates(self,q):
        for i in range(0,len(q)):
            self.p.setJointMotorControl2(self.robotId, self.active_joint_num_list[i], self.p.POSITION_CONTROL,targetPosition=q[i], force=self.MAX_TORQUES[i]) 
    def getJointStates(self):
        jointState = p.getJointStates(self.robotId,self.active_joint_num_list)
        q=[]
        qdot=[]
        for i in range(0,len(self.active_joint_num_list)):
            js = jointState[i];
            q.append(js[0])
            qdot.append(js[1])
        q = np.array(q)
        qdot = np.array(qdot)
        return q,qdot
    def getJacobian(self,M,Slist,Blist,thetalist):
        thetalist = np.reshape(thetalist,(len(thetalist),1))
        Js = JacobianSpace(Slist,thetalist);
        Jb = JacobianBody(Blist,thetalist);
        Ja = AnalyticJacobianBody(M,Blist, thetalist)
        return Js,Jb,Ja,pinv(Js),pinv(Jb) , pinv(Ja) 
    def getEEFPose(self):
        Teef = np.eye(4);
        ret = p.getLinkState(self.robotId,self.eef_num,1,1)
        pos = ret[0]
        orn = ret[1]    
        Teef[0:3,3] = np.array(pos).T
        Teef[0:3,0:3] = np.reshape(p.getMatrixFromQuaternion(orn),(3,3))
        return Teef 
    def getActuatedTorques(self):
        actuated_torques=[]
        for i in range(len(self.active_joint_num_list)):
            tempState = p.getJointState(self.robotId,self.active_joint_num_list[i])
            actuated_torques.append(tempState[3])
        return np.array(actuated_torques)      
    def getFT(self):
        val = np.array(self.p.getJointState(self.robotId,self.eef_num)[2]).T
        norm_val =sqrt(val[3]*val[3]+val[4]*val[4]+val[5]*val[5])
        return np.array([val[3],val[4],val[5],val[0],val[1],val[2]]).T          
    def applyForce(self,force):
        #self.p.applyExternalForce(self.robotId,self.eef_num,force,[0,0,0],self.p.WORLD_FRAME)
        self.p.applyExternalForce(self.robotId,self.eef_num,force,[0,0,0],self.p.LINK_FRAME)
    def drawEEF(self):
        T = self.getEEFPose();    
        line_p = T[0:3,3];
        line_px = T@np.array([0.1,0,0,1]).T
        line_py = T@np.array([0.0,0.1,0,1]).T
        line_pz = T@np.array([0.0,0,0.1,1]).T
        line_p = [line_p[0],line_p[1],line_p[2]]
        line_px = [line_px[0],line_px[1],line_px[2]]
        line_py = [line_py[0],line_py[1],line_py[2]]
        line_pz = [line_pz[0],line_pz[1],line_pz[2]]
        p.addUserDebugLine(lineFromXYZ=line_p,
                               lineToXYZ=line_px,
                               lineColorRGB=[1,0,0],
                               lineWidth=self.lineWidth,
                               lifeTime=0,
                               replaceItemUniqueId=self.x_lineId)
        p.addUserDebugLine(lineFromXYZ=line_p,
                               lineToXYZ=line_py,
                               lineColorRGB=[0,1,0],
                               lineWidth=self.lineWidth,
                               lifeTime=0,
                               replaceItemUniqueId=self.y_lineId)
        p.addUserDebugLine(lineFromXYZ=line_p,
                               lineToXYZ=line_pz,
                               lineColorRGB=[0,0,1],
                               lineWidth=self.lineWidth,
                               lifeTime=0,
                               replaceItemUniqueId=self.z_lineId)
    def setTorques(self,torques):
        for i in range(len(self.active_joint_num_list)):
            self.p.setJointMotorControl2(self.robotId, self.active_joint_num_list[i], self.p.TORQUE_CONTROL ,force=torques[i])  

