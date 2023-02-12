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
        self.p.resetDebugVisualizerCamera(1.0,90,-15,[0,0,0.5])
        self.p.resetBasePositionAndOrientation(self.robotId, baseXYZ, self.p.getQuaternionFromEuler(baseRPY))

        self.active_joint_list,self.active_joint_num_list,self.eef_num=self.getActiveJointList(self.robotId)
        self.p.enableJointForceTorqueSensor(self.robotId,self.eef_num,True)
        ## initialize all joint
        self.initializeActiveJoints(self.robotId,self.active_joint_num_list)

        self.data={}
        self.data["t"]=[]
        self.data["q"]=[]
        self.data["T"]=[]
        self.data["Xd"]=[]
        self.data["Vb"]=[]
        self.data["Vd"]=[]
        self.data["dVb"]=[]
        self.data["dVd"]=[]
        
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
        self.line_x_Id_list = []                                          
        self.line_y_Id_list = []                                          
        self.line_z_Id_list = []                                          

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
    def setData(self,t,q,T,Xd,Vb,Vd,dVb,dVd):
        self.data["t"].append(t)
        self.data["q"].append(q.tolist())
        self.data["T"].append(T.tolist())
        self.data["Xd"].append(Xd.tolist())
        self.data["Vb"].append(Vb.tolist())
        self.data["Vd"].append(Vd.tolist())
        self.data["dVb"].append(dVb.tolist())
        self.data["dVd"].append(dVd.tolist())
    def saveData(self):
        with open("output.json", 'w') as outfile:
            json.dump(self.data, outfile)
    def plotData(self):
        data={}
        with open("output.json", 'r') as file:
            data = json.load(file)
        t_list = np.array(data["t"])
        q_list = np.array(data["q"])

        T_list = np.array(data["T"])
        T_x_list = T_list[:,0,3]
        T_y_list = T_list[:,1,3]
        T_z_list = T_list[:,2,3]

        Xd_list = np.array(data["Xd"])
        Xd_x_list = Xd_list[:,0,3]
        Xd_y_list = Xd_list[:,1,3]
        Xd_z_list = Xd_list[:,2,3]

        Vd_list = np.array(data["Vd"])
        Vb_list = np.array(data["Vb"])

        dVb_list = np.array(data["dVb"])
        dVd_list = np.array(data["dVd"])


        #for i in range(0,12):
        #   plt.plot(t_list,q_rel_list[:,i])
        plt.figure()
        plt.subplot(3,1,1)
        plt.plot(t_list,T_x_list-Xd_x_list)
        plt.subplot(3,1,2)
        plt.plot(t_list,T_y_list-Xd_y_list)
        plt.subplot(3,1,3)
        plt.plot(t_list,T_z_list-Xd_z_list)
        plt.tight_layout()
        plt.figure()
        ax=plt.subplot(2,3,1)
        plt.plot(t_list,Vd_list[:,0],'r--')
        plt.plot(t_list,Vb_list[:,0],'b:')
        ax.set_title(r'$\omega_1$',fontsize='medium', usetex = True)
        ax=plt.subplot(2,3,2)
        plt.plot(t_list,Vd_list[:,1],'r--')
        plt.plot(t_list,Vb_list[:,1],'b:')
        ax.set_title(r'$\omega_2$',fontsize='medium', usetex = True)
        ax=plt.subplot(2,3,3)
        plt.plot(t_list,Vd_list[:,2],'r--')
        plt.plot(t_list,Vb_list[:,2],'b:')
        ax.set_title(r'$\omega_3$',fontsize='medium', usetex = True)
        ax=plt.subplot(2,3,4)
        plt.plot(t_list,Vd_list[:,3],'r--')
        plt.plot(t_list,Vb_list[:,3],'b:')
        ax.set_title(r'$\dot{p}_x$',fontsize='medium', usetex = True)
        ax=plt.subplot(2,3,5)
        plt.plot(t_list,Vd_list[:,4],'r--')
        plt.plot(t_list,Vb_list[:,4],'b:')
        ax.set_title(r'$\dot{p}_y$',fontsize='medium', usetex = True)
        ax=plt.subplot(2,3,6)
        plt.plot(t_list,Vd_list[:,5],'r--')
        plt.plot(t_list,Vb_list[:,5],'b:')
        ax.set_title(r'$\dot{p}_z$',fontsize='medium', usetex = True)
        plt.tight_layout()        
        plt.figure()
        ax=plt.subplot(2,3,1)
        plt.plot(t_list,dVd_list[:,0],'r--')
        plt.plot(t_list,dVb_list[:,0],'b:')
        ax.set_title(r'$\dot{\omega}_x$',fontsize='medium', usetex = True)
        ax=plt.subplot(2,3,2)
        plt.plot(t_list,dVd_list[:,1],'r--')
        plt.plot(t_list,dVb_list[:,1],'b:')
        ax.set_title(r'$\dot{\omega}_y$',fontsize='medium', usetex = True)
        ax=plt.subplot(2,3,3)
        plt.plot(t_list,dVd_list[:,2],'r--')
        plt.plot(t_list,dVb_list[:,2],'b:')
        ax.set_title(r'$\dot{\omega}_z$',fontsize='medium', usetex = True)
        ax=plt.subplot(2,3,4)
        plt.plot(t_list,dVd_list[:,3],'r--')
        plt.plot(t_list,dVb_list[:,3],'b:')
        ax.set_title(r'$\ddot{p}_x$',fontsize='medium', usetex = True)
        ax=plt.subplot(2,3,5)
        plt.plot(t_list,dVd_list[:,4],'r--')
        plt.plot(t_list,dVb_list[:,4],'b:')
        ax.set_title(r'$\ddot{p}_y$',fontsize='medium', usetex = True)
        ax=plt.subplot(2,3,6)
        plt.plot(t_list,dVd_list[:,5],'r--')
        plt.plot(t_list,dVb_list[:,5],'b:')
        ax.set_title(r'$\ddot{p}_z$',fontsize='medium', usetex = True)
        plt.tight_layout()        
        plt.show()    
    def drawT(self,T,lineWidth,x_lineId,y_lineId,z_lineId,scale):
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
                               lineColorRGB=[1/scale,0,0],
                               lineWidth=lineWidth,
                               lifeTime=0,
                               replaceItemUniqueId=x_lineId)
        p.addUserDebugLine(lineFromXYZ=line_p,
                               lineToXYZ=line_py,
                               lineColorRGB=[0,1/scale,0],
                               lineWidth=lineWidth,
                               lifeTime=0,
                               replaceItemUniqueId=y_lineId)
        p.addUserDebugLine(lineFromXYZ=line_p,
                               lineToXYZ=line_pz,
                               lineColorRGB=[0,0,1/scale],
                               lineWidth=lineWidth,
                               lifeTime=0,
                               replaceItemUniqueId=z_lineId)
    def drawEEF(self):
        T = self.getEEFPose();
        self.drawT(T,self.lineWidth,self.x_lineId,self.y_lineId,self.z_lineId,1);
    def drawTrajectory(self,Xd_list,Num):
        for idx in range(0,len(Xd_list),int(len(Xd_list)/Num)):
            T = Xd_list[idx];
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
                                   lineWidth=1,
                                   lifeTime=0)
            p.addUserDebugLine(lineFromXYZ=line_p,
                                   lineToXYZ=line_py,
                                   lineColorRGB=[0,1,0],
                                   lineWidth=1,
                                   lifeTime=0)
            p.addUserDebugLine(lineFromXYZ=line_p,
                                   lineToXYZ=line_pz,
                                   lineColorRGB=[0,0,1],
                                   lineWidth=1,
                                   lifeTime=0)
    def setTorques(self,torques):
        for i in range(len(self.active_joint_num_list)):
            self.p.setJointMotorControl2(self.robotId, self.active_joint_num_list[i], self.p.TORQUE_CONTROL ,force=torques[i])  

