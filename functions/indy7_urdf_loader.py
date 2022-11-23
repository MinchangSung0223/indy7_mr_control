import numpy as np
import pandas as pd
import json

np.set_printoptions(precision=6, suppress=True)
from mr_urdf_loader import loadURDF
from modern_robotics import *
urdf_name = "../model/indy7.urdf"
MR=loadURDF(urdf_name)
M  = np.array(MR["M"])
Slist  = np.array(MR["Slist"])
Mlist  = np.array(MR["Mlist"])
Glist  = np.array(MR["Glist"])
Blist  = np.array(MR["Blist"])
actuated_joints_num = MR["actuated_joints_num"]

thetalist = np.array([0,0,np.pi/2.0])
dthetalist = np.array([0,0,0.1])
ddthetalist = np.array([0,0,0])
g = np.array([0,0,-9.8])
Ftip = [0,0,0,0,0,0]

print("======M======")
print(M)
print("======Slist======")
print(Slist)
print("======Mlist======")
print(Mlist)
print("======Glist======")
print(Glist)
print("======Blist======")
print(Blist)
print("FKinSpace\n", FKinSpace(M,Slist,thetalist))
print("FKinBody\n", FKinBody(M,Blist,thetalist))

print("JacobianSpace\n", JacobianSpace(Slist,thetalist))
print("JacobianBody\n", JacobianBody(Blist,thetalist))

print("InverseDynamics\n" ,InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist,  Glist, Slist))

data = {}
data['Slist'] = Slist.tolist();
data['Blist'] = Blist.tolist();
data['Mlist'] = Mlist.tolist();
data['Glist'] = Glist.tolist();
data['M'] = M.tolist();

print(data)
file_path = "./MR_info.json"
with open(file_path, 'w') as outfile:
    json.dump(data, outfile)
    


