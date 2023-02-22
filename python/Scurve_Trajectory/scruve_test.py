from pyScurveGenerator import *
from math import *
from draw_functions import *

vo = 2
vf = 1
ao =20
af = 10
so = 0
sf = 1
j =5000
amax = 100
dmax = 100
vmax = 20
dt = 0.001
traj = Trajectory()
traj.vo = vo
traj.vf = vf
traj.ao = ao
traj.af = af
traj.so = so
traj.sf = sf
traj.j = j
    
traj.amax = amax
traj.dmax = dmax
traj.vmax = vmax
traj.a1 = traj.amax
traj.a2 = traj.dmax
traj.vp = traj.vmax

initial_traj = traj;

sg =ScurveGenerator([traj])
traj = sg.getTraj(0)
sg.U2DminTimeTraj(traj,traj);
U2DminTime = traj.tt;
drawTraj(sg,traj,1000)