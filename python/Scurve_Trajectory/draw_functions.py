from pyScurveGenerator import *
import matplotlib.pyplot as plt
import numpy as np


def drawTraj_(sg,traj,number,fig,ax):
	t_list = np.linspace(0,traj.tt,number)
	s_list = []
	ds_list = []
	dds_list = []
	ddds_list = []
	state_list = []
	for j in range(len(t_list)):
		val = sg.generate(traj,t_list[j]);	
		s_list.append(val[0])
		ds_list.append(val[1])
		dds_list.append(val[2])
		ddds_list.append(val[3])
		state_list.append(val[4]+1)
	state_list = np.array(state_list);
	t_list = np.array(t_list);
	s_list = np.array(s_list);
	ds_list = np.array(ds_list);
	dds_list = np.array(dds_list);
	ddds_list = np.array(ddds_list);

	ax[0].plot(t_list,s_list, linestyle='--', linewidth=1)
	ax[1].plot(t_list,ds_list, linestyle='--', linewidth=1)	
	ax[2].plot(t_list,dds_list,linestyle='--', linewidth=1)	
	ax[3].plot(t_list,ddds_list,linestyle='--', linewidth=1)	
		
	
	
	print("Trajectory Time : ",t_list[-1])

def drawTraj(sg,traj,number):
	t_list = np.linspace(0,traj.tt,number)
	s_list = []
	ds_list = []
	dds_list = []
	ddds_list = []
	state_list = []
	for j in range(len(t_list)):
		val = sg.generate(traj,t_list[j]);	
		s_list.append(val[0])
		ds_list.append(val[1])
		dds_list.append(val[2])
		ddds_list.append(val[3])
		state_list.append(val[4]+1)

	fig, ax = plt.subplots(4,1)
	fig.tight_layout() 
	state_list = np.array(state_list);
	t_list = np.array(t_list);
	s_list = np.array(s_list);
	ds_list = np.array(ds_list);
	dds_list = np.array(dds_list);
	ddds_list = np.array(ddds_list);
	color_list = np.array([[1,0,0],[1,0.5,0],[0,0,1],[0,1,0],[0,0,1],[1,0.5,0],[1,0,0]])
	t0 = 0
	try:
		t1 = t_list[state_list==1][-1]
	except:
		t1 = t0
	try:
		t2 = t_list[state_list==2][-1]
	except:
		t2 = t1
	try:
		t3 = t_list[state_list==3][-1]
	except:
		t3 = t2
	try:
		t4 = t_list[state_list==4][-1]
	except:
		t4 = t3		
	try:		
		t5 = t_list[state_list==5][-1]
	except:
		t5 = t4		
	try:
		t6 = t_list[state_list==6][-1]
	except:
		t6 = t5		
	try:
		t7 = t_list[state_list==7][-1]
	except:
		t7 = t6		

	for i in range(1,8):
		ax[0].plot(t_list[state_list==i],s_list[state_list==i],color= color_list[i-1,:])
		ax[0].set_title("s")
		ax[0].axvline(x=t1, color='k', linestyle=':', linewidth=0.1)
		ax[0].axvline(x=t2, color='k', linestyle=':', linewidth=0.1)
		ax[0].axvline(x=t3, color='k', linestyle=':', linewidth=0.1)
		ax[0].axvline(x=t4, color='k', linestyle=':', linewidth=0.1)
		ax[0].axvline(x=t5, color='k', linestyle=':', linewidth=0.1)
		ax[0].axvline(x=t6, color='k', linestyle=':', linewidth=0.1)	
		ax[0].axvline(x=t7, color='k', linestyle=':', linewidth=0.1)		
		ax[0].set_xticks([t0,t1,t2,t3,t4,t5,t6,t7])
		ax[1].plot(t_list[state_list==i],ds_list[state_list==i],color= color_list[i-1,:])
		ax[1].set_title("ds")		
		ax[1].axvline(x=t1, color='k', linestyle=':', linewidth=0.1)
		ax[1].axvline(x=t2, color='k', linestyle=':', linewidth=0.1)
		ax[1].axvline(x=t3, color='k', linestyle=':', linewidth=0.1)
		ax[1].axvline(x=t4, color='k', linestyle=':', linewidth=0.1)
		ax[1].axvline(x=t5, color='k', linestyle=':', linewidth=0.1)
		ax[1].axvline(x=t6, color='k', linestyle=':', linewidth=0.1)		
		ax[1].axvline(x=t7, color='k', linestyle=':', linewidth=0.1)		
		ax[1].set_xticks([t0,t1,t2,t3,t4,t5,t6,t7])

		ax[2].plot(t_list[state_list==i],dds_list[state_list==i],color= color_list[i-1,:])
		ax[2].set_title("dds")

		ax[2].axvline(x=t1, color='k', linestyle=':', linewidth=0.1)
		ax[2].axvline(x=t2, color='k', linestyle=':', linewidth=0.1)
		ax[2].axvline(x=t3, color='k', linestyle=':', linewidth=0.1)
		ax[2].axvline(x=t4, color='k', linestyle=':', linewidth=0.1)
		ax[2].axvline(x=t5, color='k', linestyle=':', linewidth=0.1)
		ax[2].axvline(x=t6, color='k', linestyle=':', linewidth=0.1)				
		ax[2].axvline(x=t7, color='k', linestyle=':', linewidth=0.1)		
		ax[2].set_xticks([t0,t1,t2,t3,t4,t5,t6,t7])

		ax[3].plot(t_list[state_list==i],ddds_list[state_list==i],color= color_list[i-1,:])
		ax[3].set_title("ddds")

		ax[3].axvline(x=t1, color='k', linestyle=':', linewidth=0.1)
		ax[3].axvline(x=t2, color='k', linestyle=':', linewidth=0.1)
		ax[3].axvline(x=t3, color='k', linestyle=':', linewidth=0.1)
		ax[3].axvline(x=t4, color='k', linestyle=':', linewidth=0.1)
		ax[3].axvline(x=t5, color='k', linestyle=':', linewidth=0.1)
		ax[3].axvline(x=t6, color='k', linestyle=':', linewidth=0.1)				
		ax[3].axvline(x=t7, color='k', linestyle=':', linewidth=0.1)	
		ax[3].set_xticks([t0,t1,t2,t3,t4,t5,t6,t7])	
	print("Trajectory Time : ",t_list[-1])
	plt.show()
def drawTrajList(sg,traj_list,number):
	fig, ax = plt.subplots(4,1)
	fig.tight_layout() 	
	for i in range(len(traj_list)):
		drawTraj_(sg, traj_list[i],number,fig,ax)
	plt.show()
def drawScurveGenerator(sg,dt):
	traj = sg.getTraj(0)
	taxis = np.linspace(0,traj.tt,int(traj.tt/dt))
	qaxis = []
	dqaxis = []
	ddqaxis = []
	dddqaxis = []
	state_list = []
	for t in taxis:
		val = sg.generate(traj,t);	
		qaxis.append(val[0])
		dqaxis.append(val[1])
		ddqaxis.append(val[2])
		dddqaxis.append(val[3])
		state_list.append(val[4]+1)
	state_list = np.array(state_list);		
	t0 = 0
	try:
		t1 = taxis[state_list==1][-1]
	except:
		t1 = t0
	try:
		t2 = taxis[state_list==2][-1]
	except:
		t2 = t1
	try:
		t3 = taxis[state_list==3][-1]
	except:
		t3 = t2
	try:
		t4 = taxis[state_list==4][-1]
	except:
		t4 = t3		
	try:		
		t5 = taxis[state_list==5][-1]
	except:
		t5 = t4		
	try:
		t6 = taxis[state_list==6][-1]
	except:
		t6 = t5		
	try:
		t7 = taxis[state_list==7][-1]
	except:
		t7 = t6		
	t_list = [t0,t1,t2,t3,t4,t5,t6,t7]


	fig, ax = plt.subplots(4,1);
	ax[0].plot(taxis,qaxis,'r');
	endTime = traj.tt;
	ax[0].axvline(x=endTime, color='black', linestyle='--', linewidth=1.1)
	ax[0].set_xticks(t_list, minor=False)
	ax[0].set_yticks([traj.so,traj.sf ], minor=False)
	ax[0].set_title("q")
	ax[0].grid(True)
	
	ax[1].plot(taxis,dqaxis,'g');
	ax[1].axvline(x=endTime, color='black', linestyle='--', linewidth=1.1)
	ax[1].set_xticks(t_list, minor=False)
	ax[1].set_yticks([np.min(dqaxis) ,np.max(dqaxis) ], minor=False)	
	ax[1].set_title(r'$\dot{q}$')	
	ax[1].grid(True)
	
	ax[2].plot(taxis,ddqaxis,'b');
	ax[2].axvline(x=endTime, color='black', linestyle='--', linewidth=1.1)	
	ax[2].set_xticks(t_list, minor=False)
	ax[2].set_yticks([np.min(ddqaxis) ,np.max(ddqaxis) ], minor=False)	
	ax[2].set_title(r'$\ddot{q}$')	
	ax[2].grid(True)
	
	ax[3].plot(taxis,dddqaxis,'k');			
	ax[3].axvline(x=endTime, color='black', linestyle='--', linewidth=1.1)
	ax[3].set_xticks(t_list, minor=False)
	ax[3].set_yticks([-traj.j ,traj.j ], minor=False)	
	ax[3].set_title(r'$\dddot{q}$')	
	ax[3].grid(True)
	fig.tight_layout() 
	plt.subplots_adjust(left=0.125, bottom=0.1, right=0.9, top=0.9, wspace=0.2, hspace=0.7)	
	plt.suptitle("ScurveGenerator Trajectory")
	plt.show()
	
def drawRuckig(inp,otg,out_list):
	taxis = np.array(list(map(lambda x: x.time, out_list)))
	qaxis = np.array(list(map(lambda x: x.new_position, out_list)))
	dqaxis = np.array(list(map(lambda x: x.new_velocity, out_list)))
	ddqaxis = np.array(list(map(lambda x: x.new_acceleration, out_list)))
	dddqaxis = np.diff(ddqaxis, axis=0, prepend=ddqaxis[0, 0]) / otg.delta_time
	dddqaxis[0, :] = 0.0
	dddqaxis[-1, :] = 0.0
	ddddqaxis=np.diff(dddqaxis, axis=0, prepend=dddqaxis[0, 0]) / otg.delta_time
	ddddqaxis[abs(ddddqaxis)>1/otg.delta_time]=1
	t_list=[0]
	for j in range(len(ddddqaxis)):
		if ddddqaxis[j]==1:
			if abs(t_list[-1]-taxis[j])>2*otg.delta_time:
				t_list.append(taxis[j])



	fig, ax = plt.subplots(4,1);
	ax[0].plot(taxis,qaxis,'r');
	endTime = out_list[-1].trajectory.intermediate_durations;
	ax[0].axvline(x=endTime, color='black', linestyle='--', linewidth=1.1)
	ax[0].set_xticks(t_list, minor=False)
	ax[0].set_yticks([inp.current_position[0] ,inp.target_position[0] ], minor=False)
	ax[0].set_title("q")
	ax[0].grid(True)
	
	ax[1].plot(taxis,dqaxis,'g');
	ax[1].axvline(x=endTime, color='black', linestyle='--', linewidth=1.1)
	ax[1].set_xticks(t_list, minor=False)
	ax[1].set_yticks([np.min(dqaxis) ,np.max(dqaxis) ], minor=False)	
	ax[1].set_title(r'$\dot{q}$')	
	ax[1].grid(True)
	
	ax[2].plot(taxis,ddqaxis,'b');
	ax[2].axvline(x=endTime, color='black', linestyle='--', linewidth=1.1)	
	ax[2].set_xticks(t_list, minor=False)
	ax[2].set_yticks([np.min(ddqaxis) ,np.max(ddqaxis) ], minor=False)	
	ax[2].set_title(r'$\ddot{q}$')	
	ax[2].grid(True)
	
	ax[3].plot(taxis,dddqaxis,'k');			
	ax[3].axvline(x=endTime, color='black', linestyle='--', linewidth=1.1)
	ax[3].set_xticks(t_list, minor=False)
	ax[3].set_yticks([-inp.max_jerk[0] ,inp.max_jerk[0] ], minor=False)	
	ax[3].set_title(r'$\dddot{q}$')	
	ax[3].grid(True)
	fig.tight_layout() 
	plt.subplots_adjust(left=0.125, bottom=0.1, right=0.9, top=0.9, wspace=0.2, hspace=0.7)
	plt.suptitle("Ruckig Trajectory")
	plt.show()
def drawRuckigWithScurveGenerator(inp,otg,out_list,sg,dt):

	taxis = np.array(list(map(lambda x: x.time, out_list)))
	qaxis = np.array(list(map(lambda x: x.new_position, out_list)))
	dqaxis = np.array(list(map(lambda x: x.new_velocity, out_list)))
	ddqaxis = np.array(list(map(lambda x: x.new_acceleration, out_list)))
	dddqaxis = np.diff(ddqaxis, axis=0, prepend=ddqaxis[0, 0]) / otg.delta_time
	dddqaxis[0, :] = 0.0
	dddqaxis[-1, :] = 0.0
	ddddqaxis=np.diff(dddqaxis, axis=0, prepend=dddqaxis[0, 0]) / otg.delta_time
	ddddqaxis[abs(ddddqaxis)>1/otg.delta_time]=1
	t_list=[0]
	for j in range(len(ddddqaxis)):
		if ddddqaxis[j]==1:
			if abs(t_list[-1]-taxis[j])>2*otg.delta_time:
				t_list.append(taxis[j])
	fig, ax = plt.subplots(4,1);
	ax[0].plot(taxis,qaxis,'r--');
	endTime = out_list[-1].trajectory.intermediate_durations;
	ax[0].axvline(x=endTime, color='red', linestyle='--', linewidth=0.5)
	ax[0].set_xticks(t_list, minor=False)
	ax[0].set_yticks([inp.current_position[0] ,inp.target_position[0] ], minor=False)
	ax[0].set_title("q")
	ax[0].grid(True)
	
	ax[1].plot(taxis,dqaxis,'r--');
	ax[1].axvline(x=endTime, color='red', linestyle='--', linewidth=0.5)
	ax[1].set_xticks(t_list, minor=False)
	ax[1].set_yticks([np.min(dqaxis) ,np.max(dqaxis) ], minor=False)	
	ax[1].set_title(r'$\dot{q}$')	
	ax[1].grid(True)
	
	ax[2].plot(taxis,ddqaxis,'r--');
	ax[2].axvline(x=endTime, color='red', linestyle='--', linewidth=0.5)	
	ax[2].set_xticks(t_list, minor=False)
	ax[2].set_yticks([np.min(ddqaxis) ,np.max(ddqaxis) ], minor=False)	
	ax[2].set_title(r'$\ddot{q}$')	
	ax[2].grid(True)
	
	ax[3].plot(taxis,dddqaxis,'r--');			
	ax[3].axvline(x=endTime, color='red', linestyle='--', linewidth=0.5)
	ax[3].set_xticks(t_list, minor=False)
	ax[3].set_yticks([-inp.max_jerk[0] ,inp.max_jerk[0] ], minor=False)	
	ax[3].set_title(r'$\dddot{q}$')	
	ax[3].grid(True)
	fig.tight_layout() 
	plt.subplots_adjust(left=0.125, bottom=0.1, right=0.9, top=0.9, wspace=0.2, hspace=0.7)
	plt.suptitle("Ruckig Trajectory")
	
	
	traj = sg.getTraj(0)
	taxis = np.linspace(0,traj.tt,int(traj.tt/dt))
	qaxis = []
	dqaxis = []
	ddqaxis = []
	dddqaxis = []
	state_list = []
	for t in taxis:
		val = sg.generate(traj,t);	
		qaxis.append(val[0])
		dqaxis.append(val[1])
		ddqaxis.append(val[2])
		dddqaxis.append(val[3])
		state_list.append(val[4]+1)
	state_list = np.array(state_list);		
	t0 = 0
	try:
		t1 = taxis[state_list==1][-1]
	except:
		t1 = t0
	try:
		t2 = taxis[state_list==2][-1]
	except:
		t2 = t1
	try:
		t3 = taxis[state_list==3][-1]
	except:
		t3 = t2
	try:
		t4 = taxis[state_list==4][-1]
	except:
		t4 = t3		
	try:		
		t5 = taxis[state_list==5][-1]
	except:
		t5 = t4		
	try:
		t6 = taxis[state_list==6][-1]
	except:
		t6 = t5		
	try:
		t7 = taxis[state_list==7][-1]
	except:
		t7 = t6		
	t_list = [t0,t1,t2,t3,t4,t5,t6,t7]



	ax[0].plot(taxis,qaxis,'b:');
	endTime = traj.tt;
	ax[0].axvline(x=endTime, color='blue', linestyle='--', linewidth=0.5)
	ax[0].set_xticks(t_list, minor=False)
	ax[0].set_yticks([traj.so,traj.sf ], minor=False)
	ax[0].set_title("q")
	ax[0].grid(True)
	
	ax[1].plot(taxis,dqaxis,'b:');
	ax[1].axvline(x=endTime, color='blue', linestyle='--', linewidth=0.5)
	ax[1].set_xticks(t_list, minor=False)
	ax[1].set_yticks([np.min(dqaxis) ,np.max(dqaxis) ], minor=False)	
	ax[1].set_title(r'$\dot{q}$')	
	ax[1].grid(True)
	
	ax[2].plot(taxis,ddqaxis,'b:');
	ax[2].axvline(x=endTime, color='blue', linestyle='--', linewidth=0.5)	
	ax[2].set_xticks(t_list, minor=False)
	ax[2].set_yticks([np.min(ddqaxis) ,np.max(ddqaxis) ], minor=False)	
	ax[2].set_title(r'$\ddot{q}$')	
	ax[2].grid(True)
	
	ax[3].plot(taxis,dddqaxis,'b:');			
	ax[3].axvline(x=endTime, color='blue', linestyle='--', linewidth=0.5)
	ax[3].set_xticks(t_list, minor=False)
	ax[3].set_yticks([-traj.j ,traj.j ], minor=False)	
	ax[3].set_title(r'$\dddot{q}$')	
	ax[3].grid(True)
	fig.tight_layout() 
	plt.subplots_adjust(left=0.125, bottom=0.1, right=0.9, top=0.9, wspace=0.2, hspace=0.7)	
	plt.suptitle("ScurveGenerator & Ruckig Trajectory")
	plt.show()
def drawList(t_list,s_list,ds_list,dds_list,ddds_list,state_list):
	fig, ax = plt.subplots(4,1)
	fig.tight_layout() 
	state_list = np.array(state_list);
	t_list = np.array(t_list);
	s_list = np.array(s_list);
	ds_list = np.array(ds_list);
	dds_list = np.array(dds_list);
	ddds_list = np.array(ddds_list);
	color_list = np.array([[1,0,0],[1,0.5,0],[0,0,1],[0,1,0],[0,0,1],[1,0.5,0],[1,0,0]])
	t0 = 0
	try:
		t1 = t_list[state_list==1][-1]
	except:
		t1 = t0
	try:
		t2 = t_list[state_list==2][-1]
	except:
		t2 = t1
	try:
		t3 = t_list[state_list==3][-1]
	except:
		t3 = t2
	try:
		t4 = t_list[state_list==4][-1]
	except:
		t4 = t3		
	try:		
		t5 = t_list[state_list==5][-1]
	except:
		t5 = t4		
	try:
		t6 = t_list[state_list==6][-1]
	except:
		t6 = t5		
	try:
		t7 = t_list[state_list==7][-1]
	except:
		t7 = t6		

	for i in range(1,8):
		ax[0].plot(t_list[state_list==i],s_list[state_list==i],color= color_list[i-1,:])
		ax[0].set_title("s")
		ax[0].axvline(x=t1, color='k', linestyle=':', linewidth=0.1)
		ax[0].axvline(x=t2, color='k', linestyle=':', linewidth=0.1)
		ax[0].axvline(x=t3, color='k', linestyle=':', linewidth=0.1)
		ax[0].axvline(x=t4, color='k', linestyle=':', linewidth=0.1)
		ax[0].axvline(x=t5, color='k', linestyle=':', linewidth=0.1)
		ax[0].axvline(x=t6, color='k', linestyle=':', linewidth=0.1)	
		ax[0].axvline(x=t7, color='k', linestyle=':', linewidth=0.1)		
		ax[0].set_xticks([t0,t1,t2,t3,t4,t5,t6,t7])
		ax[1].plot(t_list[state_list==i],ds_list[state_list==i],color= color_list[i-1,:])
		ax[1].set_title("ds")		
		ax[1].axvline(x=t1, color='k', linestyle=':', linewidth=0.1)
		ax[1].axvline(x=t2, color='k', linestyle=':', linewidth=0.1)
		ax[1].axvline(x=t3, color='k', linestyle=':', linewidth=0.1)
		ax[1].axvline(x=t4, color='k', linestyle=':', linewidth=0.1)
		ax[1].axvline(x=t5, color='k', linestyle=':', linewidth=0.1)
		ax[1].axvline(x=t6, color='k', linestyle=':', linewidth=0.1)		
		ax[1].axvline(x=t7, color='k', linestyle=':', linewidth=0.1)		
		ax[1].set_xticks([t0,t1,t2,t3,t4,t5,t6,t7])

		ax[2].plot(t_list[state_list==i],dds_list[state_list==i],color= color_list[i-1,:])
		ax[2].set_title("dds")

		ax[2].axvline(x=t1, color='k', linestyle=':', linewidth=0.1)
		ax[2].axvline(x=t2, color='k', linestyle=':', linewidth=0.1)
		ax[2].axvline(x=t3, color='k', linestyle=':', linewidth=0.1)
		ax[2].axvline(x=t4, color='k', linestyle=':', linewidth=0.1)
		ax[2].axvline(x=t5, color='k', linestyle=':', linewidth=0.1)
		ax[2].axvline(x=t6, color='k', linestyle=':', linewidth=0.1)				
		ax[2].axvline(x=t7, color='k', linestyle=':', linewidth=0.1)		
		ax[2].set_xticks([t0,t1,t2,t3,t4,t5,t6,t7])

		ax[3].plot(t_list[state_list==i],ddds_list[state_list==i],color= color_list[i-1,:])
		ax[3].set_title("ddds")

		ax[3].axvline(x=t1, color='k', linestyle=':', linewidth=0.1)
		ax[3].axvline(x=t2, color='k', linestyle=':', linewidth=0.1)
		ax[3].axvline(x=t3, color='k', linestyle=':', linewidth=0.1)
		ax[3].axvline(x=t4, color='k', linestyle=':', linewidth=0.1)
		ax[3].axvline(x=t5, color='k', linestyle=':', linewidth=0.1)
		ax[3].axvline(x=t6, color='k', linestyle=':', linewidth=0.1)				
		ax[3].axvline(x=t7, color='k', linestyle=':', linewidth=0.1)	
		ax[3].set_xticks([t0,t1,t2,t3,t4,t5,t6,t7])	
	print("Trajectory Time : ",t_list[-1])
	plt.show()
