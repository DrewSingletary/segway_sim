import rosbag
import sys
import pickle
import pdb
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle 
from matplotlib.animation import FuncAnimation
import scipy.io as sio
import os
sys.path.append('../src/pyFun')
from tempfile import TemporaryFile
import glob
from nav_msgs.msg import Odometry as stateTB
from geometry_msgs.msg import Twist
from main import getMOMDP

from MOMDP import MOMDP, MOMDP_TOQ, MOMDP_TO, MOMDP_Q
matplotlib.rcParams.update({'font.size': 22})


# newest = max(glob.iglob('/home/drew/rosbag_exp/*.bag'), key=os.path.getctime)
# print("Open: ", newest)
# bag = rosbag.Bag(newest)

# bagNoBarrier = rosbag.Bag('/home/drew/rosbag_exp/_2020-11-23-13-22-12.bag')
# bagNoBarrier = rosbag.Bag('/home/drew/rosbag_exp/_2020-11-23-13-22-12.bag')
# bag = rosbag.Bag('/home/drew/rosbag_exp/_2020-11-23-13-24-26.bag')

# # Exp 1
# bagNoBarrier = rosbag.Bag('/home/drew/rosbag_exp/_2020-11-21-19-57-44.bag')
# bag = rosbag.Bag('/home/drew/rosbag_exp/_2020-11-21-19-59-41.bag')
# 
# Exp 2
# bag = rosbag.Bag('/home/ugo/expDataFinal/expComp_2/_2020-11-21-20-16-55.bag')
# bagNoBarrier = rosbag.Bag('/home/ugo/expDataFinal/expComp_2/_2020-11-21-20-18-57.bag')

# # Exp 3
# bag = rosbag.Bag('/home/drew/rosbag_exp/_2020-11-23-11-31-39.bag')
# bagNoBarrier = rosbag.Bag('/home/drew/rosbag_exp/_2020-11-23-10-53-04.bag')

# # Exp 4
# bag = rosbag.Bag('/home/drew/rosbag_exp/_2020-11-23-13-45-54.bag')
# bagNoBarrier = rosbag.Bag('/home/drew/rosbag_exp/_2020-11-23-13-41-34.bag')

# Video 2 exp 2
# bag = rosbag.Bag('/home/ugo/segExp/test_2_video_2/_2020-11-23-13-49-16.bag')
# bagNoBarrier = rosbag.Bag('/home/segExp/ugo/test_2_video_2/_2020-11-23-13-47-15.bag')

# test 5
bag = rosbag.Bag('/home/ugo/expDataSeg/test_5/_2020-11-23-13-49-16.bag')
bagNoBarrier = rosbag.Bag('/home/ugo/expDataSeg/test_5/_2020-11-23-13-22-12.bag')

# # test 4
# bagNoBarrier = rosbag.Bag('/home/ugo/expDataSeg/test_4/_2020-11-23-10-53-04.bag')
# bag = rosbag.Bag('/home/ugo/expDataSeg/test_4/_2020-11-23-11-31-39.bag')

# test 3
bagNoBarrier = rosbag.Bag('/home/ugo/expDataSeg/test_3/_2020-11-23-13-47-15.bag')
bag = rosbag.Bag('/home/ugo/expDataSeg/test_3/_2020-11-23-13-49-16.bag')

x_start = 0.5
y_start = 4.5
dt_mpc = 0.05

def getPred(optSol):
	xPred = []
	yPred = []
	thetaPred = []
	vPred = []
	thetaDotPred = []
	psiPred = []
	psiDotPred = []
	u1Pred = []
	u2Pred = []
	nx = 7; nu = 2; N = 40;
	for j in range(0,N+1):
		xPred.append(optSol[j*nx + 0])
		yPred.append(optSol[j*nx + 1])
		thetaPred.append(optSol[j*nx + 2])
		vPred.append(optSol[j*nx + 3])
		thetaDotPred.append(optSol[j*nx + 4])
		psiPred.append(optSol[j*nx + 5])
		psiDotPred.append(optSol[j*nx + 6])
	for j in range(0,N):
		u1Pred.append(optSol[(N+1)*nx + j*nu + 0])
		u2Pred.append(optSol[(N+1)*nx + j*nu + 1])

	return xPred, yPred, thetaPred, vPred, thetaDotPred, psiPred, psiDotPred, u1Pred, u2Pred




input = 'y'#raw_input("Do you want to plot mid-level data? [y/n] ")
if input == 'y':
	tmin = 7+9
	tmax = 44+9
	
	dt_ll = 1/800.0
	
	h_val_noBarrier = []
	delay_t_noBarrier = []
	t_lowLevel_noBarrier = []
	uTot_noBarrieri = []
	timeCounter = []
	for topic, msg, t in bagNoBarrier.read_messages(topics=['/cyberpod/ctrl_info']):
		timeCounter.append((len(timeCounter))*dt_ll)
		if (timeCounter[-1] > tmin) and (timeCounter[-1] < tmax):
			delay_t_noBarrier.append(msg.data[0])
			uTot_noBarrieri.append([msg.data[1], msg.data[2]])
			h_val_noBarrier.append(msg.data[7])
			t_lowLevel_noBarrier.append((len(t_lowLevel_noBarrier))*dt_ll)

	uTot = []
	uCBF = []
	uMPC = []
	h_val = []
	t_lowLevel = []
	delay_t = []
	timeCounter = []
	for topic, msg, t in bag.read_messages(topics=['/cyberpod/ctrl_info']):
		timeCounter.append((len(timeCounter))*dt_ll)
		if (timeCounter[-1] > tmin) and (timeCounter[-1] < tmax):
			delay_t.append(msg.data[0])
			uTot.append([msg.data[1], msg.data[2]])
			uMPC.append([msg.data[3], msg.data[4]])
			uCBF.append([msg.data[5], msg.data[6]])
			h_val.append(msg.data[7])
			t_lowLevel.append((len(t_lowLevel))*dt_ll)

	plt.figure(figsize=(12,10))
	plt.plot(t_lowLevel_noBarrier, h_val_noBarrier, '-r', label='naive MPC')
	plt.plot(t_lowLevel, h_val, '-b', label='proposed strategy')
	plt.plot([t_lowLevel[0], t_lowLevel[-1]], [0, 0],'-k')
	plt.xlabel('Time [s]')
	plt.ylabel('h(e)')
	plt.legend(loc=0)
	plt.ylim(-2,1)

	plt.figure()
	plt.plot(t_lowLevel_noBarrier, delay_t_noBarrier, '-r', label='naive MPC')
	plt.plot(t_lowLevel, delay_t, '-b', label='proposed strategy')
	plt.ylabel('delay')
	plt.legend()

	uMPC_array = np.array(uMPC)
	uCBF_array = np.array(uCBF)
	uTot_array = np.array(uTot)

	plt.figure(figsize=(12,10))
	plt.subplot(211)
	plt.plot(t_lowLevel, uMPC_array[:, 0], '-r', label='mid-level input')
	plt.plot(t_lowLevel, uCBF_array[:, 0], '-k', label='low-level input')
	plt.plot(t_lowLevel, uTot_array[:, 0], '-b', label='total input')
	plt.xlim(30.85,31.15)
	plt.ylim(-4,2)
	plt.legend(bbox_to_anchor=(0, 1), loc='upper left', ncol=6, fontsize=18, framealpha=1)	
	plt.subplot(212)
	plt.plot(t_lowLevel, uMPC_array[:, 1], '-r', label='mid-level input')
	plt.plot(t_lowLevel, uCBF_array[:, 1], '-k', label='low-level input')
	plt.plot(t_lowLevel, uTot_array[:, 1], '-b', label='total input')
	plt.ylabel('Input [N/m]')
	plt.xlabel('Time [s]')
	plt.legend(bbox_to_anchor=(0, 1), loc='upper left', ncol=6, fontsize=18, framealpha=1)	
	plt.xlim(30.85,31.15)
	plt.ylim(-4,2)

	uTot_noBarrieri_array = np.array(uTot_noBarrieri)
	plt.figure(figsize=(12,10))
	plt.subplot(211)
	plt.plot(t_lowLevel_noBarrier, uTot_noBarrieri_array[:, 0], '-b', label='tot no barrier')
	plt.subplot(212)
	plt.plot(t_lowLevel_noBarrier, uTot_noBarrieri_array[:, 1], '-b', label='tot no barrier')
	plt.ylabel('input')
	plt.legend()

	## =======================================================
	## Read and plot INPUT
	## =======================================================
	inputVector = []
	u1=[]
	u2=[]
	time_u = []
	for topic, msg, t in bag.read_messages(topics=['/cyberpod/input']):
		inputVector.append(msg.input)
		if np.abs(msg.input[0]) < 20:
			u1.append(msg.input[0])
			u2.append(msg.input[1])
		else:
			u1.append(20.0)
			u2.append(20.0)
		time_u.append((len(time_u))*0.001)

	plt.figure()
	plt.plot(time_u, u1, label='u1')
	plt.plot(time_u, u2, label='u2')
	plt.ylabel('input')
	plt.legend()

	## =======================================================
	## Read and plot STATE
	## =======================================================
	state = []
	time_state = []
	for topic, msg, t in bag.read_messages(topics=['/cyberpod/state']):
		# state_t = [msg.x, msg.y, msg.theta, msg.v, msg.thetaDot, msg.psi, msg.psiDot]
		state_t = [msg.state[0]+x_start, msg.state[1]+y_start, msg.state[2], msg.state[3], msg.state[4], msg.state[5], msg.state[6]]
		state.append(state_t)
		time_state.append((len(time_state))*0.001)

	state_array = np.array(state)
	

	# plt.figure()
	# plt.subplot(711)
	# plt.plot(time_state, state_array[:,0], label='x')
	# plt.subplot(712)
	# plt.plot(time_state, state_array[:,1], label='x')
	# plt.subplot(713)
	# plt.plot(time_state, state_array[:,2], label='x')
	# plt.subplot(714)
	# plt.plot(time_state, state_array[:,3], label='x')
	# plt.subplot(715)
	# plt.plot(time_state, state_array[:,4], label='x')
	# plt.subplot(716)
	# plt.plot(time_state, state_array[:,5], label='x')
	# plt.subplot(717)
	# plt.plot(time_state, state_array[:,6], label='x')
	# plt.legend()

	## =======================================================
	## Read and plot PRED TRAJECTORY
	## =======================================================
	optSol = []
	time_optSol = []
	solverFlag = []
	solverTime = []
	xGoal = []
	yGoal = []
	xCurr = []
	x_IC = []
	for topic, msg, t in bag.read_messages(topics=['/cyberpod/optimal_sol']):
		optSol.append(msg.optimalSolution)
		time_optSol.append((len(time_optSol))*dt_mpc)
		solverFlag.append(msg.solverFlag)
		solverTime.append(msg.solverTime)
		xGoal.append(msg.x)
		yGoal.append(msg.y)
		x_IC.append(msg.x_IC)
		xCurr.append(msg.xCurr)
		delay_ms = msg.delay_ms

	error = []
	print("================== delay_ms: ", delay_ms)
	for i in range(1, len(xCurr)):
		if delay_ms > -0.5:
			error.append((np.array(xCurr[i])-np.array(optSol[i-1][0:7])).tolist())
		else:
			error.append((np.array(xCurr[i])-np.array(optSol[i-1][7:14])).tolist())

	error_array = np.array(error)
	
	# plt.figure()
	# plt.subplot(711)
	# plt.plot(time_optSol[0:-1], error_array[:,0], label='x')
	# plt.subplot(712)
	# plt.plot(time_optSol[0:-1], error_array[:,1], label='x')
	# plt.subplot(713)
	# plt.plot(time_optSol[0:-1], error_array[:,2], label='x')
	# plt.subplot(714)
	# plt.plot(time_optSol[0:-1], error_array[:,3], label='x')
	# plt.subplot(715)
	# plt.plot(time_optSol[0:-1], error_array[:,4], label='x')
	# plt.subplot(716)
	# plt.plot(time_optSol[0:-1], error_array[:,5], label='x')
	# plt.subplot(717)
	# plt.plot(time_optSol[0:-1], error_array[:,6], label='prediction error')
	# plt.legend()

	plt.figure()
	plt.plot(time_optSol, solverFlag , '-og',label='solverFlag')
	plt.figure()
	plt.plot(time_optSol, solverTime , '-og',label='solverTime')

	plt.figure()
	plt.plot(state_array[:,0], state_array[:,1], '-og',label='xt')
	xPred, yPred, thetaPred, vPred, thetaDotPred, psiPred, psiDotPred, u1Pred, u2Pred = getPred(optSol[0])
	plt.plot(xPred, yPred, '-ob')
	plt.legend()
	for i in range(0,6):
		plt.plot([i,i], [0,5], '-k')
		plt.plot([0,5], [i,i], '-k')


	e0 = []
	for i in range(0, len(x_IC)):
		e0.append((np.array(x_IC[i])-np.array(optSol[i][0:7])).tolist())

	e0_array = np.array(e0)
	# plt.figure()
	# plt.subplot(711)
	# plt.plot(time_optSol, e0_array[:,0], label='x')
	# plt.subplot(712)
	# plt.plot(time_optSol, e0_array[:,1], label='x')
	# plt.subplot(713)
	# plt.plot(time_optSol, e0_array[:,2], label='x')
	# plt.subplot(714)
	# plt.plot(time_optSol, e0_array[:,3], label='x')
	# plt.subplot(715)
	# plt.plot(time_optSol, e0_array[:,4], label='x')
	# plt.subplot(716)
	# plt.plot(time_optSol, e0_array[:,5], label='x')
	# plt.subplot(717)
	# plt.plot(time_optSol, e0_array[:,6], label='e_0')
	# plt.legend()

	probMiss = []
	Belief   = []
	probObst = []
	time_belief = []
	xy_seg = []
	xy_drn = []
	for topic, msg, t in bag.read_messages(topics=['/segway_sim/highLevelBelief']):
		probMiss.append(msg.probMiss)
		Belief.append(msg.bt)
		probObst.append(msg.prob)
		time_belief.append((len(time_belief)))
		if msg.targetPosDrone[0] > 0 and msg.targetPosDrone[1]>0:
			xy_drn.append(msg.targetPosDrone)
		if msg.targetPosSegway[0] > 0 and msg.targetPosSegway[1]>0:
			xy_seg.append(msg.targetPosSegway)

	probObstArray = 1-np.array(probObst)

	with open('barrier.npy', 'wb') as f:
		np.save(f, t_lowLevel)
		np.save(f, np.array(h_val))
		np.save(f, t_lowLevel_noBarrier)
		np.save(f, np.array(h_val_noBarrier))

	with open('obstBelief.npy', 'wb') as f:
		np.save(f, time_belief)
		np.save(f, np.array(probMiss))
		np.save(f, np.array(probObstArray))

	plt.show()

	input = 'n'#raw_input("Do you want to plot an animation for the predicted trajectory? [y/n] ")
	if input == 'y':
		fig = plt.figure(200)
		for i in range(0,7):
			plt.plot([i,i], [0,6], '-k')
			plt.plot([0,6], [i,i], '-k')

		plt.plot(state_array[:,0], state_array[:,1], '-og',label='xt')
		ax = plt.axes()
		goal,       = ax.plot(xGoal[0], yGoal[0], '-sr')
		prediction, = ax.plot(xPred, yPred, '-ob')
		for i in range(0,len(optSol)):
			xPred, yPred, thetaPred, vPred, thetaDotPred, psiPred, psiDotPred, u1Pred, u2Pred = getPred(optSol[i])
			prediction.set_data(xPred, yPred)
			goal.set_data(xGoal[i], yGoal[i])
			plt.draw()
			plt.pause(0.1)
			
		fig = plt.figure(99)
		ax1 = fig.add_subplot(2, 1, 1)
		plt.plot(time_u, u1, label='x')
		plt.xlim(0, time_u[-1]+20)
		u1Plot, = ax1.plot(range(0, len(u1)), u1, '-sr')
		ax2 = fig.add_subplot(2, 1, 2)
		plt.plot(time_u, u2, label='x')
		plt.xlim(0, time_u[-1]+20)
		u2Plot, = ax2.plot(range(0, len(u1)), u2, '-sr')

		for i in range(0,len(optSol)):
			xPred, yPred, thetaPred, vPred, thetaDotPred, psiPred, psiDotPred, u1Pred, u2Pred = getPred(optSol[i])
			t = time_optSol[i]
			u1Plot.set_data(np.arange(t,t+len(u1Pred)*dt_mpc, dt_mpc ) , u1Pred)
			u2Plot.set_data(np.arange(t,t+len(u2Pred)*dt_mpc, dt_mpc ) , u2Pred)
			plt.draw()
			plt.pause(0.1)

		fig = plt.figure(100)
		ax1 = fig.add_subplot(5, 1, 1)
		plt.plot(time_state, state_array[:,2], label='x')
		thetaPredPlot, = ax1.plot(range(0, len(thetaPred)), thetaPred, '-sr')
		ax2 = fig.add_subplot(5, 1, 2)
		plt.plot(time_state, state_array[:,3], label='x')
		vPredPlot, = ax2.plot(range(0, len(vPred)), vPred, '-sr')
		ax3 = fig.add_subplot(5, 1, 3)
		plt.plot(time_state, state_array[:,4], label='x')
		thetaDotPredPlot, = ax3.plot(range(0, len(thetaDotPred)), thetaDotPred, '-sr')
		ax4 = fig.add_subplot(5, 1, 4)
		plt.plot(time_state, state_array[:,5], label='x')
		psiPredPlot, = ax4.plot(range(0, len(psiPred)), psiPred, '-sr')
		ax5 = fig.add_subplot(5, 1, 5)
		plt.plot(time_state, state_array[:,6], label='x')
		psiDotPredPlot, = ax5.plot(range(0, len(psiDotPred)), psiDotPred, '-sr')

		for i in range(0,len(optSol)):
			xPred, yPred, thetaPred, vPred, thetaDotPred, psiPred, psiDotPred, u1Pred, u2Pred = getPred(optSol[i])
			t = time_optSol[i]
			thetaPredPlot.set_data(np.arange(t,t+len(thetaPred)*dt_mpc, dt_mpc ) , thetaPred)
			vPredPlot.set_data(np.arange(t,t+len(vPred)*dt_mpc, dt_mpc ) , vPred)
			thetaDotPredPlot.set_data(np.arange(t,t+len(thetaDotPred)*dt_mpc, dt_mpc ) , thetaDotPred)
			psiPredPlot.set_data(np.arange(t,t+len(psiPred)*dt_mpc, dt_mpc ) , psiPred)
			psiDotPredPlot.set_data(np.arange(t,t+len(psiDotPred)*dt_mpc, dt_mpc ) , psiDotPred)
			plt.draw()
			plt.pause(0.1)

			# pdb.set_trace()


		# plt.subplot(512)
		# plt.plot(time_state, state_array[:,3], label='x')
		# plt.subplot(513)
		# plt.plot(time_state, state_array[:,4], label='x')
		# plt.subplot(514)
		# plt.plot(time_state, state_array[:,5], label='x')
		# plt.subplot(515)
		# plt.plot(time_state, state_array[:,6], label='x')

## =======================================================
## Read Low Level Log
## =======================================================
input = 'n'#raw_input("Do you want to plot low-level data? [y/n] ")
if input == 'y':
	X = []
	Xn = []
	uMPC = []
	uCBF = []
	uTot = []
	flagQP = []
	V = []
	h = []
	time_lowLevel = []
	error_lowLevel = []
	dt_lowlevel = 0.001
	for topic, msg, t in bag.read_messages(topics=['/cyberpod/ctrl_info_']):
		pdb.set_trace()
		X.append(msg.X)
		Xn.append(msg.Xn)
		error_lowLevel.append( (np.array(msg.X)-np.array(msg.Xn)).tolist() )
		time_lowLevel.append((len(time_lowLevel))*dt_lowlevel)
		flagQP.append(msg.flagQP)
		uMPC.append(msg.uMPC)
		uCBF.append(msg.uCBF)
		uTot.append(msg.uTot)
		V.append(msg.V)
		h.append(msg.h)

	plt.figure()
	plt.plot(time_lowLevel, h, '-o', label='h')
	plt.ylabel('h')
	plt.xlabel('Time')
	plt.ylim(-1.1, 1)

	plt.figure()
	plt.subplot(211)
	plt.plot(time_lowLevel, h, '-o', label='h')
	plt.subplot(212)
	plt.plot(time_lowLevel, V, '-o', label='V')

	uMPC_array = np.array(uMPC);
	uCBF_array = np.array(uCBF);
	uTot_array = np.array(uTot);

	plt.figure()
	plt.subplot(211)
	plt.plot(time_lowLevel, uMPC_array[:, 0], '-o', label='MPC')
	plt.plot(time_lowLevel, uCBF_array[:, 0], '-o', label='CBF')
	plt.plot(time_lowLevel, uTot_array[:, 0], '-o', label='tot')
	plt.subplot(212)
	plt.plot(time_lowLevel, uMPC_array[:, 1], '-o', label='MPC')
	plt.plot(time_lowLevel, uCBF_array[:, 1], '-o', label='CBF')
	plt.plot(time_lowLevel, uTot_array[:, 1], '-o', label='tot')
	plt.ylabel('input')
	plt.legend()

	plt.figure()
	plt.plot(time_lowLevel, uMPC_array[:, 0], '-o', label='MPC')
	plt.plot(time_lowLevel, uCBF_array[:, 0], '-o', label='CBF')
	plt.plot(time_lowLevel, uTot_array[:, 0], '-o', label='tot')
	plt.figure()
	plt.plot(time_lowLevel, uMPC_array[:, 1], '-o', label='MPC')
	plt.plot(time_lowLevel, uCBF_array[:, 1], '-o', label='CBF')
	plt.plot(time_lowLevel, uTot_array[:, 1], '-o', label='tot')
	plt.ylabel('input')
	plt.legend()

	error_array = np.array(error_lowLevel)
	X_array = np.array(X)
	Xn_array = np.array(Xn)

	plt.figure()
	plt.subplot(711)
	plt.plot(time_lowLevel, error_array[:,0], '-o', label='x')
	plt.subplot(712)
	plt.plot(time_lowLevel, error_array[:,1], '-o', label='x')
	plt.subplot(713)
	plt.plot(time_lowLevel, error_array[:,2], '-o', label='x')
	plt.subplot(714)
	plt.plot(time_lowLevel, error_array[:,3], '-o', label='x')
	plt.subplot(715)
	plt.plot(time_lowLevel, error_array[:,4], '-o', label='x')
	plt.subplot(716)
	plt.plot(time_lowLevel, error_array[:,5], '-o', label='x')
	plt.subplot(717)
	plt.plot(time_lowLevel, error_array[:,6], '-o', label='error')
	plt.legend()

	plt.figure()
	plt.subplot(711)
	plt.plot(time_lowLevel, error_array[:,0], '-o', label='x')
	plt.plot(time_lowLevel, X_array[:,0], '-o', label='x')
	plt.plot(time_lowLevel, Xn_array[:,0], '-o', label='x')
	plt.subplot(712)
	plt.plot(time_lowLevel, error_array[:,1], '-o', label='x')
	plt.plot(time_lowLevel, X_array[:,1], '-o', label='x')
	plt.plot(time_lowLevel, Xn_array[:,1], '-o', label='x')
	plt.subplot(713)
	plt.plot(time_lowLevel, error_array[:,2], '-o', label='x')
	plt.plot(time_lowLevel, X_array[:,2], '-o', label='x')
	plt.plot(time_lowLevel, Xn_array[:,2], '-o', label='x')
	plt.subplot(714)
	plt.plot(time_lowLevel, error_array[:,3], '-o', label='x')
	plt.plot(time_lowLevel, X_array[:,3], '-o', label='x')
	plt.plot(time_lowLevel, Xn_array[:,3], '-o', label='x')
	plt.subplot(715)
	plt.plot(time_lowLevel, error_array[:,4], '-o', label='x')
	plt.plot(time_lowLevel, X_array[:,4], '-o', label='x')
	plt.plot(time_lowLevel, Xn_array[:,4], '-o', label='x')
	plt.subplot(716)
	plt.plot(time_lowLevel, error_array[:,5], '-o', label='x')
	plt.plot(time_lowLevel, X_array[:,5], '-o', label='x')
	plt.plot(time_lowLevel, Xn_array[:,5], '-o', label='x')
	plt.subplot(717)
	plt.plot(time_lowLevel, error_array[:,6], '-o', label='error')
	plt.plot(time_lowLevel, X_array[:,6], '-o', label='x')
	plt.plot(time_lowLevel, Xn_array[:,6], '-o', label='xn')
	plt.legend()

	plt.figure()
	plt.plot(time_lowLevel, flagQP, '-o', label='x')


	plt.show()

bag.close()

## Read and plot STATE
