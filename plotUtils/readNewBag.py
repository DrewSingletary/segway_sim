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

# bag = rosbag.Bag('/home/ugo/rosbag/_2020-10-31-15-01-06.bag')
# bagNoBarrier = rosbag.Bag('/home/ugo/rosbag/_2020-10-31-15-04-29.bag')

newest = max(glob.iglob('/home/drew/rosbag/*.bag'), key=os.path.getctime)
print("Open: ", newest)
bag = rosbag.Bag(newest)

# bagNoBarrier = rosbag.Bag('/home/drew/rosbag/_2020-11-17-13-05-14.bag')
bagNoBarrier = rosbag.Bag('/home/drew/rosbag/_2020-11-21-17-51-18.bag')
# bagNoBarrier = rosbag.Bag('/home/drew/rosbag/_2020-11-17-13-01-37.bag')


dt_mpc = 0.05
col_grid = 8
row_grid = 8
x_start_s=0.5
x_start_tb=0.5
y_start_s=4.5
y_start_tb=3.5
T_end = 120

opt = 1
if opt == 1:
	# fileName = sys.path[0]+'/../src/pyFun/data/Qug_1/MOMDP_obj_8x8ug_2.pkl'
	# fileName = sys.path[0]+'/../src/pyFun/TOQug_1/MOMDP_obj_10x10ug_2.pkl'
	fileName = sys.path[0]+'/../src/pyFun/data/TOQ_1/MOMDP_obj_5x5_3.pkl'

	pickle_in = open(fileName,"rb")
	momdp = pickle.load(pickle_in)
else:
    gridWorld = '10x10ug'
    numObst = 3
    policy = 'TOQ'
    printLevel = 0
    discOpt = 1
    momdp   = getMOMDP(gridWorld, numObst, policy, printLevel, -1, discOpt, unGoal = True, valFunFlag = False)

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

def addDynamicComponent(momdp, ax, col, row, colorComponent, totProb):
	obstPatchList = []
	for i in range(0, len(row) ):
		x = col[i]
		y = momdp.gridVar.shape[0] - row[i] - 1
		patch = Rectangle((x, y), 1, 1, fc =colorComponent, ec =colorComponent)
		patch.set_alpha(1-totProb[i])
		obstPatchList.append( patch )
		ax.add_patch(obstPatchList[-1])
	return obstPatchList

def addStaticComponents(momdp, ax, typeComponent, colorComponent):
	idxY, idxX = np.where(momdp.gridVar == typeComponent)
	for i in range(0, idxX.shape[0] ):
		x = idxX[i]
		y = momdp.gridVar.shape[0] - idxY[i] - 1
		ax.add_patch( Rectangle((x, y), 1, 1, fc =colorComponent, ec =colorComponent) ) 
input = raw_input("Do you want to plot mid-level data? [y/n] ")
if input == 'y':
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

	xy_seg_array = np.array(xy_seg)
	xy_drn_array = np.array(xy_drn)

	## =======================================================
	## Read and plot INPUT
	## =======================================================
	inputVector = []
	u1=[]
	u2=[]
	time_u = []
	for topic, msg, t in bag.read_messages(topics=['/segway_sim/input']):
		inputVector.append(msg.inputVec)
		if np.abs(msg.inputVec[0]) < 20:
			u1.append(msg.inputVec[0])
			u2.append(msg.inputVec[1])
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
	for topic, msg, t in bag.read_messages(topics=['/segway_sim/state_true']):
		state_t = [msg.x, msg.y, msg.theta, msg.v, msg.thetaDot, msg.psi, msg.psiDot]
		# state_t = [msg.state[0], msg.state[1], msg.state[2], msg.state[3], msg.state[4], msg.state[5], msg.state[6]]
		state.append(state_t)
		time_state.append((len(time_state))*0.001)

	state_array = np.array(state)
	
	plt.figure()
	plt.subplot(711)
	plt.plot(time_state, state_array[:,0], label='x')
	plt.subplot(712)
	plt.plot(time_state, state_array[:,1], label='x')
	plt.subplot(713)
	plt.plot(time_state, state_array[:,2], label='x')
	plt.subplot(714)
	plt.plot(time_state, state_array[:,3], label='x')
	plt.subplot(715)
	plt.plot(time_state, state_array[:,4], label='x')
	plt.subplot(716)
	plt.plot(time_state, state_array[:,5], label='x')
	plt.subplot(717)
	plt.plot(time_state, state_array[:,6], label='x')
	plt.legend()

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
	xyContPlan = []
	for topic, msg, t in bag.read_messages(topics=['/segway_sim/optimal_sol']):
		optSol.append(msg.optimalSolution)
		time_optSol.append((len(time_optSol))*dt_mpc)
		solverFlag.append(msg.solverFlag)
		solverTime.append(msg.solverTime)
		xGoal.append(msg.x)
		yGoal.append(msg.y)
		x_IC.append(msg.x_IC)
		xCurr.append(msg.xCurr)
		delay_ms = msg.delay_ms
		if msg.contPlan == 1:
			xyContPlan.append(optSol[-1][0:2])



	error = []
	print("================== delay_ms: ", delay_ms)
	for i in range(1, len(xCurr)):
		if delay_ms > -0.5:
			error.append((np.array(xCurr[i])-np.array(optSol[i-1][0:7])).tolist())
		else:
			error.append((np.array(xCurr[i])-np.array(optSol[i-1][7:14])).tolist())

	error_array = np.array(error)
	
	plt.figure()
	plt.subplot(711)
	plt.plot(time_optSol[0:-1], error_array[:,0], label='x')
	plt.subplot(712)
	plt.plot(time_optSol[0:-1], error_array[:,1], label='x')
	plt.subplot(713)
	plt.plot(time_optSol[0:-1], error_array[:,2], label='x')
	plt.subplot(714)
	plt.plot(time_optSol[0:-1], error_array[:,3], label='x')
	plt.subplot(715)
	plt.plot(time_optSol[0:-1], error_array[:,4], label='x')
	plt.subplot(716)
	plt.plot(time_optSol[0:-1], error_array[:,5], label='x')
	plt.subplot(717)
	plt.plot(time_optSol[0:-1], error_array[:,6], label='x')
	plt.legend()

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
	plt.figure()
	plt.subplot(711)
	plt.plot(time_optSol, e0_array[:,0], label='x')
	plt.subplot(712)
	plt.plot(time_optSol, e0_array[:,1], label='x')
	plt.subplot(713)
	plt.plot(time_optSol, e0_array[:,2], label='x')
	plt.subplot(714)
	plt.plot(time_optSol, e0_array[:,3], label='x')
	plt.subplot(715)
	plt.plot(time_optSol, e0_array[:,4], label='x')
	plt.subplot(716)
	plt.plot(time_optSol, e0_array[:,5], label='x')
	plt.subplot(717)
	plt.plot(time_optSol, e0_array[:,6], label='e_0')
	plt.legend()


	fig = plt.figure()
	ax = plt.subplot2grid((1, 1), (0, 0))
	plt.plot(state_array[:,0], state_array[:,1], '-k',label='Segway')
	for i in range(0,row_grid+1):
		plt.plot([i,i], [0,col_grid], '-k')
	for i in range(0, col_grid+1):
		plt.plot([0,row_grid], [i,i], '-k')
	plt.plot(xy_seg_array[:,0], xy_seg_array[:,1], 'sk',label='mid-level goals')
	# plt.plot(0.5, 7.5, 'sk')
	
	# Draw regions
	# Add goal
	goalColor  =(0.0, 0.7, 0.0)
	if momdp.unGoal == False:
		addStaticComponents(momdp, ax, 1, goalColor)
	else:
		totProb = [0.3, 0.3]
		goalPatchList = addDynamicComponent(momdp, ax, momdp.col_goal, momdp.row_goal, goalColor, totProb)
	
	# Add known static obstacles
	obsColor  =(0.5, 0.0, 0.0)
	addStaticComponents(momdp, ax, -1, obsColor)
	# Add uncertain regions
	obsColor  =(0.5, 0.0, 0.0)
	totProb = [0.8, 0.8, 0.8]
	obstPatchList = addDynamicComponent(momdp, ax, momdp.col_obs, momdp.row_obs, obsColor, totProb)
	# ax.square()
	for i in range(0, len(xyContPlan)):
		if i == 0: 	
			plt.plot(xyContPlan[i][0], xyContPlan[i][1], 'ob', label='solved contingency MPC')
		else:
			plt.plot(xyContPlan[i][0], xyContPlan[i][1], 'ob')

	ax.set(aspect='equal')
	plt.xlabel('x [m]', fontsize=22)
	plt.ylabel('y [m]', fontsize=22)
	plt.legend(loc=1, fontsize=22, framealpha=1)


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

	xy_seg_array = np.array(xy_seg)
	xy_drn_array = np.array(xy_drn)

	BeliefArray = np.array(Belief)
	probObstArray = 1-np.array(probObst)
	plt.figure(figsize=(12,10))
	plt.plot(time_belief, probObstArray[:,0],'-o', color='brown', label='R1')
	plt.plot(time_belief, probObstArray[:,1],'--s', color='brown', label='R2')
	# plt.plot(time_belief, probObstArray[:,2],'-.*', color='brown', label='R3')
	# plt.plot(time_belief, probObstArray[:,3],'--sg', label='G1')
	# plt.plot(time_belief, probObstArray[:,4],'--sg', label='G2')
	plt.plot(time_belief, probObstArray[:,2],'--sg', label='G1')
	plt.plot(time_belief, probObstArray[:,3],'--sg', label='G2')
	plt.plot(time_belief, probMiss,'-k', label='Mission success')
	plt.legend(bbox_to_anchor=(0, 1), loc='upper left', ncol=6, fontsize=18, framealpha=1)	
	plt.xlabel("High-level time k", fontsize=22)
	plt.ylabel("Probability", fontsize=22)
	plt.ylim(-0.01,1.1)
	plt.show()

	input = raw_input("Do you want to plot an animation for the predicted trajectory? [y/n] ")
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
input = raw_input("Do you want to plot low-level data? [y/n] ")
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
	QPtime = []
	dt_lowlevel = 0.001
	for topic, msg, t in bag.read_messages(topics=['/segway_sim/lowLevelLog']):
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
		QPtime.append(msg.QPtime)

	time_lowLevel_noBarrier = []
	h_noBarrier = []
	for topic, msg, t in bagNoBarrier.read_messages(topics=['/segway_sim/lowLevelLog']):
			time_lowLevel_noBarrier.append((len(time_lowLevel_noBarrier))*dt_lowlevel)
			h_noBarrier.append(msg.h)
			
	plt.figure(figsize=(12,10))
	plt.plot(time_lowLevel_noBarrier, h_noBarrier, '-r', label=' mid-layer + MPC')
	plt.plot(time_lowLevel, h, '-b', label='proposed strategy')
	plt.plot([-10, 100], [0,0], '-k')
	plt.ylabel('h(e)')
	plt.xlabel('Time [s]')
	plt.legend(loc=4)
	# plt.ylim(-1.1, 1)

	plt.figure(figsize=(12,10))
	plt.plot(time_optSol[1:], solverTime[1:], '-k', label='mid-level')
	plt.plot(time_lowLevel[1:], QPtime[1:], '-b', label='low-level')
	plt.ylabel('Solver time [s]')
	plt.xlabel('Time [s]')
	plt.legend()
	# plt.ylim(-1.1, 1)

	plt.figure(figsize=(12,10))
	plt.subplot(211)
	plt.plot(time_lowLevel, h, '-o', label='h')
	plt.subplot(212)
	plt.plot(time_lowLevel, V, '-o', label='V')

	uMPC_array = np.array(uMPC);
	uCBF_array = np.array(uCBF);
	uTot_array = np.array(uTot);

	plt.figure(figsize=(12,10))
	plt.subplot(211)
	plt.plot(time_lowLevel, uMPC_array[:, 0], '-r', label='mid-level input')
	plt.plot(time_lowLevel, uCBF_array[:, 0], '-k', label='low-level input')
	plt.plot(time_lowLevel, uTot_array[:, 0], '-b', label='total input')
	plt.ylabel('Input [N/m]')
	plt.subplot(212)
	plt.legend(bbox_to_anchor=(0, 1), loc='upper left', ncol=6, fontsize=18, framealpha=1)	
	plt.plot(time_lowLevel, uMPC_array[:, 1], '-r', label='mid-level input')
	plt.plot(time_lowLevel, uCBF_array[:, 1], '-k', label='low-level input')
	plt.plot(time_lowLevel, uTot_array[:, 1], '-b', label='total input')
	plt.ylabel('Input [N/m]')
	plt.xlabel('Time [s]')
	plt.legend(bbox_to_anchor=(0, 1), loc='upper left', ncol=6, fontsize=18, framealpha=1)	


	plt.figure(figsize=(12,10))
	plt.subplot(211)
	plt.plot(time_lowLevel, uMPC_array[:, 0], '-r', label='mid-level input')
	plt.plot(time_lowLevel, uCBF_array[:, 0], '-k', label='low-level input')
	plt.plot(time_lowLevel, uTot_array[:, 0], '-b', label='total input')
	plt.ylabel('Input [N/m]')
	plt.legend(bbox_to_anchor=(0, 1), loc='upper left', ncol=6, fontsize=18, framealpha=1)	
	plt.xlim(14.5, 15.5)
	plt.subplot(212)
	plt.plot(time_lowLevel, uMPC_array[:, 1], '-r', label='mid-level input')
	plt.plot(time_lowLevel, uCBF_array[:, 1], '-k', label='low-level input')
	plt.plot(time_lowLevel, uTot_array[:, 1], '-b', label='total input')
	plt.ylabel('Input [N/m]')
	plt.xlabel('Time [s]')
	plt.legend(bbox_to_anchor=(0, 1), loc='upper left', ncol=6, fontsize=18, framealpha=1)	
	plt.xlim(14.5, 15.5)

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
