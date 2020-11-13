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
sys.path.append('../src/pyFun')
from MOMDP import MOMDP, MOMDP_TOQ, MOMDP_TO, MOMDP_Q
matplotlib.rcParams.update({'font.size': 22})

# bag = rosbag.Bag('/home/drew/rosbag/_2020-10-09-17-21-59.bag')
bag = rosbag.Bag('/home/ugo/rosbag/_2020-10-14-00-29-33.bag')
option = 0

if option == 1:
	fileName = sys.path[0]+'/../src/pyFun/multiAgent/segway_7x7_2.pkl'
else:
	fileName = sys.path[0]+'/../src/pyFun/multiAgent/segway_7x7ug_2.pkl'

pickle_in = open(fileName,"rb")
momdp = pickle.load(pickle_in)

dt_mpc = 0.05
col_grid = 7
row_grid = 7

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

def saveGit(name, xaxis, variableAnimate, color, labels, yLimits):
    fig = plt.figure()
    fig.set_tight_layout(True)
    ax = plt.axes()
    lineList = []
    for i in range(0, len(variableAnimate)):
    	line, = ax.plot([], [], color[i], label=labels[i],zorder=1)
    	lineList.append(line)
	plt.legend()

    def update(i):
	    for j in range(0, len(variableAnimate)):
	        lineList[j].set_data(xaxis[0:i], variableAnimate[j][0:i])
	        ax.set(xlim=(0, i+1), ylim=yLimits)
    dataPoints = variableAnimate[0].shape[0]

    anim = FuncAnimation(fig, update, frames=np.arange(0, dataPoints+1), interval=100)
    # anim.save(name+'.gif', dpi=80, writer='imagemagick')
    anim.save(name+'.gif', dpi=80)

def main():

	## =======================================================
	## Read and plot Belief
	## =======================================================
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
	plt.figure()
	if option == 1:
		plt.plot(time_belief, probMiss,'-k', label='Mission success')
		plt.plot(time_belief, probObstArray[:,0],'-ob', label='R1')
		plt.plot(time_belief, probObstArray[:,1],'-og', label='R2')
	else:
		plt.plot(time_belief, probMiss,'-k', label='Mission success')
		plt.plot(time_belief, probObstArray[:,0],'-ob', label='R1')
		plt.plot(time_belief, probObstArray[:,1],'-og', label='R2')
		plt.plot(time_belief, probObstArray[:,2],'-or', label='G1')
		plt.plot(time_belief, probObstArray[:,3],'-oy', label='G2')
	plt.legend()

	if option == 1:
		saveGit('prob', time_belief, [np.array(probMiss), probObstArray[:,0], probObstArray[:,1]], ['k','b','g'],['Mission', 'R1', 'R2'], (-0.1, 1.3))
	else:
		saveGit('prob', time_belief, [np.array(probMiss), probObstArray[:,0], probObstArray[:,1], probObstArray[:,2], probObstArray[:,3]], ['k','b','g','r','y'],['Mission', 'R1', 'R2', 'G1', 'G2'], (-0.1, 1.3))
	## =======================================================
	## Read and plot INPUT
	## =======================================================
	# plt.figure()
	# plt.plot(time_u, u1, label='u1')
	# plt.plot(time_u, u2, label='u2')
	# plt.ylabel('input')
	# plt.legend()

	## =======================================================
	## Read and plot STATE
	## =======================================================
	state_s = []
	time_state_s = []
	for topic, msg, t in bag.read_messages(topics=['/cyberpod/state']):
		# state_t = [msg.x, msg.y, msg.theta, msg.v, msg.thetaDot, msg.psi, msg.psiDot]
		state_t = [msg.state[0], msg.state[1], msg.state[2], msg.state[3], msg.state[4], msg.state[5], msg.state[6]]
		state_s.append(state_t)
		time_state_s.append((len(time_state_s))*0.001)

	state_array = np.array(state_s)
	
	# plt.figure()
	# plt.subplot(711)
	# plt.plot(time_state_s, state_s_array[:,0], label='x')
	# plt.subplot(712)
	# plt.plot(time_state_s, state_s_array[:,1], label='y')
	# plt.subplot(713)
	# plt.plot(time_state_s, state_s_array[:,2], label='theta')
	# plt.subplot(714)
	# plt.plot(time_state_s, state_s_array[:,3], label='v')
	# plt.subplot(715)
	# plt.plot(time_state_s, state_s_array[:,4], label='thetaDot')
	# plt.subplot(716)
	# plt.plot(time_state_s, state_s_array[:,5], label='psi')
	# plt.subplot(717)
	# plt.plot(time_state_s, state_s_array[:,6], label='psiDot')
	# plt.legend()

	state_d = []
	time_state_d = []
	for topic, msg, t in bag.read_messages(topics=['/t2/odom']):
		state_t = [msg.x, msg.y, msg.vbx, msg.vby]
		# state_t = [msg.state[0], msg.state[1], msg.state[2], msg.state[3], msg.state[4], msg.state[5], msg.state[6]]
		state_d.append(state_t)
		time_state_d.append((len(time_state_d))*0.001)

	state_d_array = np.array(state_d)
	
	plt.figure()
	plt.subplot(411)
	plt.plot(time_state_d, state_d_array[:,0], label='x')
	plt.subplot(412)
	plt.plot(time_state_d, state_d_array[:,1], label='y')
	plt.subplot(413)
	plt.plot(time_state_d, state_d_array[:,2], label='vx')
	plt.subplot(414)
	plt.plot(time_state_d, state_d_array[:,3], label='vy')
	plt.legend()






	## =======================================================
	## Read and plot PRED TRAJECTORY and Error
	## =======================================================
	optSol = []
	time_optSol = []
	solverFlag = []
	solverTime = []
	xGoal = []
	yGoal = []
	xCurr = []
	x_IC = []
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

	drone_solv = []
	time_drone_opt = []
	for topic, msg, t in bag.read_messages(topics=['/segway_sim/drone_opt']):
		drone_solv.append(msg.solverTime)
		time_drone_opt.append((len(time_drone_opt))*dt_mpc)
	

	error = []
	print("================== delay_ms: ", delay_ms)
	for i in range(1, len(xCurr)):
		if delay_ms > -0.5:
			error.append((np.array(xCurr[i])-np.array(optSol[i-1][0:7])).tolist())
		else:
			error.append((np.array(xCurr[i])-np.array(optSol[i-1][7:14])).tolist())

	error_array = np.array(error)

	## =======================================================
	## Read and plot XY Multi Agent
	## =======================================================
	fig = plt.figure()
	ax = plt.subplot2grid((1, 1), (0, 0))
	plt.plot(state_s_array[:,0], state_s_array[:,1], '-k',label='Segway')
	plt.plot(state_d_array[:,0], state_d_array[:,1], '-b',label='Drone')
	for i in range(0,row_grid+1):
		plt.plot([i,i], [0,col_grid], '-k')
	for i in range(0, col_grid+1):
		plt.plot([0,row_grid], [i,i], '-k')
	plt.plot(xy_seg_array[:,0], xy_seg_array[:,1], 'sk',label='Segway goal positions')
	plt.plot(xy_drn_array[:,0], xy_drn_array[:,1], 'sb',label='Drone goal positions')
	
	# Draw regions
	# Add goal
	goalColor  =(0.0, 0.7, 0.0)
	if momdp.unGoal == False:
		addStaticComponents(momdp, ax, 1, goalColor)
	else:
		totProb = [0.3, 0.3]
		goalPatchList = addDynamicComponent(momdp, ax, momdp.col_goal, momdp.row_goal, goalColor, totProb)
	
	# Add known static obstacles
	obsColor  =(0.7, 0.2, 0.2)
	addStaticComponents(momdp, ax, -1, obsColor)
	# Add uncertain regions
	obsColor  =(0.7, 0.2, 0.2)
	totProb = [0.4, 0.4]
	obstPatchList = addDynamicComponent(momdp, ax, momdp.col_obs, momdp.row_obs, obsColor, totProb)
	# ax.square()
	ax.set(aspect='equal')
	plt.xlabel('x [m]', fontsize=22)
	plt.ylabel('y [m]', fontsize=22)
	plt.legend(loc=1, fontsize=22, framealpha=1)



	fig = plt.figure()
	# ax = fig.add_subplot(2, 1, 0)
	ax = plt.subplot2grid((20, 1), (0, 0), rowspan=16)
	plt.plot(state_s_array[:,0], state_s_array[:,1], '-k',label='Segway')
	plt.plot(state_d_array[:,0], state_d_array[:,1], '-r',label='Drone')
	for i in range(0,row_grid+1):
		plt.plot([i,i], [0,col_grid], '-k')
	for i in range(0, col_grid+1):
		plt.plot([0,row_grid], [i,i], '-k')
	plt.plot(xy_seg_array[:,0], xy_seg_array[:,1], 'sk',label='Segway goal positions')
	plt.plot(xy_drn_array[:,0], xy_drn_array[:,1], 'sr',label='Drone goal positions')
	
	# Draw regions
	# Add goal
	goalColor  =(0.0, 0.7, 0.0)
	if momdp.unGoal == False:
		addStaticComponents(momdp, ax, 1, goalColor)
	else:
		totProb = [0.3, 0.3]
		goalPatchList = addDynamicComponent(momdp, ax, momdp.col_goal, momdp.row_goal, goalColor, totProb)
	
	# Add known static obstacles
	obsColor  =(1.0, 1.0, 0.0)
	addStaticComponents(momdp, ax, -1, obsColor)
	# Add uncertain regions
	obsColor  =(0.0, 0.1, 0.8)
	totProb = [0.2, 0.2]
	obstPatchList = addDynamicComponent(momdp, ax, momdp.col_obs, momdp.row_obs, obsColor, totProb)
	# ax.square()
	ax.set(aspect='equal')
	plt.xlabel('x [m]', fontsize=22)
	plt.ylabel('y [m]', fontsize=22)
	plt.legend(loc=1, fontsize=22, framealpha=1)
	
	# ax = fig.add_subplot(4, 1, 4)
	ax = plt.subplot2grid((20, 1), (17, 0), rowspan=3)
	if option == 1:
		plt.plot(time_belief, probMiss,'-k', label='Mission success')
		plt.plot(time_belief, probObstArray[:,0],'-ob', label='R1')
		plt.plot(time_belief, probObstArray[:,1],'-og', label='R2')
	else:
		plt.plot(time_belief, probMiss,'-k', label='Mission success')
		plt.plot(time_belief, probObstArray[:,0],'-ob', label='R1')
		plt.plot(time_belief, probObstArray[:,1],'--sb', label='R2')
		plt.plot(time_belief, probObstArray[:,2],'-og', label='G1')
		plt.plot(time_belief, probObstArray[:,3],'--sg', label='G2')
		plt.legend(bbox_to_anchor=(0, 1), loc='upper left', ncol=5, fontsize=18, framealpha=1)	
		plt.ylabel('Probability', fontsize=22)
		plt.ylim(-0.1,1.6)
	plt.xlabel('high-level time k')

	
	# Continous time figure
	fig = plt.figure()
	# subplot 1
	# subplot 2
	ax = plt.subplot2grid((3, 1), (0, 0), rowspan=1)
	plt.plot(time_optSol, solverTime , '-k',label='Segway MPC')
	plt.plot(time_drone_opt[0:-inputOffSet], drone_solv[inputOffSet:] , '-r',label='Drone MPC')
	plt.ylabel('time [s]', fontsize=22)
	plt.legend(bbox_to_anchor=(0, 1), loc='upper left', ncol=2, fontsize=22, framealpha=1)	
	# subplot 3
	ax = plt.subplot2grid((3, 1), (1, 0), rowspan=1)
	plt.plot(time_u, u1, '-r', label='Left motor')
	plt.plot(time_u, u2, '-b', label='Right motor')
	plt.plot([0, time_u[-1]], [10, 10], '-k', label='Constraint')
	plt.plot([0, time_u[-1]], [-10, -10], '-k')
	plt.ylabel('Torque [m]', fontsize=22)
	plt.ylim(-11.0, 11.5)
	plt.legend(bbox_to_anchor=(0, 1), loc='upper left', ncol=3, fontsize=22, framealpha=1)	
	# subplot 4
	ax = plt.subplot2grid((3, 1), (2, 0), rowspan=1)
	plt.plot(time_u_d[0:-inputOffSet], u1_d[inputOffSet:], '-r', label='Vx')
	plt.plot(time_u_d[0:-inputOffSet], u2_d[inputOffSet:], '-b', label='Vy')
	plt.plot([0, time_u_d[-inputOffSet]], [1, 1], '-k', label='Constraint')
	plt.plot([0, time_u_d[-inputOffSet]], [-1, -1], '-k')
	plt.ylim(-1.1, 1.4)
	plt.xlabel('time [s]', fontsize=22)
	plt.ylabel('velocity [m/s]', fontsize=22)
	plt.legend(bbox_to_anchor=(0, 1), loc='upper left', ncol=3, fontsize=22, framealpha=1)	


	fig = plt.figure()
	# subplot 1
	ax = plt.subplot2grid((4, 1), (0, 0), rowspan=1)
	plt.plot(time_belief, probMiss,'-k', label='Mission success')
	plt.plot(time_belief, probObstArray[:,0],'-ob', label='R1')
	plt.plot(time_belief, probObstArray[:,1],'--sb', label='R2')
	plt.plot(time_belief, probObstArray[:,2],'-og', label='G1')
	plt.plot(time_belief, probObstArray[:,3],'--sg', label='G2')
	plt.legend(bbox_to_anchor=(0, 1), loc='upper left', ncol=5, fontsize=22, framealpha=1)	
	plt.ylabel('Probability', fontsize=22)
	plt.ylim(-0.1,1.4)
	# subplot 2
	ax = plt.subplot2grid((4, 1), (1, 0), rowspan=1)
	plt.plot(time_optSol, solverTime , '-g',label='Segway MPC')
	plt.plot(time_drone_opt[0:-inputOffSet], drone_solv[inputOffSet:] , '-b',label='Drone MPC')
	plt.ylabel('time [s]', fontsize=22)
	plt.legend(bbox_to_anchor=(0, 1), loc='upper left', ncol=2, fontsize=22, framealpha=1)	
	# subplot 3
	ax = plt.subplot2grid((4, 1), (2, 0), rowspan=1)
	plt.plot(time_u, u1, '-r', label='Left motor')
	plt.plot(time_u, u2, '-b', label='Right motor')
	plt.plot([0, time_u[-1]], [10, 10], '-k', label='Constraint')
	plt.plot([0, time_u[-1]], [-10, -10], '-k')
	plt.ylabel('Torque [m]', fontsize=22)
	plt.ylim(-11.0, 11.5)
	plt.legend(bbox_to_anchor=(0, 1), loc='upper left', ncol=3, fontsize=22, framealpha=1)	
	# subplot 4
	ax = plt.subplot2grid((4, 1), (3, 0), rowspan=1)
	plt.plot(time_u_d[0:-inputOffSet], u1_d[inputOffSet:], '-r', label='Vx')
	plt.plot(time_u_d[0:-inputOffSet], u2_d[inputOffSet:], '-b', label='Vy')
	plt.plot([0, time_u_d[-inputOffSet]], [1, 1], '-k', label='Constraint')
	plt.plot([0, time_u_d[-inputOffSet]], [-1, -1], '-k')
	plt.ylim(-1.1, 1.4)
	plt.xlabel('time [s]', fontsize=22)
	plt.ylabel('velocity [m/s]', fontsize=22)
	plt.legend(bbox_to_anchor=(0, 1), loc='upper left', ncol=3, fontsize=22, framealpha=1)	
	
	
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
	# plt.plot(time_optSol[0:-1], error_array[:,6], label='x')
	# plt.legend()

	plt.figure()
	plt.plot(time_optSol, solverTime , '-g',label='Segway MPC')
	plt.plot(time_drone_opt, drone_solv , '-b',label='Drone MPC')

	plt.show()

	# plt.figure()
	# plt.plot(time_optSol, solverTime , '-og',label='solverTime')

	# plt.figure()
	# plt.plot(state_s_array[:,0], state_s_array[:,1], '-og',label='xt')
	# xPred, yPred, thetaPred, vPred, thetaDotPred, psiPred, psiDotPred, u1Pred, u2Pred = getPred(optSol[0])
	# plt.plot(xPred, yPred, '-ob')
	# plt.legend()
	# for i in range(0,row_grid+1):
	# 	plt.plot([i,i], [0,col_grid], '-k')
	# for i in range(0, col_grid+1):
	# 	plt.plot([0,row_grid], [i,i], '-k')


	# e0 = []
	# for i in range(0, len(x_IC)):
	# 	e0.append((np.array(x_IC[i])-np.array(optSol[i][0:7])).tolist())

	# e0_array = np.array(e0)
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
	# plt.show()


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
main()
bag.close()

## Read and plot STATE
