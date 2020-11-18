
import pdb
import scipy.io as sio
import numpy as np
import pickle
from MOMDP import MOMDP
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle 
from matplotlib import animation

def plotFun(momdp, xt, bt):
	fig = plt.figure()
	ax = fig.add_subplot(1, 1, 1)
	ax = plt.axes(xlim=(0, momdp.gridVar.shape[1]), ylim=(0, momdp.gridVar.shape[0]))

	# Major ticks every 20, minor ticks every 5
	major_ticks = np.arange(0, momdp.gridVar.shape[0], 1)
	ax.set_yticks(major_ticks)
	ax.set_yticks(major_ticks, minor=True)
	ax.grid(which='both')
	ax.grid(which='major', alpha=1.0)

	# Add goal
	goalColor  =(0.0, 0.7, 0.0)
	if momdp.unGoal == False:
		addStaticComponents(momdp, ax, 1, goalColor)
	else:
		goalPatchList = addDynamicComponent(momdp, ax, momdp.col_goal, momdp.row_goal, goalColor, bt)

	# Add known static obstacles
	obsColor  =(0.7, 0.2, 0.2)
	addStaticComponents(momdp, ax, -1, obsColor)

	# Add uncertain regions
	obsColor  =(0.7, 0.2, 0.2)
	obstPatchList = addDynamicComponent(momdp, ax, momdp.col_obs, momdp.row_obs, obsColor, bt)

	# Add agent: position will be updated in the time for loop
	patch = plt.Circle((0.5, -0.5), 0.25, fc='b')
	patch.center = (0, momdp.gridVar.shape[0])
	ax.add_patch(patch)

	# Time loop
	for k in range(0, len(xt)):
		# Update agent position (if the current postion is not the fictitious state of the product MOMDP)
		if  (momdp.unGoal == False) or (xt[k] not in momdp.goal):
			x, y = findxy(xt[k], momdp)
			patch.center = (x[0], y[0])

		# Update belief visualization
		updateDynamicComponents(obstPatchList, momdp, bt, k, momdp.row_obs, offset = 0)

		if momdp.unGoal == True:
			updateDynamicComponents(goalPatchList, momdp, bt, k, momdp.row_goal, offset = momdp.numObs)

		plt.draw()
		plt.pause(0.5)


def findxy(xt, momdp):
	idxY, idxX = np.where(momdp.stateMap == xt)
	x = idxX+0.5
	y = momdp.gridVar.shape[0] - idxY - 0.5	
	return x, y

def updateDynamicComponents(patchList, momdp, bt, k, row, offset = 0):
	for i in range(0, len(row) ):
		totProb = 0;
		for j in range(0, momdp.numO):
			totProb = totProb + (1-momdp.comb[j][offset + i])*bt[k][j];
		patchList[i].set_alpha(np.round(1-totProb, momdp.digitPrecision))

def addDynamicComponent(momdp, ax, col, row, colorComponent, bt):
	obstPatchList = []
	for i in range(0, len(row) ):
		x = col[i]
		y = momdp.gridVar.shape[0] - row[i] - 1
		patch = Rectangle((x, y), 1, 1, fc =colorComponent, ec =colorComponent)
		totProb = 0;
		for j in range(0, momdp.numO):
			totProb = totProb + (1-momdp.comb[j][i])*bt[0][j];
		patch.set_alpha(1-totProb)
		obstPatchList.append( patch )
		ax.add_patch(obstPatchList[-1])
	return obstPatchList

def addStaticComponents(momdp, ax, typeComponent, colorComponent):
	idxY, idxX = np.where(momdp.gridVar == typeComponent)
	for i in range(0, idxX.shape[0] ):
		x = idxX[i]
		y = momdp.gridVar.shape[0] - idxY[i] - 1
		ax.add_patch( Rectangle((x, y), 1, 1, fc =colorComponent, ec =colorComponent) ) 

def loadParameters(gridWorld, numObst, unGoal):
	if gridWorld == '15x15' or gridWorld == '10x10ug':
		totTimeSteps = 60
	elif gridWorld == '8x8ug':
		totTimeSteps = 50
	elif gridWorld == '7x7_d' or gridWorld == '7x7ug_d':
		totTimeSteps = 20
	else:
		totTimeSteps = 30
		# totTimeSteps = 40
	if unGoal == False:
		if numObst == 2:
			loc        = (0,0)
			initBelief = [0.9, 0.4] 
		elif numObst == 3:
			loc        = (1,1,0)
			initBelief = [0.9, 0.3, 0.4]
		elif numObst==4:
			if gridWorld == '5x5':
				loc        = (1,1,1,0)
			elif gridWorld == '10x5':
				loc        = (1,1,1,1)
			else:
				loc        = (0,1,1,1)
			initBelief = [0.9, 0.3, 0.4,0.5]
	else:
		if numObst == 2:
			loc        = (0,0,0,1)
			initBelief = [0.8, 0.3, 0.6,0.4]
		elif numObst == 3:
			loc        = (1,0,0,0,1)
			initBelief = [0.9, 0.3, 0.4, 0.1, 0.8]
		elif numObs == 4:
			loc        = (1,1,1,0,1)
			initBelief = [0.9, 0.3, 0.4, 0.9, 0.6]

	if gridWorld == '7x7ug_d':
		loc        = (0,0,0,1)
		initBelief = [0.8, 0.95, 0.90,0.05]
		
	return totTimeSteps, loc, initBelief

def eveluateMOMDP(momdp, loc, initBelief, xt, bt, printLevel):
	
	V_t0, Vrealized, J_t0, failure, tRun, xt, bt = simulateMOMDP(momdp, loc, initBelief, xt, bt, printLevel, 3) # Uncomment to get exact measurement
	# V_t0, Vrealized, J_t0, failure, tRun, xt, bt = simulateMOMDP(momdp, loc, initBelief, xt, bt, 0, 3)
	# _, _, _, _, _, xt_1, _ = simulateMOMDP(momdp, loc, initBelief, xt, bt, 0, 0)
	# _, _, _, _, _, xt_2, _ = simulateMOMDP(momdp, loc, initBelief, xt, bt, 0, 1)
	# if xt_1 == xt and xt_2 == xt:
	# 	if momdp.printLevel >= 1: print('Expected value computed correctly')
	# else:
	# 	print('Need to consider all possible combination of possible observations')
	# 	pdb.set_trace()

	return V_t0, Vrealized, J_t0, failure, tRun, xt, bt

def simulateMOMDP(momdp, loc, initBelief, xt, bt, printLevel, obstOpt):
	momdp.printLevel = printLevel
	
	momdp.initZ(loc)

	V_t0 = np.max(np.dot(momdp.V[0][0].T, bt[0]))
	J_t0 = np.max(np.dot(momdp.J[0][0].T, bt[0]))

	if momdp.printLevel >=1 : print("====== Simulation Time Loop ======")
	at = []
	t = 0
	if momdp.policy == 'TO':
		spec = V_t0
	else:
		spec = J_t0

	while (xt[-1] != momdp.goal and spec > 0):
		[action, spec, cost] = momdp.evaluatePolicy(t, xt[-1], bt[-1])
		at.append(action)

		xt.append(momdp.propagate(xt[-1], momdp.zt, at[-1]))
		oMeas = momdp.getObservation(xt[-1], obstOpt)
		bt.append( momdp.updateBelief(xt[-2], xt[-1], at[-1], oMeas, bt[-1]) )
		t += 1

	if momdp.printLevel >= 1: print(momdp.stateMap)
	if momdp.printLevel >= 1: print('MOMDP trajectory: ', xt)

	tRun = float(t)
	failure = int(xt[-1] != momdp.goal)
	if failure == 0:
		Vrealized =  len(momdp.V) - tRun
		if momdp.printLevel >=1: print("====== End Simulation: Goal Reached in ", tRun, " time steps! The realized cost is: ", Vrealized)
	else:
		Vrealized = 0
		if momdp.printLevel >=1: print("====== End Simulation: Failed to reach the goal")
	
	return V_t0, Vrealized, J_t0, failure, tRun, xt, bt



def loadGrid(gridVarOpt):
	# This file load a gird world which is deinfed by the variable gridVar.
	# Given a cell positioned at (i,j) in the grid world we have:
	# - gridVar[i,i] = 0   if the (i,j) position is free space
	# - gridVar[i,i] = 0.5 if the (i,j) position is an uncertain region
	# - gridVar[i,i] = 1   if the (i,j) position is the goal
	
	print("gridVarOpt: ", gridVarOpt)
	if (gridVarOpt == '5x5ug_2'):
		gridVar  = np.array([[0,        0,         0,         0,    1.0000],
							[0,         0,   -1.0000,    0.5000,   -1.0000],
							[0,         0,   -1.0000,         0,         0],
							[0,         0,   -1.0000,   -1.0000,    0.5000],
							[0,         0,   -1.0000,         0,    1.0000]])

	elif (gridVarOpt == '5x5ug_3'):
		gridVar  = np.array([[0,        0,         0,         0,         0],
							[0,         0,   -1.0000,    0.5000,    0.5000],
							[0,         0,   -1.0000,    1.0000,         0],
							[0,         0,   -1.0000,         0,         0],
							[0,         0,    0.5000,         0,    1.0000]])
	elif (gridVarOpt == '5x5_3'):
		gridVar  = np.array([[0,        0,         0,         0,         0],
							[0,         0,   -1.0000,    0.5000,    0.5000],
							[0,         0,   -1.0000,         0,         0],
							[0,         0,   -1.0000,         0,         0],
							[0,         0,    0.5000,         0,    1.0000]])

	elif (gridVarOpt == '5x5_4'):
		gridVar  = np.array([[0,         0,         0,         0,         0],
							[0,         0,   -1.0000,    0.5000,    0.5000],
							[0,         0,    0.5000,         0,         0],
							[0,         0,   -1.0000,         0,         0],
							[0,         0,    0.5000,         0,    1.0000]])
	elif (gridVarOpt == '10x5_3'):
		gridVar  = np.array([[0,         0,         0,    0.5000,    1.0000],
							[0,   -1.0000,         0,   -1.0000,         0],
							[0,   -1.0000,         0,   -1.0000,         0],
							[0,   -1.0000,         0,    0.5000,         0],
							[0,   -1.0000,         0,    0.5000,         0],
							[0,   -1.0000,         0,   -1.0000,         0],
							[0,   -1.0000,         0,   -1.0000,         0],
							[0,   -1.0000,         0,   -1.0000,         0],
							[0,   -1.0000,   -1.0000,   -1.0000,         0],
							[0,         0,         0,         0,         0]])
	
	elif (gridVarOpt == '7x7ug_d_2'):
		gridVar =np.array([[1.0,        0,         0,    0.5000,         0,         0,         0],
							[0,         0,         0,         0,         0,         0,         0],
							[0,         0,         0,         0,         0,         0,         0],
							[0,         0,         0,         0,         0,         0,    0.5000],
							[0,         0,         0,         0,         0,         0,         0],
							[0,         0,         0,    0.5000,         0,         0,         0],
							[0,         0,         0,         0,         0,         0,    0.5000],]);
	
	elif (gridVarOpt == '7x7ug_2'):
		gridVar =np.array([[0,         0,   -1.0000,    1.0000,         0,         0,         0],
						   [0,         0,   -1.0000,         0,         0,         0,         0],
						   [0,         0,   -1.0000,         0,         0,         0,         0],
						   [0,         0,   -1.0000,         0,   -1.0000,   -1.0000,    0.5000],
						   [0,         0,   -1.0000,         0,   -1.0000,         0,         0],
						   [0,         0,   -1.0000,    0.5000,   -1.0000,         0,         0],
						   [0,         0,         0,         0,   -1.0000,         0,    1.0000],]);

	elif (gridVarOpt == '7x7_2'):
		gridVar =np.array([[0,         0,   -1.0000,         0,         0,         0],
         				   [0,         0,   -1.0000,         0,         0,         0],
         				   [0,         0,   -1.0000,         0,         0,         0],
         				   [0,         0,   -1.0000,         0,   -1.0000,    0.5000],
         				   [0,         0,   -1.0000,    0.5000,   -1.0000,         0],
         				   [0,         0,         0,         0,   -1.0000,    1.0000],]);
	
	elif (gridVarOpt == '7x7ug_2'):
		gridVar =np.array([[0,         0,   -1.0000,         0,         0,         0,         0],
         				   [0,         0,   -1.0000,         0,         0,         0,         0],
         				   [0,         0,   -1.0000,         0,         0,         0,         0],
         				   [0,    1.0000,   -1.0000,         0,   -1.0000,   -1.0000,    0.5000],
         				   [0,         0,   -1.0000,         0,   -1.0000,         0,         0],
         				   [0,         0,   -1.0000,    0.5000,   -1.0000,         0,         0],
         				   [0,         0,         0,         0,   -1.0000,         0,    1.0000],]);

	elif (gridVarOpt == '7x7_d_2'):
		gridVar =np.array([[1.0,        0,         0,         0,         0,         0],
         					[0,         0,         0,         0,         0,         0],
         					[0,         0,         0,         0,         0,         0],
         					[0,         0,         0,         0,         0,    0.5000],
         					[0,         0,         0,    0.5000,         0,         0],
         					[0,         0,         0,         0,         0,         0],]);
	elif (gridVarOpt == '7x7ug_d_2'):
		gridVar =np.array([[1.0,        0,         0,         0,         0,         0,         0],
         					[0,         0,         0,         0,         0,         0,         0],
         					[0,         0,         0,         0,         0,         0,         0],
         					[0,    0.5000,         0,         0,         0,         0,    0.5000],
         					[0,         0,         0,         0,         0,         0,         0],
         					[0,         0,         0,    0.5000,         0,         0,         0],
         					[0,         0,         0,         0,         0,         0,    0.5000],]);

	elif (gridVarOpt == '10x5_4'):
		gridVar  = np.array([[0,         0,         0,    0.5000,    1.0000],
							[0,   -1.0000,         0,   -1.0000,         0],
							[0,   -1.0000,         0,   -1.0000,         0],
							[0,   -1.0000,         0,    0.5000,         0],
							[0,   -1.0000,         0,    0.5000,         0],
							[0,   -1.0000,         0,    0.5000,         0],
							[0,   -1.0000,         0,   -1.0000,         0],
							[0,   -1.0000,         0,   -1.0000,         0],
							[0,   -1.0000,   -1.0000,   -1.0000,         0],
							[0,         0,         0,         0,         0]])

	elif (gridVarOpt == '15x15_3'):
		gridVar =np.array([[0,         0,         0,         0,         0,         0,         0,   -1.0000,         0,         0,         0,         0,         0,        0,         0],
					       [0,         0,         0,         0,         0,         0,         0,   -1.0000,         0,         0,         0,         0,         0,        0,         0],
					       [0,         0,         0,         0,         0,         0,         0,   -1.0000,         0,         0,         0,         0,         0,        0,         0],
					       [0,         0,         0,         0,         0,         0,         0,   -1.0000,         0,         0,         0,         0,         0,        0,         0],
					       [0,         0,         0,         0,         0,         0,         0,    0.5000,         0,         0,         0,         0,         0,        0,         0],
					       [0,         0,   -1.0000,   -1.0000,   -1.0000,         0,         0,   -1.0000,         0,         0,         0,         0,         0,        0,         0],
					       [0,         0,   -1.0000,   -1.0000,   -1.0000,         0,         0,   -1.0000,         0,         0,         0,         0,         0,        0,         0],
					       [0,         0,   -1.0000,   -1.0000,   -1.0000,         0,         0,   -1.0000,         0,         0,         0,         0,         0,        0,         0],
					       [0,         0,   -1.0000,   -1.0000,   -1.0000,    0.5000,   -1.0000,   -1.0000,         0,         0,         0,         0,         0,        0,         0],
					       [0,         0,   -1.0000,   -1.0000,         0,         0,   -1.0000,   -1.0000,         0,         0,         0,         0,         0,        0,         0],
					       [0,         0,   -1.0000,   -1.0000,         0,         0,         0,         0,         0,         0,         0,         0,         0,        0,         0],
					       [0,         0,         0,   -1.0000,         0,         0,         0,         0,         0,         0,         0,         0,         0,        0,         0],
					       [0,         0,         0,   -1.0000,         0,         0,         0,         0,         0,         0,         0,         0,         0,        0,         0],
					       [0,         0,         0,    0.5000,         0,         0,         0,         0,         0,         0,         0,         0,         0,        0,         0],
					       [0,         0,         0,   -1.0000,         0,         0,         0,         0,         0,         0,         0,         0,         0,        0,    1.0000]])

	elif (gridVarOpt == '15x15_4'):
		gridVar =np.array([[0,         0,         0,         0,         0,         0,         0,   -1.0000,         0,         0,         0,         0,         0,        0,         0],
					[0,         0,         0,         0,         0,         0,         0,   -1.0000,         0,         0,         0,         0,         0,        0,         0],
					[0,         0,         0,         0,         0,         0,         0,   -1.0000,         0,         0,         0,         0,         0,        0,         0],
					[0,         0,         0,         0,         0,         0,         0,    0.5000,         0,         0,         0,         0,         0,        0,         0],
					[0,         0,         0,         0,         0,         0,         0,    0.5000,         0,         0,         0,         0,         0,        0,         0],
					[0,         0,   -1.0000,   -1.0000,   -1.0000,         0,         0,   -1.0000,         0,         0,         0,         0,         0,        0,         0],
					[0,         0,   -1.0000,   -1.0000,   -1.0000,         0,         0,   -1.0000,         0,         0,         0,         0,         0,        0,         0],
					[0,         0,   -1.0000,   -1.0000,   -1.0000,         0,         0,   -1.0000,         0,         0,         0,         0,         0,        0,         0],
					[0,         0,   -1.0000,   -1.0000,   -1.0000,    0.5000,   -1.0000,   -1.0000,         0,         0,         0,         0,         0,        0,         0],
					[0,         0,   -1.0000,   -1.0000,         0,         0,   -1.0000,   -1.0000,         0,         0,         0,         0,         0,        0,         0],
					[0,         0,   -1.0000,   -1.0000,         0,         0,         0,         0,         0,         0,         0,         0,         0,        0,         0],
					[0,         0,         0,   -1.0000,         0,         0,         0,         0,         0,         0,         0,         0,         0,        0,         0],
					[0,         0,         0,   -1.0000,         0,         0,         0,         0,         0,         0,         0,         0,         0,        0,         0],
					[0,         0,         0,    0.5000,         0,         0,         0,         0,         0,         0,         0,         0,         0,        0,         0],
					[0,         0,         0,   -1.0000,         0,         0,         0,         0,         0,         0,         0,         0,         0,        0,    1.0000]])
	
	elif (gridVarOpt == '10x10ug_3'):
		gridVar =np.array([[0,         0,         0,         0,         0,         0,         0,       0,       0,       0],
						   [0,         0,   -1.0000,   -1.0000,   -1.0000,   -1.0000,   -1.0000, -1.0000, -1.0000,  0.5000],
						   [0,         0,   -1.0000,         0,    1.0000,         0,         0,       0,       0,       0],
						   [0,         0,   -1.0000,         0,         0,         0,         0,       0,       0,       0],
						   [0,         0,   -1.0000,         0,   -1.0000,   -1.0000,    0.5000, -1.0000, -1.0000, -1.0000],
						   [0,         0,   -1.0000,         0,   -1.0000,         0,         0,       0,       0,       0],
						   [0,         0,   -1.0000,    0.5000,   -1.0000,         0,         0,       0,       0,       0],
						   [0,         0,         0,         0,   -1.0000,         0,         0,       0,       0,       0],
						   [0,         0,         0,         0,   -1.0000,         0,         0,       0,       0,       0],
						   [0,         0,         0,         0,   -1.0000,         0,         0,       0,       0,   1.000]]);

	elif (gridVarOpt == '10x10ug_2'):
		gridVar =np.array([[0,         0,         0,         0,         0,         0,         0,       0,       0,       0],
						   [0,         0,   -1.0000,   -1.0000,   -1.0000,   -1.0000,   -1.0000, -1.0000, -1.0000,  0.5000],
						   [0,         0,   -1.0000,         0,    1.0000,         0,         0,       0,       0,       0],
						   [0,         0,   -1.0000,         0,         0,         0,         0,       0,       0,       0],
						   [0,         0,   -1.0000,         0,   -1.0000,   -1.0000,    0.0000, -1.0000, -1.0000, -1.0000],
						   [0,         0,   -1.0000,         0,   -1.0000,         0,         0,       0,       0,       0],
						   [0,         0,   -1.0000,    0.5000,   -1.0000,         0,         0,       0,       0,       0],
						   [0,         0,         0,         0,   -1.0000,         0,         0,       0,       0,       0],
						   [0,         0,         0,         0,   -1.0000,         0,         0,       0,       0,       0],
						   [0,         0,         0,         0,   -1.0000,         0,         0,       0,       0,   1.000]]);
	
	elif (gridVarOpt == '8x8ug_2'):
		gridVar =np.array([[      0,         0,         0,         0,         0,       0,       0,       0],
						   [-1.0000,         0,         0,         0,         0,       0,       0,       0],
						   [      0,         0,   -1.0000,   -1.0000,   -1.0000, -1.0000, -1.0000,  0.5000],
						   [      0,   -1.0000,   -1.0000,         0,         0, -1.0000,       0,       0],
						   [      0,         0,   -1.0000,         0,    1.0000, -1.0000,       0,       0],
						   [-1.0000,    0.5000,   -1.0000,         0,         0, -1.0000,       0,       0],
						   [      0,         0,   -1.0000,         0,   -1.0000, -1.0000,       0,       0],
						   [ 1.0000,         0,   -1.0000,         0,         0,       0,       0,       0]]);
	
	print(gridVarOpt)
	return gridVar