import numpy as np
import pdb
import itertools
import time
import multiprocessing
import random
from multiprocessing.pool import ThreadPool

class MOMDP(object):
	"""docstring for MOMDP"""
	def __init__(self, gridVar, totTimeSteps, printLevel, policy, discOpt, unGoal = False, valFunFlag = True):
		self.policy         = policy
		self.printLevel     = printLevel
		self.digitPrecision = 14
		self.toll           = 10**(-self.digitPrecision)
		self.unGoal         = unGoal
		self.cellWidth      = 1.0
		self.cellHight      = 1.0

		self.gridVar 		= gridVar
		self.row     		= np.shape(self.gridVar)[0]
		self.col     		= np.shape(self.gridVar)[1]
		self.numUnKnownObst = sum(sum(gridVar==0.5))
		self.numKnownObst   = sum(sum(gridVar==-1))

		self.numA = 4
		self.numO = 2**( sum(sum(gridVar==0.5)) + sum(sum(gridVar==1.0))*unGoal )
		self.numZ = self.numO
		self.numS = self.row*self.col - sum(sum(gridVar==-1)) + 1*unGoal # Add fictitious goal state if goal regions are uncertain: unGoal = True
		
		self.stateMap  = self.stateMap(gridVar)

		self.obsLoc = self.locateUR(0.5)
		self.row_obs, self.col_obs   = self.findRowCol(self.obsLoc)

		if unGoal == False:
			self.goal     = self.stateMap[np.where(gridVar == 1)].tolist()
			self.numUGoal = 0 # set number of uncertain goal = 0
		else:
			self.goal     = [self.numS-1]
			self.goalLoc  = self.locateUR(1.0)
			self.row_goal, self.col_goal = self.findRowCol(self.goalLoc)
			self.numUGoal = np.shape(self.goalLoc)[0]

		self.numObs = np.shape(self.obsLoc)[0]
		self.numUR  = self.numObs + self.numUGoal
		self.combWrongOrder = list(itertools.product([0, 1], repeat=self.numUR))
		self.comb = []
		for i in range(0, len(self.combWrongOrder)):
			self.comb.append(self.combWrongOrder[i][::-1])	

		self.actVec = np.array(['n','s','w','e']);

		# Initialize vectors for computing successor states in the obstacle free settings
		self.add_col = []
		self.add_row = []
		
		# North action
		self.add_col.append(0)
		self.add_row.append(-1)

		# South action
		self.add_col.append(0)
		self.add_row.append(+1)

		# West action
		self.add_col.append(-1)
		self.add_row.append(0)

		# East action
		self.add_col.append(+1)
		self.add_row.append(0)

		# Compute transition matrices
		self.Px_list = []
		self.Pe_list = []
		self.Oe_list = []
		for a in range(0, self.numA):
			Px_action = []
			Pe_action = []
			for e in range(0, self.numO):
				activeObstacles = []
				activeGoals     = []
				for i in range(0, self.numObs):
					if self.comb[e][i] == 1:
						activeObstacles.append(self.obsLoc[i])
				for i in range(0, self.numUGoal):
					if self.comb[e][self.numObs + i] == 1: # first self.numObs entries are related with the obstacles
						activeGoals.append(self.goalLoc[i])
				Px_action.append( self.computeTs(a, activeObstacles, activeGoals) )
				Pe_action.append( self.computeTe(e) )
				
			self.Px_list.append(Px_action)
			self.Pe_list.append(Pe_action)

		for a in range(0, self.numA):
			Oe_action = []
			for e in range(0, self.numO):
				Oe_action.append( self.computeO(self.Px_list[a][e], a, e) )
			self.Oe_list.append(Oe_action)

		# Compute M matrix TO DO: add description
		self.M = [[[[np.zeros((self.numO, self.numO)) for sub in range(self.numO)] for sub in range(self.numS)] for sub in range(self.numS)] for sub in range(self.numA)] 

		# Initialize matrix to zero
		for a in range(0, self.numA):
			for x in range(0, self.numS):
				for xnext in self.sucessorStates(x, a):
					for o  in range(0, self.numO):
						for i in range(0, self.numO):
							for j in range(0, self.numO):
								self.M[a][x][xnext][o][i, j] =  self.Oe_list[a][i][o,xnext] * self.Px_list[a][j][x,xnext] * self.Pe_list[a][j][x,i]

		self.sucessorStatesList = [[self.sucessorStates(xt, a) for a in range(self.numA)] for xt in range(self.numS)]
		
		# Initialize Belief
		self.initPointBased(discOpt)

		self.timeSteps = totTimeSteps
		self.V = [[np.zeros((self.numO, np.shape(self.Belief)[1])) for sub in range(self.numS)] for sub in range(self.timeSteps)]
		self.J = [[np.zeros((self.numO, np.shape(self.Belief)[1])) for sub in range(self.numS)] for sub in range(self.timeSteps)]
		
		for goal in self.goal:
			self.V[self.timeSteps-1][goal] = np.ones((self.numO, np.shape(self.Belief)[1]))
			self.J[self.timeSteps-1][goal] = np.ones((self.numO, np.shape(self.Belief)[1]))

		self.V =  np.array(self.V)
		self.J =  np.array(self.J)
		self.M =  np.array(self.M)

		if valFunFlag == True:
			print("Start Value Function Approximation")
			self.computeValFun()
			print("Done Value Function Approximation")

	def computeValFun(self):
		self.totTime = 0
		self.avgBackupTime = 0
		self.avgLoopBackupTime = 0
		for i in range(1, self.timeSteps): # this for loop cannot be parallelized 
			backUpTime = 0
			totTime_start = time.time()
			self.t = self.timeSteps - i - 1
			for s in range(0, self.numS): # this for loop can be parallelized
				t_start = time.time()
				self.backupLoopBeliefPoint(s)
				backUpTime += time.time() - t_start
			self.totTime += time.time() - totTime_start			
		
			self.avgBackupTime += backUpTime/(self.numS*self.numBeliefPoints)
			self.avgLoopBackupTime += backUpTime/(self.numS)
			print("Updated step: ", self.t, ". Backup time: ", backUpTime/(self.numS*self.numBeliefPoints), " Update at state s time: ", backUpTime/self.numS )

		print("Total time: ", self.totTime)
		self.avgBackupTime = self.avgBackupTime/(self.timeSteps-1)
		print("Avarage backup time: ", self.avgBackupTime)

	def findRowCol(self, locations):
		row = []
		col = []
		for i in range(0, len(locations)):
			r,c = np.where(self.stateMap == locations[i])
			row.append(r[0]); 
			col.append(c[0])
		return row, col

	def locateUR(self, regionType):
		# TO DO: add comment why this order and what are region types
		stateLocation = []
		for i in range(0, self.col):
			for j in range(0, self.row):
				if self.gridVar[j,i] == regionType: 
					stateLocation.append(self.stateMap[j,i])
		return stateLocation

	def computeO(self, Ts, a, e):
		# M1 used when adiacent to unknown obstacle
		M1 = np.array([[1.0, 0.0],
					   [0.0, 1.0]])

		# M2 used when on the diagonal
		M2 = np.array([[0.8, 0.2],
					   [0.2, 0.8]])

		# M3 used in all other cases
		M3 = np.array([[0.5, 0.5],
					   [0.5, 0.5]])

		O = np.zeros((self.numO, self.numS))
		if self.unGoal == True:
			O[:, self.numS-1] = np.ones(self.numO)/self.numO # if you are the the goal all observations are equality likely
		
		# loop over all grid points
		for i in range(0, self.row):
			for j in range(0, self.col):
				state = np.zeros( (self.numS, 1) )
				if (self.stateMap[i,j]>=0): # Check if stateMap(i,j) is a state
					state[self.stateMap[i,j], 0] = 1
					stateNext = np.dot(Ts.T, state)

					# Here we want to compute P(y_{k+1} | x_{k+1}, u_k). So first
					# we have to check if it is possible to be at x_{k+1} given
					# that we have applied u_k. If it is not possible, the
					# observing P(y_{k+1} | x_{k+1}, u_k) = 0. So, we are going to
					# check if the pre state is feasible: preFeasible
					if np.array_equal(state, stateNext): # check if applying the current action we stay still
						preFeasible = 1;
					else: # if applying the current action we are not still --> check pre state
						preState_row = i-self.add_row[a]
						preState_col = j-self.add_col[a]
						if (0 <= preState_row) and (preState_row < self.row) and (0 <= preState_col) and (preState_col < self.col): # check if pre state is in the grid
							preFeasible  = self.stateMap[preState_row, preState_col] >= 0 # check is pre state is not a known obstacle
						else:
							preFeasible = 0 # if outside the grid --> not possible to be at x_{k+1} after applying u_k

					# the variable preFeasible tells us if we could be at x_{k+1} after applying u_k 
					P_observe = 1 
					for ii in range(0, self.numObs):
						dist = np.abs(i-self.row_obs[ii]) + np.abs(j-self.col_obs[ii]) # compute distance from obstacle number ii
						
						# compute join probability 
						if dist <= 1:
							P_observe = np.kron(M1,P_observe)*preFeasible
						elif (dist == 2) and (np.abs(i-self.row_obs[ii]) == 1):
							P_observe = np.kron(M2,P_observe)*preFeasible
						else:
							P_observe = np.kron(M3,P_observe)*preFeasible

					for ii in range(0, self.numUGoal):
						dist = np.abs(i-self.row_goal[ii]) + np.abs(j-self.col_goal[ii]) # compute distance from obstacle number ii
						# compute join probability 
						if dist <= 0:
							P_observe = np.kron(M1,P_observe)*preFeasible
						elif (dist == 1):
							P_observe = np.kron(M2,P_observe)*preFeasible
						else:
							P_observe = np.kron(M3,P_observe)*preFeasible

					O[:, self.stateMap[i,j]] = P_observe[:, e]
		
		return O

	def computeTe(self, e):
		unobservableStateVector = np.zeros( (self.numO,1) )
		unobservableStateVector[e,0] = 1
		Te = np.ones((self.numS, 1))*unobservableStateVector.T
		return Te

	def computeTs(self, a , activeObstacles, activeGoals):
		T = np.zeros( (self.numS, self.numS) )
		
		if self.unGoal == True:
			T[self.numS-1, self.numS-1] = 1 # if you are at a fictitious goal state --> stay there

		for i in range(0, self.row):
			for j in range(0, self.col):
				if self.stateMap[i,j] >= 0:
					# compute successor
					nextState_row = max(0, min(i + self.add_row[a], self.row-1))
					nextState_col = max(0, min(j + self.add_col[a], self.col-1))

					if (self.stateMap[i,j] in activeGoals):
						T[self.stateMap[i,j], self.numS-1] = 1 # if you are at an active goal --> transition to fictitious goal state
					elif ( (self.gridVar[i,j] == 1) and (self.unGoal == False) ) or (self.gridVar[i,j] == -1) or (self.stateMap[i,j] in activeObstacles) or (self.stateMap[nextState_row,nextState_col] == -1):
						T[self.stateMap[i,j], self.stateMap[i,j]] = 1
					else:
						T[self.stateMap[i,j], self.stateMap[nextState_row,nextState_col]] = 1
		return T.astype(int)
	

	def sucessorStates(self, xt, at):
		nextStatetList = [self.propagate(xt, z, at) for z in range(0, self.numZ)]
		return list( dict.fromkeys(nextStatetList) )

	def propagate(self, xt, zt, at):
		return np.argmax(np.dot(self.Px_list[at][zt].T, self.stateToVector(xt)))
	
	def stateToVector(self, xt):
		xtVec	 = np.zeros(self.numS)
		xtVec[xt] = 1
		return xtVec
		
	def stateMap(self, gridVar):
		row = gridVar.shape[0]
		col = gridVar.shape[1]
		stateMap = -1*np.ones((row, col)) # Basically stateMap is a matrix which contains the numbered states
		obsStateCounter = 0 # Number of grid observable states (no known obstables)
		for i in range(0,row):
			for j in range(0, col):
				if gridVar[i,j]>=0:
					stateMap[i,j] = obsStateCounter
					obsStateCounter = obsStateCounter + 1

		if self.printLevel >= 1: print("State Map: ")
		if self.printLevel >= 1: print(stateMap)
		return stateMap.astype(int)

	def initPointBased(self, discOpt):
		# Several options for discretizing the belief space are provided.
		# Set discOpt = 1 for the results from table 1
		# Set discOpt = 2 for the results from table 2
		# Options discOpt = 3 and discOpt = 4 lead to the same results as discOpt = 2.
		
		if discOpt == 1:
			self.obstBelief = np.eye(self.numUR)
			self.obstBelief = np.concatenate( (self.obstBelief, -np.eye(self.numUR)+np.ones((self.numUR,self.numUR)) ) , axis=0)
			self.obstBelief = np.concatenate( (self.obstBelief, 0.5*(-np.eye(self.numUR)+np.ones((self.numUR,self.numUR))) ) , axis=0)

			self.obstBelief = np.concatenate( (self.obstBelief, np.zeros((1,self.numUR))) , axis=0)
			self.obstBelief = np.concatenate( (self.obstBelief, np.ones((1,self.numUR))) , axis=0)
			self.obstBelief = np.concatenate( (self.obstBelief, 0.5*np.ones((1,self.numUR))) , axis=0)
			self.obstBelief = np.concatenate( (self.obstBelief, 0.2*np.ones((1,self.numUR))) , axis=0)
			self.obstBelief = np.concatenate( (self.obstBelief, 0.9*np.ones((1,self.numUR))) , axis=0)
		
		elif discOpt == 2: 
			totObst = self.numUR
			dim = np.linspace(0.0, 1.0, 4) 
			  
			x = (dim, )
			for i in range(1, totObst):
				x = x + (dim, )

			self.obstBelief  = np.vstack( np.meshgrid(*x) ).reshape(totObst, -1).T
		
		elif discOpt == 3: 
			self.obstBelief = np.array([self.comb[0]])
			for i in range(1, len(self.comb)):
				toAdd = np.array([self.comb[i]]).astype(float)
				toAdd[toAdd > 0.5] = 1.0
				toAdd[toAdd < 0.5] = 0.0
				self.obstBelief = np.concatenate( (self.obstBelief, toAdd ) , axis=0)
				
			for i in range(0, len(self.comb)):
					toAdd = np.array([self.comb[i]]).astype(float)
					if np.sum(toAdd) > 0:
						toAdd[toAdd > 0.5] = 0.5
						self.obstBelief = np.concatenate( (self.obstBelief, toAdd) , axis=0)

			for i in range(0, len(self.comb)):
					toAdd = np.array([self.comb[i]]).astype(float)
					if np.sum(toAdd) > 0:
						toAdd[toAdd < 0.5] = 0.5
						self.obstBelief = np.concatenate( (self.obstBelief, toAdd) , axis=0)

		elif discOpt == 4:
			self.obstBelief = np.array([self.comb[0]])
			for i in range(1, len(self.comb)):
				self.obstBelief = np.concatenate( (self.obstBelief, np.array([self.comb[i]])) , axis=0)

			
			if self.unGoal == True:
				for i in range(0, len(self.comb)):
						toAdd = np.array([self.comb[i]]).astype(float)
						for j in range(0, self.numObs):
							if toAdd[0,j] > 0.5: toAdd[0,j] = 0.9
							if toAdd[0,j] < 0.5: toAdd[0,j] = 0.7
						
						for j in range(0, self.numUGoal):
							if toAdd[0,j+self.numObs] < 0.5: toAdd[0,j+self.numObs] = 0.9
							if toAdd[0,j+self.numObs] > 0.5: toAdd[0,j+self.numObs] = 0.7				
						self.obstBelief = np.concatenate( (self.obstBelief, toAdd) , axis=0)

						toAdd = np.array([self.comb[i]]).astype(float)
						for j in range(0, self.numObs):
							if toAdd[0,j] > 0.5: toAdd[0,j] = 0.9
							if toAdd[0,j] < 0.5: toAdd[0,j] = 0.1
						
						for j in range(0, self.numUGoal):
							if toAdd[0,j+self.numObs] < 0.5: toAdd[0,j+self.numObs] = 0.9
							if toAdd[0,j+self.numObs] > 0.5: toAdd[0,j+self.numObs] = 0.1				
						self.obstBelief = np.concatenate( (self.obstBelief, toAdd) , axis=0)

			else:
				for i in range(0, len(self.comb)):
					toAdd = np.array([self.comb[i]]).astype(float)
					toAdd[toAdd > 0.5] = 0.9
					toAdd[toAdd < 0.5] = 0.1
					self.obstBelief = np.concatenate( (self.obstBelief, toAdd) , axis=0)


		# From individual probability to belief
		self.Belief = np.zeros((self.numO, self.obstBelief.shape[0]))
		for i in range(0, self.obstBelief.shape[0]):
			self.Belief[:,i] = self.initBelief(self.obstBelief[i,:])
		
		self.numBeliefPoints = self.obstBelief.shape[0]

		print("Belif points: ", self.numBeliefPoints)

	def initBelief(self, probInit):
		InitialBelief = []	
		# Loop to assign prob
		for i in range(0, len(self.comb)):
			prob = 1
			for j in range(0, len(probInit)):
				prob = prob * ( (1-self.comb[i][j]) * probInit[j] + self.comb[i][j] * (1 - probInit[j]) )
			InitialBelief.append(prob)

		return np.array(InitialBelief)
	
	def initZ(self, loc):
		self.loc = loc
		for i in range(0,len(self.comb)):
			if self.comb[i]==loc:
				self.zt = i
				if self.printLevel >= 2: print("==== Current state: ", self.zt, ". Location: ", loc)

	def evaluateCost(self, t, xt, bt):		
		return np.max(np.dot(self.V[t][xt].T, bt))

	def updateBelief(self, xt, xNext, at, ot, bt):
		return np.dot( self.M[at][xt][xNext][ot], bt )/np.sum(np.dot( self.M[at][xt][xNext][ot], bt ))

	def getObservation(self, xt, obstOpt):
		# obstacle observation options:
		# - obstOpt = 0 ---> always measure 0 unless perfect measurement
		# - obstOpt = 1 ---> always measure 1 unless perfect measurement
		# - obstOpt = 2 ---> perfect measurement
		# - obstOpt = 3 ---> random unless perfect measurement
		
		if obstOpt == 2:
			meas = self.zt
		else:
			locMeas = list(self.loc)
			row, col = np.where(xt == self.stateMap)
			for i in range(0, self.numObs):
				dist = np.abs(row-self.row_obs[i]) + np.abs(col-self.col_obs[i]) # compute distance from obstacle number ii
				if dist >= 2: # if dist >= 2 --> random measurment otherwise exact one
					if obstOpt < 2: 
						locMeas[i] = obstOpt
					else:
						locMeas[i] = np.round(random.uniform(0, 1))
	
			if self.unGoal == True:
				for i in range(0, self.numUGoal):
					dist = np.abs(row-self.row_goal[i]) + np.abs(col-self.col_goal[i]) # compute distance from obstacle number ii
					if dist >= 1: # if dist >= 1 --> random measurment otherwise exact one
						if obstOpt < 2:
							locMeas[self.numObs + i] = obstOpt
						else:
							locMeas[self.numObs + i] = np.round(random.uniform(0, 1))

			locMeas = tuple(locMeas)
			for i in range(0,len(self.comb)):
				if self.comb[i]==locMeas:
					meas = i
		return meas

	def getCoordinates(self, xt):
		idx = np.where(self.stateMap==xt)
		x   = idx[1] + self.cellWidth/2.0
		y   = np.shape(self.gridVar)[0] - idx[0] - self.cellHight/2.0
		
		return (x[0], y[0])

	def getBoxCurren(self, x1):
		xCoor = self.getCoordinates(x1)
		xBox = [xCoor[0] + width for width in [self.cellWidth/2.0, -self.cellWidth/2.0]]
		yBox = [xCoor[1] + width for width in [self.cellWidth/2.0, -self.cellWidth/2.0]]
		return (min(xBox), max(xBox), min(yBox), max(yBox))

	def getBoxConstr(self, x1, x2):
		xCoor = [self.getCoordinates(x) for x in [x1, x2]]
		xBox = [xy[0] + width for xy in xCoor for width in [self.cellWidth/2.0, -self.cellWidth/2.0]]
		yBox = [xy[1] + width for xy in xCoor for width in [self.cellWidth/2.0, -self.cellWidth/2.0]]
		return (min(xBox), max(xBox), min(yBox), max(yBox))

	def approxValueFunction(self, t, xCurr, bCurr):
		coordXY = []
		xCurLst = []
		for i in range(0, min(3, self.V.shape[0] - t - 1)):
			if xCurr == self.goal and coordXY != [] and sum(sum(self.gridVar==1.0)) > 1:
				coordXY.append(coordXY[-1])
			else:
				coordXY.append(self.getCoordinates(xCurr))
			xCurLst.append(xCurr)
			[at, spec, cost] = self.evaluatePolicy(t+i, xCurr, bCurr)
			xCurr = self.propagate(xCurr, 0, at)

		return coordXY, xCurLst

	def updateMOMDP(self, t, xt, bt):
		coordXY, xCurLst = self.approxValueFunction(t, xt, bt)
		[action, spec, cost] = self.evaluatePolicy(t, xt, bt)
		boxConstraints = self.getBoxConstr(xCurLst[0], xCurLst[1])
		boxNext        = self.getBoxCurren(xCurLst[1])

		return action, coordXY, boxConstraints, boxNext

	def computeBelief(self, bt, idx):
		prob = 0
		for i in range(0, len(self.comb)):
			prob = prob + ( (1-self.comb[i][idx]) * bt[i] ) #+ comb[i][j] * (1 - bt[j]) )
		return prob

class MOMDP_TOQ(MOMDP):

	def __init__(self, gridVar, totTimeSteps, printLevel, policy, discOpt, unGoal = False):  
		# super().__init__(gridVar, totTimeSteps, printLevel, policy, discOpt, unGoal) # uncomment for py3
		super(MOMDP_TOQ, self).__init__(gridVar, totTimeSteps, printLevel, policy, discOpt, unGoal) # uncomment for py2

	def  backupLoopBeliefPoint(self, xt):
		J_out	= np.zeros((np.shape(self.Belief)[0],np.shape(self.Belief)[1]))
		V_out	= np.zeros((np.shape(self.Belief)[0],np.shape(self.Belief)[1]))
						
		
		for j in range(0, np.shape(self.Belief)[1]): # loop over belief points (this for loop can be parallelized)
			# for each belief point we are going to compute a cost vector -->
			# initialize cost and value function vector for all action. Afterwards,
			# we are going to take the max
			V_alpha_a      = np.zeros((np.shape(self.Belief)[0], self.numA))
			V_cost_alpha_a = np.zeros(self.numA)

			J_alpha_a      = np.zeros((np.shape(self.Belief)[0], self.numA))
			J_cost_alpha_a = np.zeros( self.numA)
			
			bt = self.Belief[:, j]	
			for a in range(0, self.numA):
				for xNext in self.sucessorStatesList[xt][a]:
					Mbt = np.dot(self.M[a,xt,xNext], bt)
					J_lambda_kax = np.dot(self.J[self.t+1,xNext].T, Mbt.T)
					V_lambda_kax = np.dot(self.V[self.t+1,xNext].T, Mbt.T)

					idxOptList = []
					for o in range(0, self.numZ):
						J_lambda_kaxo = J_lambda_kax[:, o]
						idxToChange = J_lambda_kaxo < (J_lambda_kaxo.max()-self.toll)
						J_lambda_kaxo[idxToChange] = -1000  # Set non-max value to low numer 
						sumVec = J_lambda_kaxo + V_lambda_kax[:, o]
						idxOptList.append(np.argmax(sumVec))

					V_alpha_a[:, a] += np.tensordot(self.V[self.t+1,xNext,:,idxOptList], self.M[a,xt,xNext], 2)
					J_alpha_a[:, a] += np.tensordot(self.J[self.t+1,xNext,:,idxOptList], self.M[a,xt,xNext], 2)

				# Select Cost
				if xt in self.goal:
					V_cost_alpha_a[a] = 1 + np.dot(V_alpha_a[:, a].T, bt)
					J_cost_alpha_a[a] = 1
				else:
					V_cost_alpha_a[a] = np.dot(V_alpha_a[:, a].T, bt)
					J_cost_alpha_a[a] = np.dot(J_alpha_a[:, a].T, bt)	
	
			idxToChange = J_cost_alpha_a < (J_cost_alpha_a.max()-self.toll)  # Where values are low
			J_cost_alpha_a[idxToChange] = -1000  # Set non-max value to low numer
			sumVec = J_cost_alpha_a + V_cost_alpha_a
			idxOpt = np.argmax(sumVec)

			if xt in self.goal:
				V_out[:,j] = np.ones(np.shape(self.Belief)[0]) + V_alpha_a[:,idxOpt]
				J_out[:,j] = np.ones(np.shape(self.Belief)[0])
			else:
				V_out[:,j] = V_alpha_a[:,idxOpt]
				J_out[:,j] = J_alpha_a[:,idxOpt]

		self.V[self.t,xt] = V_out
		self.J[self.t,xt] = J_out

	def evaluatePolicy(self, t, xt, bt):
		V_alpha_a = np.zeros((self.numZ, self.numA))
		J_alpha_a = np.zeros((self.numZ, self.numA))
		actionCost = []
		actionSpec = []
	
		for a in range(0, self.numA):
			for xNext in self.sucessorStatesList[xt][a]:
				for o in range(0, self.numZ):
					# Cost Value Function Update
					J_lambda_kaxo = np.dot(self.J[t+1,xNext].T, np.dot( self.M[a,xt,xNext,o], bt ))
					V_lambda_kaxo = np.dot(self.V[t+1,xNext].T, np.dot( self.M[a,xt,xNext,o], bt ))

					idx = np.where( (J_lambda_kaxo >= np.max(J_lambda_kaxo)-self.toll) )			
					possOpt		 = -np.inf * np.ones(np.shape(V_lambda_kaxo)[0])
					possOpt[idx] =  V_lambda_kaxo[idx]
					
					idxOpt = np.argmax(possOpt)
					
					V_alpha_a[:, a] = V_alpha_a[:, a] + np.dot(self.M[a,xt,xNext,o].T, self.V[t+1,xNext,:, idxOpt])
					J_alpha_a[:, a] = J_alpha_a[:, a] + np.dot(self.M[a,xt,xNext,o].T, self.J[t+1,xNext,:, idxOpt])


			# Select Cost
			if xt in self.goal: stageCost = 1;
			else: stageCost = 0
			actionCost.append(np.round(stageCost + np.dot(V_alpha_a[:, a], bt), self.digitPrecision))
			actionSpec.append(np.round(np.dot(J_alpha_a[:, a], bt), self.digitPrecision))


		# Pick best action
		probability = max(actionSpec)
		cost		= max(actionCost)
		if self.printLevel > 2: print("Constraint vector: ", actionSpec)
		if self.printLevel > 2: print("Cost Vector: ", actionCost)
		if probability == 0:
			if self.printLevel >= 1: print("Abort Mission")
			probability    = 0
			selectedAction = 0
			cost           =  np.inf
		else:
			# Pick action with highest prob sat specs
			action = np.where(np.array(actionSpec) >= probability-self.toll)
			# Among the one with highest prob sat specs pick best cost
			possOpt		    = -np.inf * np.ones(self.numA)
			possOpt[action] =  np.array(actionCost)[action]
			actionSel	    =  np.where(possOpt==np.max(possOpt))
			# Print to sceen
			if self.printLevel > 2: print("Possible Moves ", self.actVec[action])
			if self.printLevel > 2: print("Same Cost Moves: ", self.actVec[actionSel])
			if self.printLevel > 2: print("Selected Action: ", self.actVec[actionSel[0][-1]])

			selectedAction = actionSel[0][-1]

		if self.printLevel >= 2: print("Probability satinsfying spec: ", probability, ". Expected Cost: ", cost)

		return selectedAction, probability, cost

class MOMDP_TOQ_d(MOMDP):

	def __init__(self, gridVar, totTimeSteps, printLevel, policy, discOpt, momdpSegway, unGoal = False):  

		self.momdpSegway = momdpSegway
		# super().__init__(gridVar, totTimeSteps, printLevel, policy, discOpt, valFunFlag = False) # uncomment for py3
		super(MOMDP_TOQ_d, self).__init__(gridVar, totTimeSteps, printLevel, policy, discOpt, valFunFlag = False) # uncomment for py2
		# Initialize stage cost --> shift value function by gamma + add a vector of zeros to ensure positivity
		self.stageCost   = momdpSegway.J[0, self.goal[0]]-0.8
		zeroVecFlag = False
		for i in range(0, self.stageCost.shape[1]-1):
			if (self.stageCost[:,i]==self.stageCost[:,i+1]).all():
				self.stageCost[:, i] = np.zeros(self.stageCost[:, i].shape)
				zeroVecFlag = True

		for i in range(0, self.stageCost.shape[1]):
			if (self.stageCost[:, i] < 0).all():
				print(self.stageCost[:, i])
				self.stageCost[:, i] = np.zeros(self.stageCost[:, i].shape)
				zeroVecFlag = True

		if zeroVecFlag == False:
			print("Failed to initialize the stage cost! No vector is negative --> Need to change belief dim to add vector of zeros")
			pdb.set_trace()

		
		# Modify V[N, goal]
		print("stageCost: ", self.stageCost)
		self.V[-1, self.goal[0]] = self.stageCost

		print("Start Value Function Approximation")
		self.computeValFun()
		print("Done Value Function Approximation")
	
	def initPointBased(self, discOpt):
		self.Belief = self.momdpSegway.Belief
		self.obstBelief =self.momdpSegway.obstBelief
		self.numBeliefPoints = self.obstBelief.shape[0]
		print("Belif points: ", self.numBeliefPoints)


	def locateUR(self, regionType):
		# Overwriting the method locateUR: it is needed that the obstacle order is equalt to the obstacle+goal order 
		# from the Segway policy, otherwise we cannot use the value function from the Segway
		stateLocation = []
		self.row_obs = self.momdpSegway.row_obs[:]
		self.row_obs.extend(self.momdpSegway.row_goal[:])
		self.col_obs = self.momdpSegway.col_obs[:]
		self.col_obs.extend(self.momdpSegway.col_goal[:])
		for i in range(0, len(self.row_obs)):
				stateLocation.append(self.stateMap[self.row_obs[i], self.col_obs[i]])
		return stateLocation

	def computeTs(self, a , activeObstacles, activeGoals):
		T = np.zeros( (self.numS, self.numS) )
		for i in range(0, self.row):
			for j in range(0, self.col):
				# compute successor
				nextState_row = max(0, min(i + self.add_row[a], self.row-1))
				nextState_col = max(0, min(j + self.add_col[a], self.col-1))
				T[self.stateMap[i,j], self.stateMap[nextState_row,nextState_col]] = 1
		return T.astype(int)

	def computeO(self, Ts, a, e):
		# M1 used when adiacent to unknown obstacle
		M1 = np.array([[1.0, 0.0],
					   [0.0, 1.0]])

		# M2 used when on the diagonal
		M2 = np.array([[0.8, 0.2],
					   [0.2, 0.8]])

		# M3 used in all other cases
		M3 = np.array([[0.5, 0.5],
					   [0.5, 0.5]])

		O = np.zeros((self.numO, self.numS))
		
		# loop over all grid points
		for i in range(0, self.row):
			for j in range(0, self.col):
				state = np.zeros( (self.numS, 1) )
				if (self.stateMap[i,j]>=0): # Check if stateMap(i,j) is a state
					state[self.stateMap[i,j], 0] = 1
					stateNext = np.dot(Ts.T, state)

					# Here we want to compute P(y_{k+1} | x_{k+1}, u_k). So first
					# we have to check if it is possible to be at x_{k+1} given
					# that we have applied u_k. If it is not possible, the
					# observing P(y_{k+1} | x_{k+1}, u_k) = 0. So, we are going to
					# check if the pre state is feasible: preFeasible
					if np.array_equal(state, stateNext): # check if applying the current action we stay still
						preFeasible = 1;
					else: # if applying the current action we are not still --> check pre state
						preState_row = i-self.add_row[a]
						preState_col = j-self.add_col[a]
						if (0 <= preState_row) and (preState_row < self.row) and (0 <= preState_col) and (preState_col < self.col): # check if pre state is in the grid
							preFeasible  = self.stateMap[preState_row, preState_col] >= 0 # check is pre state is not a known obstacle
						else:
							preFeasible = 0 # if outside the grid --> not possible to be at x_{k+1} after applying u_k

					# the variable preFeasible tells us if we could be at x_{k+1} after applying u_k 
					P_observe = 1 
					for ii in range(0, self.numObs):
						dist = np.abs(i-self.row_obs[ii]) + np.abs(j-self.col_obs[ii]) # compute distance from obstacle number ii
						
						# compute join probability 
						if dist <= 0:
							P_observe = np.kron(M1,P_observe)*preFeasible
						elif (dist == 1):
							P_observe = np.kron(M2,P_observe)*preFeasible
						else:
							P_observe = np.kron(M3,P_observe)*preFeasible
					O[:, self.stateMap[i,j]] = P_observe[:, e]
		return O

	def  backupLoopBeliefPoint(self, xt):
		J_out	= np.zeros((np.shape(self.Belief)[0],np.shape(self.Belief)[1]))
		V_out	= np.zeros((np.shape(self.Belief)[0],np.shape(self.Belief)[1]))

		for j in range(0, np.shape(self.Belief)[1]): # loop over belief points (this for loop can be parallelized)
			# for each belief point we are going to compute a cost vector -->
			# initialize cost and value function vector for all action. Afterwards,
			# we are going to take the max
			V_alpha_a      = np.zeros((np.shape(self.Belief)[0], self.numA))
			V_cost_alpha_a = np.zeros(self.numA)

			J_alpha_a      = np.zeros((np.shape(self.Belief)[0], self.numA))
			J_cost_alpha_a = np.zeros( self.numA)
			
			bt = self.Belief[:, j]	
			for a in range(0, self.numA):
				for xNext in self.sucessorStatesList[xt][a]:
					Mbt = np.dot(self.M[a,xt,xNext], bt)
					J_lambda_kax = np.dot(self.J[self.t+1,xNext].T, Mbt.T)
					V_lambda_kax = np.dot(self.V[self.t+1,xNext].T, Mbt.T)

					idxOptList = []
					for o in range(0, self.numZ):
						J_lambda_kaxo = J_lambda_kax[:, o]
						idxToChange = J_lambda_kaxo < (J_lambda_kaxo.max()-self.toll)
						J_lambda_kaxo[idxToChange] = -1000  # Set non-max value to low numer 
						sumVec = J_lambda_kaxo + V_lambda_kax[:, o]
						idxOptList.append(np.argmax(sumVec))

					V_alpha_a[:, a] += np.tensordot(self.V[self.t+1,xNext,:,idxOptList], self.M[a,xt,xNext], 2)
					J_alpha_a[:, a] += np.tensordot(self.J[self.t+1,xNext,:,idxOptList], self.M[a,xt,xNext], 2)

				# Select Cost
				if xt in self.goal:
					V_cost_alpha_a[a] = np.max(np.dot(self.stageCost.T, bt)) + np.dot(V_alpha_a[:, a].T, bt)
					J_cost_alpha_a[a] = 1
				else:
					V_cost_alpha_a[a] = np.dot(V_alpha_a[:, a].T, bt)
					J_cost_alpha_a[a] = np.dot(J_alpha_a[:, a].T, bt)	
	
			idxToChange = J_cost_alpha_a < (J_cost_alpha_a.max()-self.toll)  # Where values are low
			J_cost_alpha_a[idxToChange] = -1000  # Set non-max value to low numer
			sumVec = J_cost_alpha_a + V_cost_alpha_a
			idxOpt = np.argmax(sumVec)

			if xt in self.goal:
				idxStageCost = np.argmax(np.dot(self.stageCost.T, bt))
				V_out[:,j] = self.stageCost[:, idxStageCost] + V_alpha_a[:,idxOpt]
				J_out[:,j] = np.ones(np.shape(self.Belief)[0])				
			else:
				V_out[:,j] = V_alpha_a[:,idxOpt]
				J_out[:,j] = J_alpha_a[:,idxOpt]

		self.V[self.t,xt] = V_out
		self.J[self.t,xt] = J_out


	def evaluatePolicy(self, t, xt, bt):
		V_alpha_a = np.zeros((self.numZ, self.numA))
		J_alpha_a = np.zeros((self.numZ, self.numA))
		actionCost = []
		actionSpec = []
	
		for a in range(0, self.numA):
			for xNext in self.sucessorStatesList[xt][a]:
				for o in range(0, self.numZ):
					# Cost Value Function Update
					J_lambda_kaxo = np.dot(self.J[t+1,xNext].T, np.dot( self.M[a,xt,xNext,o], bt ))
					V_lambda_kaxo = np.dot(self.V[t+1,xNext].T, np.dot( self.M[a,xt,xNext,o], bt ))

					idx = np.where( (J_lambda_kaxo >= np.max(J_lambda_kaxo)-self.toll) )			
					possOpt		 = -np.inf * np.ones(np.shape(V_lambda_kaxo)[0])
					possOpt[idx] =  V_lambda_kaxo[idx]
					
					idxOpt = np.argmax(possOpt)
					
					V_alpha_a[:, a] = V_alpha_a[:, a] + np.dot(self.M[a,xt,xNext,o].T, self.V[t+1,xNext,:, idxOpt])
					J_alpha_a[:, a] = J_alpha_a[:, a] + np.dot(self.M[a,xt,xNext,o].T, self.J[t+1,xNext,:, idxOpt])


			# Select Cost
			if xt in self.goal: 
				stageCost = np.max(np.dot(self.stageCost.T, bt));
			else: 
				stageCost = 0
			actionCost.append(np.round(stageCost + np.dot(V_alpha_a[:, a], bt), self.digitPrecision))
			actionSpec.append(np.round(np.dot(J_alpha_a[:, a], bt), self.digitPrecision))


		# Pick best action
		probability = max(actionSpec)
		cost		= max(actionCost)
		if self.printLevel > 2: print("Constraint vector: ", actionSpec)
		if self.printLevel > 2: print("Cost Vector: ", actionCost)
		if probability == 0:
			if self.printLevel >= 1: print("Abort Mission")
			probability    = 0
			selectedAction = 0
			cost           =  np.inf
		else:
			# Pick action with highest prob sat specs
			action = np.where(np.array(actionSpec) >= probability-self.toll)
			# Among the one with highest prob sat specs pick best cost
			possOpt		    = -np.inf * np.ones(self.numA)
			possOpt[action] =  np.array(actionCost)[action]
			actionSel	    =  np.where(possOpt==np.max(possOpt))
			# Print to sceen
			if self.printLevel > 2: print("Possible Moves ", self.actVec[action])
			if self.printLevel > 2: print("Same Cost Moves: ", self.actVec[actionSel])
			if self.printLevel > 2: print("Selected Action: ", self.actVec[actionSel[0][0]])

			selectedAction = actionSel[0][0]

		if self.printLevel >= 2: print("Probability satinsfying spec: ", probability, ". Expected Cost: ", cost)

		return selectedAction, probability, cost

class MOMDP_TO(MOMDP):

	def __init__(self, gridVar, totTimeSteps, printLevel, policy, discOpt):  
		super().__init__(gridVar, totTimeSteps, printLevel, policy, discOpt) 

	def  backupLoopBeliefPoint(self, xt):
		V_out	= np.zeros((np.shape(self.Belief)[0],np.shape(self.Belief)[1]))			
		
		for j in range(0, np.shape(self.Belief)[1]): # loop over belief points
			# for each belief point we are going to compute a cost vector -->
			# initialize cost and value function vector for all action. Afterwards,
			# we are going to take the max
			V_alpha_a      = np.zeros((np.shape(self.Belief)[0], self.numA))
			V_cost_alpha_a = np.zeros(self.numA)

			
			bt = self.Belief[:, j]	
			for a in range(0, self.numA):
				for xNext in self.sucessorStatesList[xt][a]:
					Mbt = np.dot(self.M[a,xt,xNext], bt)
					V_lambda_kax = np.dot(self.V[self.t+1,xNext].T, Mbt.T)

					idxOptList = []
					for o in range(0, self.numZ):
						idxOptList.append(np.argmax(V_lambda_kax[:, o]))

					V_alpha_a[:, a] += np.tensordot(self.V[self.t+1,xNext,:,idxOptList], self.M[a,xt,xNext], 2)

				# Select Cost
				if xt in self.goal:
					V_cost_alpha_a[a] = 1 + np.dot(V_alpha_a[:, a].T, bt)
				else:
					V_cost_alpha_a[a] = np.dot(V_alpha_a[:, a].T, bt)
	
			idxOpt = np.argmax(V_cost_alpha_a)

			if xt in self.goal:
				V_out[:,j] = np.ones(np.shape(self.Belief)[0]) + V_alpha_a[:,idxOpt]
			else:
				V_out[:,j] = V_alpha_a[:,idxOpt]

		self.V[self.t,xt] = V_out


	def evaluatePolicy(self, t, xt, bt):
		V_alpha_a = np.zeros((self.numZ, self.numA))
		actionCost = []
	
		for a in range(0, self.numA):
			for xNext in self.sucessorStatesList[xt][a]:
				for o in range(0, self.numZ):
					# Cost Value Function Update
					V_lambda_kaxo = np.dot(self.V[t+1,xNext].T, np.dot( self.M[a,xt,xNext,o], bt ))
					idxOpt = np.argmax(V_lambda_kaxo)
					V_alpha_a[:, a] = V_alpha_a[:, a] + np.dot(self.M[a,xt,xNext,o].T, self.V[t+1,xNext,:, idxOpt])


			# Select Cost
			if xt in self.goal: stageCost = 1;
			else: stageCost = 0
			actionCost.append(stageCost + np.dot(V_alpha_a[:, a], bt))

		# Pick best action
		cost		= max(actionCost)
		probability = np.inf
		if self.printLevel > 2: print("Cost Vector: ", actionCost)
		if cost == 0:
			if self.printLevel >= 1: print("Run out of time abort the mission")
			probability    = 0
			selectedAction = 0
			cost           = np.inf
		else:
			actionSel	    =  np.where(actionCost==np.max(actionCost))
			# Print to sceen
			if self.printLevel > 2: print("Same Cost Moves: ", self.actVec[actionSel])
			if self.printLevel > 2: print("Selected Action: ", self.actVec[actionSel[0][0]])

			selectedAction = actionSel[0][0]
			probability    = np.inf # the probability cannot be computed in TO, unless we run out of time --> failure
		
		if self.printLevel >= 2: print("Probability satinsfying spec: ", probability, ". Expected Cost: ", cost)

		return selectedAction, probability, cost

class MOMDP_Q(MOMDP):

	def __init__(self, gridVar, totTimeSteps, printLevel, policy, discOpt):  
		super().__init__(gridVar, totTimeSteps, printLevel, policy, discOpt) 

	def  backupLoopBeliefPoint(self, xt):
		J_out	= np.zeros((np.shape(self.Belief)[0],np.shape(self.Belief)[1]))				
		
		for j in range(0, np.shape(self.Belief)[1]): # loop over belief points
			# for each belief point we are going to compute a cost vector -->
			# initialize cost and value function vector for all action. Afterwards,
			# we are going to take the max
			J_alpha_a      = np.zeros((np.shape(self.Belief)[0], self.numA))
			J_cost_alpha_a = np.zeros( self.numA)
			J_cost_alpha_a_vec = np.zeros( self.numA)
			J_alpha_a_vec      = np.zeros((np.shape(self.Belief)[0], self.numA))
			bt = self.Belief[:, j]	
			for a in range(0, self.numA):
				for xNext in self.sucessorStatesList[xt][a]:
					Mbt_vec = np.dot(self.M[a,xt,xNext], bt)
					J_lambda_kax = np.dot(self.J[self.t+1,xNext].T, Mbt_vec.T)

					idxOptList = []
					for o in range(0, self.numZ):
						idxOptList.append(np.argmax(J_lambda_kax[:, o] ))

					J_alpha_a[:, a] += np.tensordot(self.J[self.t+1,xNext,:,idxOptList], self.M[a,xt,xNext], 2)


				# Select Cost
				if xt in self.goal:
					J_cost_alpha_a[a] = 1
				else:
					J_cost_alpha_a[a] = np.round(np.dot(J_alpha_a[:, a].T, bt), self.digitPrecision)

			idxOpt = np.argmax( J_cost_alpha_a )
			if xt in self.goal:
				J_out[:,j] = np.ones(np.shape(self.Belief)[0])
			else:
				J_out[:,j] = J_alpha_a[:,idxOpt]

		self.J[self.t,xt] = J_out


	def evaluatePolicy(self, t, xt, bt):
		J_alpha_a = np.zeros((self.numZ, self.numA))
		actionSpec = []
	
		for a in range(0, self.numA):
			for xNext in self.sucessorStatesList[xt][a]:
				for o in range(0, self.numZ):
					# Cost Value Function Update
					J_lambda_kaxo = np.dot(self.J[t+1][xNext].T, np.dot( self.M[a,xt,xNext,o], bt ))
					
					idxOpt = np.argmax(J_lambda_kaxo)
					
					J_alpha_a[:, a] = J_alpha_a[:, a] + np.dot(self.M[a,xt,xNext,o].T, self.J[t+1,xNext,:, idxOpt])


			# Select Cost
			actionSpec.append(np.round(np.dot(J_alpha_a[:, a], bt), self.digitPrecision))

		# Pick best action
		probability = max(actionSpec)
		cost        =  np.inf # set cost to inf because no cost is computed for the Quantitative policy
		# pdb.set_trace()
		if self.printLevel > 2: print("Max spec vector: ", actionSpec)
		if probability == 0:
			if self.printLevel >= 1: print("Abort Mission")
			probability    = 0
			selectedAction = 0
			
		else:
			actionSel	    =  np.where(actionSpec==np.max(actionSpec))
			# Print to sceen
			if self.printLevel > 2: print("Possible Moves ", self.actVec[actionSel])
			if self.printLevel > 2: print("Selected Action: ", self.actVec[actionSel[0][0]])

			selectedAction = actionSel[0][0]

		if self.printLevel >= 2: print("Probability satinsfying spec: ", probability, ". Expected Cost: ", cost)

		return selectedAction, probability, cost


