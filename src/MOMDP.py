import numpy as np
import pdb
import itertools

class MOMDP(object):
	"""docstring for MOMDP"""
	def __init__(self, Px, M, V, J, gridVar, verbose):
		self.Px = Px # P[action, zCurr]
		self.M  = M  # P[action, xCurr, xNext, oCurr]
		self.V  = V
		self.J  = J
		self.verbose = verbose

		self.numA = Px.shape[0]
		self.numO = Px.shape[1] 
		self.numZ = self.numO
		self.numX = M.shape[1]

		self.gridVar   = gridVar
		self.stateMap  = self.stateMap(gridVar)
		self.cellWidth = 1.0
		self.cellHight = 1.0

		self.numUnKnownObst = np.where( (gridVar>0) & (gridVar<1) )[0].shape[0]
		self.numKnownObst   = np.where( gridVar<0 )[0].shape[0]

		self.goal = self.stateMap[np.where(gridVar == 1)]

		self.actVec = np.array(['n','s','w','e']);

	def evaluatePolicy(self, t, xt, bt):
		V_alpha_a = np.zeros((self.numZ, self.numA))
		J_alpha_a = np.zeros((self.numZ, self.numA))
		actionCost = []
		actionSpec = []
	
		for a in range(0, self.numA):
			for xNext in self.sucessorStates(xt, a):
				for o in range(0, self.numZ):
					# Cost Value Function Update
					V_lambda_kaxo = np.dot(self.V[t+1, xNext].T, np.dot( self.M[a, xt, xNext, o], bt ))
					idx = np.argmax(V_lambda_kaxo, 0)
					V_alpha_a[:, a] = V_alpha_a[:, a] + np.dot(self.M[a, xt, xNext, o], self.V[t+1, xNext][:, idx])
					# Spec Value Function Update
					J_lambda_kaxo = np.dot(self.J[t+1, xNext].T, np.dot( self.M[a, xt, xNext, o], bt ))
					idx = np.argmax(J_lambda_kaxo, 0)
					J_alpha_a[:, a] = J_alpha_a[:, a] + np.dot(self.M[a, xt, xNext, o], self.J[t+1, xNext][:, idx])

			# Overwrite if next state = goal state (TO DO: remove for loop)
			for o in range(0, self.numZ):
				if self.propagate(xt, o, a) == self.goal:
					J_alpha_a[o, a] = 1

			# Select Cost
			if xt == self.goal: stageCost = 1;
			else: stageCost = 0
			actionCost.append(stageCost + np.dot(V_alpha_a[:, a], bt))
			actionSpec.append(np.dot(J_alpha_a[:, a], bt))


		# Pick best action
		probability = max(actionSpec)
		cost        = max(actionCost)
		if probability == 0:
			print(bt)
			print("Abort Mission")
		else:
			# Pick action with highest prob sat specs
			action = np.where(np.array(actionSpec) == probability)
			# Among the one with highest prob sat specs pick best cost
			possOpt         = -np.inf * np.ones(self.numA)
			possOpt[action] =  np.array(actionCost)[action]
			actionSel       =  np.where(possOpt==np.max(possOpt))
			# Print to sceen
			if (self.verbose==1): print("Possible Moves ", self.actVec[action])
			if (self.verbose==1): print("Same Cost Moves: ", self.actVec[actionSel])
			if (self.verbose==1): print("Selected Action: ", self.actVec[actionSel[0][-1]])


		if (self.verbose==1): print("Probability satinsfying spec: ", probability, ". Expected Cost: ", cost)

		return actionSel[0][-1], probability, cost

	def sucessorStates(self, xt, at):
		nextStatetList = [self.propagate(xt, z, at) for z in range(0, self.numZ)]
		return list( dict.fromkeys(nextStatetList) )

	def propagate(self, xt, zt, at):
		return np.argmax(np.dot(self.Px[at, zt].T, self.stateToVector(xt)))
	
	def stateToVector(self, xt):
		xtVec     = np.zeros(self.numX)
		xtVec[xt] = 1
		return xtVec

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

		print("State Map: ")
		print(stateMap)

		print("gridVar: ")
		print(gridVar)

		return stateMap

	def initBeliefAndZ(self, loc, probInit):
		self.zt = int(loc, 2)
		InitialBelief = []
		# List all comb
		comb = list(itertools.product([0, 1], repeat=len(probInit)))
		
		# Loop to assign prob
		for i in range(0, len(comb)):
			prob = 1
			for j in range(0, len(probInit)):
				prob = prob * ( (1-comb[i][j]) * probInit[j] + comb[i][j] * (1 - probInit[j]) )
			InitialBelief.append(prob)

		return [np.array(InitialBelief)]

	def computeObstacleBelief(self, bt, idx):
		comb = list(itertools.product([0, 1], repeat=self.numUnKnownObst))
		prob = 0
		for i in range(0, len(comb)):
			prob = prob + ( (1-comb[i][idx]) * bt[i] ) #+ comb[i][j] * (1 - bt[j]) )
		return prob

	def evaluateCost(self, t, xt, bt):		
		return np.max(np.dot(self.V[t, xt].T, bt))

	def updateBelief(self, xt, xNext, at, ot, bt):
		return np.dot( self.M[at, xt, xNext, ot], bt )/np.sum(np.dot( self.M[at, xt, xNext, ot], bt ))

	def approxValueFunction(self, t, xCurr, bCurr):
		coordXY = []
		coordCo = []
		xCurLst = []
		for i in range(0,3):
			coordCo.append(self.evaluateCost(t+1, xCurr, bCurr))
			coordXY.append(self.getCoordinates(xCurr))
			xCurLst.append(xCurr)
		
			[at, spec, cost] = self.evaluatePolicy(t+i, xCurr, bCurr)
			xCurr = self.propagate(xCurr, 0, at)

		return coordXY, coordCo, xCurLst

	def updateMOMDP(self, t, xt, bt):
	    coordXY, coordCo, xCurLst = self.approxValueFunction(t, xt, bt)
	    boxConstraints = self.getBoxConstr(xCurLst[0], xCurLst[1])
	    boxNext        = self.getBoxCurren(xCurLst[1])
	    [action, spec, cost] = self.evaluatePolicy(t, xt, bt)
	    print("coordXY: ", coordXY)
	    print("coordCo: ", coordCo)
	    print("xCurLst: ", xCurLst)
	    print("boxConstraints: ", boxConstraints)
	    print("boxNext: ", boxNext)

	    return action, coordXY, coordCo, boxConstraints, boxNext



class MOMDP_obst(object):
	"""docstring for MOMDP"""
	def __init__(self, Px, M, V, J, gridVar, verbose):
		self.Px = Px # P[action, zCurr]
		self.M  = M  # P[action, xCurr, xNext, oCurr]
		self.V  = V
		self.J  = J
		self.verbose = verbose

		self.numA = Px.shape[0]
		self.numO = Px.shape[1] 
		self.numZ = self.numO
		self.numX = M.shape[1]

		self.gridVar   = gridVar
		self.stateMap  = self.stateMap(gridVar)
		self.cellWidth = 1.0
		self.cellHight = 1.0

		self.numUnKnownObst = np.where( (gridVar>0) & (gridVar<1) )[0].shape[0]
		self.numKnownObst   = np.where( gridVar<0 )[0].shape[0]

		self.goal = self.stateMap[np.where(gridVar == 1)]

		self.actVec = np.array(['n','s','w','e']);

	def evaluatePolicy(self, t, xt, bt):
		V_alpha_a = np.zeros((self.numZ, self.numA))
		J_alpha_a = np.zeros((self.numZ, self.numA))
		actionCost = []
		actionSpec = []
	
		toll = 0.00001
		for a in range(0, self.numA):
			for xNext in self.sucessorStates(xt, a):
				for o in range(0, self.numZ):
					# Cost Value Function Update
					J_lambda_kaxo = np.dot(self.J[t+1, xNext].T, np.dot( self.M[a, xt, xNext, o], bt ))
					V_lambda_kaxo = np.dot(self.V[t+1, xNext].T, np.dot( self.M[a, xt, xNext, o], bt ))

					idx = np.where( (J_lambda_kaxo >= np.max(J_lambda_kaxo)-toll) & (J_lambda_kaxo <= np.max(J_lambda_kaxo)+toll) )			
					possOpt         = -np.inf * np.ones(np.shape(V_lambda_kaxo)[0])
					possOpt[idx] =  V_lambda_kaxo[idx]
					
					idxOpt = np.argmax(possOpt)
					
					V_alpha_a[:, a] = V_alpha_a[:, a] + np.dot(self.M[a, xt, xNext, o].T, self.V[t+1, xNext][:, idxOpt])
					J_alpha_a[:, a] = J_alpha_a[:, a] + np.dot(self.M[a, xt, xNext, o].T, self.J[t+1, xNext][:, idxOpt])
					# Spec Value Function Update


			# Select Cost
			if xt == self.goal:
				actionCost.append(1 + np.dot(V_alpha_a[:, a], bt))
				actionSpec.append(np.dot(np.ones(np.shape(bt)[0]), bt))
			else: 
				actionCost.append(np.dot(V_alpha_a[:, a], bt))
				actionSpec.append(np.dot(J_alpha_a[:, a], bt))
			

		# Pick best action
		probability = max(actionSpec)
		cost        = max(actionCost)
		if probability == 0:
			print(bt)
			print("Abort Mission")
		else:
			# Pick action with highest prob sat specs
			action = np.where( (np.array(actionSpec) >= probability-toll ) & (np.array(actionSpec) <= probability+toll ))
			# Among the one with highest prob sat specs pick best cost
			possOpt         = -np.inf * np.ones(self.numA)
			possOpt[action] =  np.array(actionCost)[action]
			actionSel       =  np.where(possOpt==np.max(possOpt))
			# Print to sceen
			if (self.verbose==1): print("Possible Moves ", self.actVec[action])
			if (self.verbose==1): print("Same Cost Moves: ", self.actVec[actionSel])
			if (self.verbose==1): print("Selected Action: ", self.actVec[actionSel[0][-1]])
			print("Possible Moves ", self.actVec[action], actionSpec)
			print("Same Cost Moves: ", self.actVec[actionSel], actionCost)
			print("Selected Action: ", self.actVec[actionSel[0][-1]], t, bt)


		if (self.verbose==1): print("Probability satinsfying spec: ", probability, ". Expected Cost: ", cost)

		return actionSel[0][-1], probability, cost

	def sucessorStates(self, xt, at):
		nextStatetList = [self.propagate(xt, z, at) for z in range(0, self.numZ)]
		print(list( dict.fromkeys(nextStatetList) ))
		return list( dict.fromkeys(nextStatetList) )

	def propagate(self, xt, zt, at):
		return np.argmax(np.dot(self.Px[at, zt].T, self.stateToVector(xt)))
	
	def stateToVector(self, xt):
		xtVec     = np.zeros(self.numX)
		xtVec[xt] = 1
		return xtVec

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

		print("State Map: ")
		print(stateMap)

		print("gridVar: ")
		print(gridVar)

		return stateMap

	def initBeliefAndZ(self, obstBelief, unobsState):
		self.zt = unobsState
		# List all comb
		self.comb = np.shape(obstBelief)[0]
		print("==============================", obstBelief/np.sum(obstBelief))
		return [obstBelief/np.sum(obstBelief)]

	def computeObstacleBelief(self, bt, idx):
		return 1-bt[idx]

	def evaluateCost(self, t, xt, bt):		
		return np.max(np.dot(self.V[t, xt].T, bt))

	def updateBelief(self, xt, xNext, at, ot, bt):
		print("bt: ", bt)
		print("xt: ", xt)
		print("xNext: ", xNext)
		print("at", at)
		print("ot", ot)
		
		print("bt+: ", np.dot( self.M[at, xt, xNext, ot], bt )/np.sum(np.dot( self.M[at, xt, xNext, ot], bt )))
		
		return np.dot( self.M[at, xt, xNext, ot], bt )/np.sum(np.dot( self.M[at, xt, xNext, ot], bt ))

	def approxValueFunction(self, t, xCurr, bCurr):
		coordXY = []
		coordCo = []
		xCurLst = []
		for i in range(0,3):
			coordCo.append(self.evaluateCost(t+1, xCurr, bCurr))
			coordXY.append(self.getCoordinates(xCurr))
			xCurLst.append(xCurr)
		
			[at, spec, cost] = self.evaluatePolicy(t+i, xCurr, bCurr)
			xCurr = self.propagate(xCurr, 0, at)

		return coordXY, coordCo, xCurLst

	def updateMOMDP(self, t, xt, bt):
	    coordXY, coordCo, xCurLst = self.approxValueFunction(t, xt, bt)
	    boxConstraints = self.getBoxConstr(xCurLst[0], xCurLst[1])
	    boxNext        = self.getBoxCurren(xCurLst[1])
	    [action, spec, cost] = self.evaluatePolicy(t, xt, bt)
	    if xt == 17:
	    	print("Here!!!!!!!!!!!!!!")
	    	print("time: ",t)
	    	print(1-bt)
	    # print("coordXY: ", coordXY)
	    # print("coordCo: ", coordCo)
	    # print("xCurLst: ", xCurLst)
	    # print("boxConstraints: ", boxConstraints)
	    # print("boxNext: ", boxNext)

	    return action, coordXY, coordCo, boxConstraints, boxNext


