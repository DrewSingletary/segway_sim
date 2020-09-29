import pdb
import scipy.io as sio
import numpy as np
import pickle
import sys
from utils import *
from MOMDP import MOMDP, MOMDP_TOQ, MOMDP_TOQ_d


def main():
	load          = 0 # 0 = compute policy and save policy object, 1 = load policy object, -1 = compute policy but DO NOT save it
	digitsResults = 6 # number of digits to print, just for visual output 
	printLevel    = 4 # 0 = only value function update and results, 1 = minimal, 2 = verbose
	
	# Evaluate single policy. Details in evaluateSinglePolicy() function
	discOpt = 2
	evaluateSinglePolicy(load, digitsResults, printLevel, discOpt)	

def evaluateSinglePolicy(load, digitsResults, printLevel, discOpt):
	numObst = 2

	# gridWorld = '7x7'
	gridWorld = '7x7ug'
	policy = 'segway'
	momdpSegway = getMOMDP(gridWorld, numObst, policy, printLevel, load, discOpt, unGoal = True)
	runSim(momdpSegway, gridWorld, numObst, policy, printLevel, digitsResults) 
	policy = 'drone'
	gridWorld = '7x7ug_d' # IMPORTANT: for the drone there is always one goal --> unGoal = False
	# gridWorld = '7x7_d' # IMPORTANT: for the drone there is always one goal --> unGoal = False
	numObst = momdpDrone  = getMOMDP(gridWorld, numObst, policy, printLevel, load, discOpt, momdpSegway = momdpSegway)
	runSim(momdpDrone, gridWorld, numObst, policy, printLevel, digitsResults) 
	
def getMOMDP(gridWorld, numObst, policy, printLevel, load, discOpt, unGoal = False, momdpSegway = None):
	totTimeSteps, _, _ = loadParameters(gridWorld, numObst, unGoal)


	fileName = sys.path[0]+'/multiAgent/'+policy+'_'+gridWorld+'_'+str(numObst)+'.pkl'

	if load <= 0: # If load <= 0 compute the policy and store it if load == 0
		gridVar = loadGrid(gridWorld+'_'+str(numObst))
		if policy == 'segway':
			momdp = MOMDP_TOQ(gridVar, totTimeSteps,printLevel, policy, discOpt, unGoal = True)
		elif policy == 'drone':
			momdp = MOMDP_TOQ_d(gridVar, totTimeSteps,printLevel, policy, discOpt, momdpSegway)

		if load == 0:
			pickle_out = open(fileName,"wb")
			pickle.dump(momdp, pickle_out)
			pickle_out.close()
	else: 
		pickle_in = open(fileName,"rb")
		momdp = pickle.load(pickle_in)
	
	return momdp

def runSim(momdp, gridWorld, numObst, policy, printLevel, digits):
	totTimeSteps, loc, initBelief = loadParameters(gridWorld, numObst, momdp.unGoal)

	bt = [momdp.initBelief(initBelief)] # initial belief
	xt = [0] # initial state
	V_t0, Vrealized, J_t0, failure,tRun, xt, bt = eveluateMOMDP(momdp, loc, initBelief, xt, bt, printLevel)	

	plotFun(momdp, xt, bt)
	print("cl: ", xt)

if __name__ == "__main__":
    # execute only if run as a script
    main()