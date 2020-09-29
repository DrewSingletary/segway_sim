import sys
import pdb
import scipy.io as sio
import numpy as np
import pickle
from utils import *
from MOMDP import MOMDP, MOMDP_TOQ, MOMDP_TO, MOMDP_Q


def main():
	load          = 0 # 0 = compute policy and save policy object, 1 = load policy object, -1 = compute policy but DO NOT save it
	digitsResults = 6 # number of digits to print, just for visual output 
	printLevel    = 3 # 0 = only value function update and results, 1 = minimal, 2 = verbose
	
	# Evaluate single policy. Details in evaluateSinglePolicy() function
	discOpt = 1
	evaluateSinglePolicy(load, digitsResults, printLevel, discOpt)	

	# # Save .txt file with results for Table I
	# discOpt = 1 # 1 = 2^{n_u} + n_u belief points, 2 = 2(2^{n_u}) belief points
	# gridWorldList = ['5x5' , '10x5', '15x15' ]
	# obstList      = [3, 4]
	# policyList    = ['TO', 'Q', 'TOQ']
	# evaluateAllPolicies(load, digitsResults,printLevel, discOpt, gridWorldList, obstList, policyList)

	# # Save .txt file with results for Table II
	# discOpt = 2 # 1 = 2^{n_u} + n_u belief points, 2 = 2(2^{n_u}) belief points
	# gridWorldList = ['15x15' ]
	# obstList      = [4]
	# evaluateAllPolicies(load, digitsResults,printLevel, discOpt, gridWorldList, obstList, policyList)

def evaluateSinglePolicy(load, digitsResults, printLevel, discOpt):
	# gridWorld = '5x5ug'
	# numObst = 2
	# gridWorld = '5x5ug'
	# numObst = 3
	gridWorld = '5x5'
	numObst = 3
	policy = 'TOQ'

	momdp   = getMOMDP(gridWorld, numObst, policy, printLevel, load, discOpt, unGoal = False)

	# # Evaluate expected cost and probability of failure
	# results = runAllSim(momdp, gridWorld, numObst, policy, printLevel, digitsResults) 

	# Run a simulation for an environment realization which is defined in the function loadParameters() from the file utils.py
	results = runSim(momdp, gridWorld, numObst, policy, printLevel, digitsResults) 

def evaluateAllPolicies(load, digitsResults, printLevel, discOpt, gridWorldList, obstList, policyList):

	resultsList = []
	for gridWorld in gridWorldList:
		for numObst in obstList:
			for policy in policyList:
				momdp   = getMOMDP(gridWorld, numObst, policy, printLevel, load, discOpt)
				results = runAllSim(momdp, gridWorld, numObst, policy, printLevel, digitsResults)
				resultsList.append(results)

	# Save and print to screen the results
	print("====== Results for expected cost and prob sat specs")
	fileToWrite = open("result_expected_cost_Table_"+str(discOpt)+".txt","w") 
	for i  in range(0, len(resultsList)):
		print(resultsList[i][0])
		fileToWrite.writelines(resultsList[i][0]+'\n') 
	fileToWrite.close() #to change file access modes 

	fileToWrite = open("result_expected_time_Table_"+str(discOpt)+".txt","w") 
	print("====== Results for expected time and prob of failure")
	for i  in range(0, len(resultsList)):
		print(resultsList[i][1])
		fileToWrite.writelines(resultsList[i][1]+'\n') 
	fileToWrite.close() #to change file access modes 

def getMOMDP(gridWorld, numObst, policy, printLevel, load, discOpt, unGoal = False):
	totTimeSteps, _, _ = loadParameters(gridWorld, numObst, unGoal)

	if unGoal == False:
		fileName = sys.path[0]+'/'+policy+'_'+str(discOpt)+'/'+'MOMDP_obj_'+gridWorld+'_'+str(numObst)+'.pkl'
	else:
		fileName = sys.path[0]+'/'+policy+'ug_'+str(discOpt)+'/'+'MOMDP_obj_'+gridWorld+'_'+str(numObst)+'.pkl'

	
	if load <= 0: # If load <= 0 compute the policy and store it if load == 0
		gridVar = loadGrid(gridWorld+'_'+str(numObst))
		if policy == 'TOQ':
			# momdp = MOMDP_TOQ_notVectorized(gridVar, totTimeSteps,printLevel, policy, discOpt)
			momdp = MOMDP_TOQ(gridVar, totTimeSteps,printLevel, policy, discOpt, unGoal)
		elif policy == 'Q':
			momdp = MOMDP_Q(gridVar, totTimeSteps,printLevel, policy, discOpt)
		elif policy == 'TO':
			momdp = MOMDP_TO(gridVar, totTimeSteps,printLevel, policy, discOpt)

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
	
	# Print and store results
	print("================ Final Results for ", policy, " Policy in ", gridWorld, " grid world with ", numObst, "obstacles.")
	print("Vrealized: ", Vrealized, " and V0: ", V_t0)
	print("Prob Sat Spec: ", 1 - failure, " and lower bound: ", J_t0)
	print("Time: ", tRun)
	print("Policy synthesis time: ", momdp.totTime, " Avg backup time: ", momdp.avgBackupTime)


def runAllSim(momdp, gridWorld, numObst, policy, printLevel, digits):
	totTimeSteps, _, initBelief = loadParameters(gridWorld, numObst, momdp.unGoal)

	probFailure = 0
	expCost     = 0
	expTime     = 0
	for i in range(0,len(momdp.comb)): # loop over all possible 2^numObst obstacle configurations 
		loc = momdp.comb[i] # initialize true obstacle location
		bt = [momdp.initBelief(initBelief)] # initial belief
		xt = [0] # initial state
		V_t0, Vrealized, J_t0, failure,tRun, _, _ = eveluateMOMDP(momdp, loc, initBelief, xt, bt, printLevel)	
		probFailure += failure*bt[0][i] # add prob failure times probability of this scenario
		expCost     += Vrealized*bt[0][i] # add cost times probability of this scenario
		expTime     += tRun*bt[0][i]*(failure==0)

	# Print and store results
	print("================ Final Results for ", policy, " Policy in ", gridWorld, " grid world with ", numObst, "obstacles.")
	print("Expected cost: ", expCost, " and V0: ", V_t0)
	print("Prob Sat Spec: ", 1 - probFailure, " and lower bound: ", J_t0)
	print("Expected time: ", expTime)
	print("Prob Failure: ", probFailure, " and upper bound: ", 1-J_t0)
	print("Policy synthesis time: ", momdp.totTime, " Avg backup time: ", momdp.avgBackupTime)

	if policy == 'TO': policy = 'TO '
	if policy == 'Q': policy = 'Q  '
	stringTest = policy+"_"+gridWorld+"_"+str(numObst)
	stringCost = " || ExpCost: "+str(round(expCost,digits))+", V0: "+str(round(V_t0,digits))
	stringProb = " || P specs: "+str(round(1-probFailure,digits))+", J0: "+str(round(J_t0,digits))
	stringTime = " || Tot Time: "+str(round(momdp.totTime,digits))+", backup time: "+str(round(momdp.avgBackupTime,digits))

	stringExpT = " || ExpTime: "+str(round(expTime,digits))
	stringFail = " || P fail: "+str(round(probFailure,digits))+" and upper-bound 1-J0: "+str(round(1-J_t0,digits))

	return [stringTest+stringCost+stringProb+stringTime, stringTest+stringExpT+stringFail+stringTime]

if __name__ == "__main__":
    # execute only if run as a script
    main()