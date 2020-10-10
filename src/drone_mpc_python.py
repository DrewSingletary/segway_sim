#!/usr/bin/env python

import os
import sys
import datetime
import rospy
import numpy as np
import time
import scipy
import pdb
import sys
import time
import matplotlib.pyplot as plt
from segway_sim.msg import state
from segway_sim.msg import stateDrone
from segway_sim.msg import valFunCnst
from segway_sim.msg import optSol
import scipy.io as sio
import numpy as np
# from MOMDP import MOMDP_Segway, MOMDP_Drone
from segway_sim.msg import goalSetAndState

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
from segway_sim.msg import cmdDrone as cmd
sys.path.append(sys.path[0]+'/pyFun')
print("sys.path: ", sys.path)
from PredictiveController import MPC

homedir = os.path.expanduser("~")    

def main():
    # Initializa ROS node
    rospy.init_node("drone_mpc_python")
    
    # Initialize parameters
    loop_rate      = 20.0
    dt             = 1.0/loop_rate
    rate           = rospy.Rate(loop_rate)
    x_start        = rospy.get_param("/segway_sim/drone_mpc_python/x_start_d")
    y_start        = rospy.get_param("/segway_sim/drone_mpc_python/y_start_d")
    exp_flag       = rospy.get_param("/segway_sim/mpc/hardware")

    optSol_pub = rospy.Publisher('/segway_sim/drone_opt', optSol, queue_size=1)
    optSol_msg = optSol()

    drone_state_pub = rospy.Publisher('/segway_sim/drone_state', stateDrone, queue_size=1)
    drone_state = stateDrone()  

    cmd_des_pub = rospy.Publisher('/uav_sim_ros/uav_cmd_des', cmd, queue_size=1)
    cmd_des = cmd()   

    # Initialize subscriber and publisher
    highLevelMsg = highLevelGoalState(x_start, y_start) # this object read the true state (not the estimated)
    droneState = droneStateMeasurement(x_start,y_start,exp_flag)
    
    ## Initialize goal state

    ## Initialize state
    xDrone = [np.array([x_start, 0, y_start, 0])]

    ## Init System Matrices
    A = np.eye(4) + dt*np.array([[0, 1, 0, 0],
                                 [0, 0, 0, 0], 
                                 [0, 0, 0, 1],
                                 [0, 0, 0, 0]])
    B = dt*np.array([[0, 0],
                     [1, 0], 
                     [0, 0],
                     [0, 1]])

    print(A)
    print(B)

    Q  = 1000* np.diag([1,.71,1,0.71])
    Qf = 100*np.diag([1,0,1,0])
    R  = 0.001*np.diag([1,1])
    dR  = 0.001*np.array([1,1])

    N = 20
    Fx = np.array([[ 0,  1, 0, 0],
                   [ 0, -1, 0, 0],
                   [ 0,  1, 0, 1],
                   [ 0,  0, 0,-1]])
    vmax = 1.5
    bx = vmax*np.ones(4)

    aMax = 5000
    Fu = np.kron(np.eye(2), np.array([1, -1])).T
    bu = np.array([[aMax],  
                   [aMax],  
                   [aMax],  
                   [aMax]]) 
    
    xRef = np.zeros(4)
    mpc  = MPC(4, 2, N, Q, R, Qf, dR, Fx, bx, Fu, bu, xRef, A, B)

    cmd_des.vDes[0] = 0.0
    cmd_des.vDes[1] = 0.0
    cmd_des.vDes[2] = 0.0
    cmd_des.vDes[3] = 0.0
    cmd_des_pub.publish(cmd_des)

    counter = 0
    while (not rospy.is_shutdown()):
        if (droneState.state != []):
            ## Solve MPC
            goalState = np.array([highLevelMsg.xGoal, 0, highLevelMsg.yGoal, 0])
            if (mpc.xRef == goalState).all():
                mpc.N = max(10, mpc.N-1)
            else:
                mpc.N    = N
                mpc.xRef = goalState

            startTimer = datetime.datetime.now()
            mpc.solve(droneState.state) 
            endTimer = datetime.datetime.now(); deltaTimer = endTimer - startTimer
            
            optSol_msg.solverTime = deltaTimer.total_seconds()

            if deltaTimer.total_seconds()>0.05:
                print("drone not real time: ", deltaTimer.total_seconds())

            if (np.abs(mpc.xPred[1,1]) <= vmax+0.05) and (np.abs(mpc.xPred[1,1]) <= vmax+0.05) and mpc.feasible == 1:
                cmd_des.vDes[0] = mpc.xPred[1,1]
                cmd_des.vDes[1] = mpc.xPred[1,3]
                cmd_des.vDes[2] = 0.0
                cmd_des.vDes[3] = 0.0
            else:
                cmd_des.vDes[0] = 0.0
                cmd_des.vDes[1] = 0.0
                cmd_des.vDes[2] = 0.0
                cmd_des.vDes[3] = 0.0

            cmd_des_pub.publish(cmd_des)
            optSol_pub.publish(optSol_msg)

            rate.sleep()
            

# ===============================================================================================================================
# ==================================================== END OF MAIN ==============================================================
# ===============================================================================================================================
class highLevelGoalState():
    def __init__(self, x_start, y_start):
        self.flag  = 0
        self.xGoal = x_start
        self.yGoal = y_start
        rospy.Subscriber("/segway_sim/droneGoalSetAndState", goalSetAndState, self.goalSetAndState_callback)

    def goalSetAndState_callback(self, msg):
        self.flag = 1
        self.xGoal = msg.x
        self.yGoal = msg.y

class droneStateMeasurement():
    def __init__(self, x_start,y_start,exp_flag):
        self.x_start = x_start
        self.y_start = y_start

        if exp_flag == 1:
            self.state = np.array([x_start, y_start, 0.0, 0.0])
            rospy.Subscriber("/uav_sim_ros/state", stateDrone, self.droneStateExp_callback)
        else:
            self.state = np.array([0.0, 0.0, 0.0, 0.0])
            rospy.Subscriber("/uav_sim_ros/state", stateDrone, self.droneState_callback)

    def droneState_callback(self, msg):
        self.state = np.array([msg.x, msg.vbx, msg.y, msg.vby])

    def droneStateExp_callback(self, msg):
        self.state = np.array([msg.x+self.x_start, msg.vbx, msg.y+self.y_start, msg.vby])

if __name__ == "__main__":

    try:    
        main()
        
    except rospy.ROSInterruptException:
        pass
