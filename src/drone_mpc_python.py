#!/usr/bin/env python

import os
import sys
import datetime
import rospy
import numpy as np
import scipy
import pdb
import sys
import time
import matplotlib.pyplot as plt
from cyberpod_sim_ros.msg import state
from uav_sim_ros.msg import state as stateDrone
from cyberpod_sim_ros.msg import valFunCnst
import scipy.io as sio
import numpy as np
# from MOMDP import MOMDP_Segway, MOMDP_Drone
from cyberpod_sim_ros.msg import goalSetAndState

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
from uav_sim_ros.msg import cmd
sys.path.append(sys.path[0]+'/pyFun')
print("sys.path: ", sys.path)
from PredictiveController import MPC

homedir = os.path.expanduser("~")    

def main():
    # Initializa ROS node
    rospy.init_node("drone_mpc_python")
    
    # Initialize parameters
    loop_rate      = 50.0
    dt             = 1.0/loop_rate
    rate           = rospy.Rate(loop_rate)
    x_start        = rospy.get_param("/segway_sim/mpc/x_start")
    y_start        = rospy.get_param("/segway_sim/mpc/y_start")

    drone_state_pub = rospy.Publisher('/segway_sim/drone_state', state, queue_size=1)
    drone_state = state()  

    cmd_des_pub = rospy.Publisher('/uav_sim_ros/uav_cmd_des', cmd, queue_size=1)
    cmd_des = cmd()   

    # Initialize subscriber and publisher
    highLevelMsg = highLevelGoalState(x_start, y_start) # this object read the true state (not the estimated)
    stateMeasurements = StatateMeasurements() # this object read the true state (not the estimated)
    droneState = droneStateMeasurement()
    
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

    Q  = 1000 * np.diag([1,0,1,0])
    Qf = 1000*np.diag([1,1,1,1])
    R  = 0.001*np.diag([1,1])
    
    K, X, eigVals = dlqr(A,B,Q,R)
    Acl = np.array(A-B*K)
    K = np.array(K)


    N = 20
    Fx = np.array([[ 0,  1, 0, 0],
                   [ 0, -1, 0, 0],
                   [ 0,  0, 0, 1],
                   [ 0,  0, 0,-1]])
    bx = 10*np.ones(4)

    aMax = 5000
    Fu = np.kron(np.eye(2), np.array([1, -1])).T
    bu = np.array([[aMax],  
                   [aMax],  
                   [aMax],  
                   [aMax]]) 
    
    xRef = np.zeros(4)
    mpc  = MPC(4, 2, N, Q, R, Qf, Fx, bx, Fu, bu, xRef, A, B)

    cmd_des.vDes[0] = 0.0
    cmd_des.vDes[1] = 0.0
    cmd_des.vDes[2] = 0.0
    cmd_des.vDes[3] = 0.0
    cmd_des_pub.publish(cmd_des)

    while (not rospy.is_shutdown()):
        if (stateMeasurements.state != []):
            ## Solve MPC
            goalState = np.array([highLevelMsg.xGoal, 0, highLevelMsg.yGoal, 0])
            mpc.xRef = goalState
            mpc.solve(droneState.state) 

            # print(xDrone[-1])
            cmd_des.vDes[0] = mpc.xPred[1,1]#-3.0*errorState[0]
            cmd_des.vDes[1] = mpc.xPred[1,3]#-3.0*errorState[2]
            cmd_des.vDes[2] = 0.0
            cmd_des.vDes[3] = 0.0
            cmd_des_pub.publish(cmd_des)

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

def dlqr(A,B,Q,R):
    """Solve the discrete time lqr controller.
     
    x[k+1] = A x[k] + B u[k]
     
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    """
    #ref Bertsekas, p.151
     
    #first, try to solve the ricatti equation
    X = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))
     
    #compute the LQR gain
    K = np.matrix(scipy.linalg.inv(B.T*X*B+R)*(B.T*X*A))
     
    eigVals, eigVecs = scipy.linalg.eig(A-B*K)
     
    return K, X, eigVals

class StatateMeasurements():
    def __init__(self):
        self.state = []
        rospy.Subscriber("/segway_sim/state_true", state, self.state_callback)

    def state_callback(self, msg):
        self.state = [msg.x, msg.y, msg.theta, msg.v, msg.thetaDot, msg.psi, msg.psiDot]

class droneStateMeasurement():
    def __init__(self):
        self.state = np.array([0.0, 0.0, 0.0, 0.0])
        rospy.Subscriber("/uav_sim_ros/state", stateDrone, self.droneState_callback)

    def droneState_callback(self, msg):
        self.state = np.array([msg.x, msg.vbx, msg.y, msg.vby])

if __name__ == "__main__":

    try:    
        main()
        
    except rospy.ROSInterruptException:
        pass
