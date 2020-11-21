#!/usr/bin/env python

import os
import datetime
import rospy
import numpy as np
import scipy
import pdb
import sys
import time
import matplotlib.pyplot as plt
import pickle
import sys
from segway_sim.msg import highLevelBelief


sys.path.append(sys.path[0]+'/pyFun')
print("sys.path: ", sys.path)

from segway_sim.msg import goalSetAndState
import scipy.io as sio
import numpy as np
from MOMDP import MOMDP_TOQ
from main import getMOMDP

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
homedir = os.path.expanduser("~")    

def main():
    # Initializa ROS node
    rospy.init_node("momdp")

    # Initialize node parameters
    loop_rate      = 100.0
    dt             = 1.0/loop_rate
    rate           = rospy.Rate(loop_rate)
    x_start        = rospy.get_param("mpc/x_start")
    y_start        = rospy.get_param("mpc/y_start")
    expFlag        = rospy.get_param("momdp_node/expFlag")

    if expFlag == 1:
        from ambercortex_ros.msg import state
    else:
        from segway_sim.msg import state

    # Initialize subscriber and publisher
    statateMeasurements = StatateMeasurements(x_start, y_start, expFlag, state) # this object read the true state (not the estimated)
    publisher = rospy.Publisher('/segway_sim/visualization_marker_array', MarkerArray)
    markerArray = MarkerArray()
    goalSetAndState_pub = rospy.Publisher('goalSetAndState', goalSetAndState, queue_size=1)
    publisherBelief     = rospy.Publisher('/segway_sim/highLevelBelief', highLevelBelief)
    goalSetAndStateMsg  = goalSetAndState()   
    msghighLevelBelief  = highLevelBelief()

    option = 1
    if option == 1:
        # Import MOMDP
        fileName  = sys.path[0]+'/pyFun/data/Qug_1/MOMDP_obj_8x8ug_2.pkl'
        pickle_in = open(fileName,"rb")
        momdp     = pickle.load(pickle_in)

        # Init Environment
        loc        = (0,0,0,1)
        initBelief = [0.4, 0.6, 0.5,0.9]
    elif option == 2:
        # Import MOMDP
        fileName  = sys.path[0]+'/pyFun/data/TOQug_1/MOMDP_obj_10x10ug_2.pkl'
        pickle_in = open(fileName,"rb")
        momdp     = pickle.load(pickle_in)

        # Init Environment
        loc        = (1,0,0,1)
        initBelief = [0.9, 0.3, 0.2,0.4]
    elif option == 3:
        # Import MOMDP
        fileName  = sys.path[0]+'/pyFun/data/TOQ_1/MOMDP_obj_5x5_3.pkl'
        pickle_in = open(fileName,"rb")
        momdp     = pickle.load(pickle_in)

        # Init Environment
        loc        = (1,1,0)
        initBelief = [0.9, 0.3, 0.2]
    else:
        gridWorld = '10x10ug'
        numObst = 3
        policy = 'TOQ'
        printLevel = 0
        discOpt = 1
        momdp   = getMOMDP(gridWorld, numObst, policy, printLevel, -1, discOpt, unGoal = True)
        # Init Environment
        loc        = (1, 0, 0, 0, 1)
        initBelief = [0.7, 0.5, 0.4, 0.2,0.4]

    xt         = [0]
    t          = 0
    verbose    = 0
    at = []

    # Initialize MOMDP
    momdp.initZ(loc)
    momdp.printLevel = 0
    bt = [momdp.initBelief(initBelief)]
    # Update MOMDP
    [action, coordXY, boxConstraints, boxNext] = momdp.updateMOMDP(t, xt[-1], bt[-1]);
    at.append(action)
    xt.append(momdp.propagate(xt[-1], momdp.zt, at[-1]))
    t =+ 1

    # Initialize marker array for obstacles
    markerArray = initMarkerArray(markerArray, momdp)
    # Update \alpha for obstacles: set \alpha = 1-prob beeing free
    markerArray, obstBelief = updateObstacles(markerArray, momdp, bt[-1])
    pSuccess = np.max(np.dot(momdp.J[t-1, xt[-1]].T, bt[-1])) # This is the probability of success
    msghighLevelBelief.probMiss = pSuccess
    for i in range(0, bt[-1].shape[0]):
        msghighLevelBelief.bt[i] = bt[-1][i]

    for i in range(0, len(obstBelief)):
        msghighLevelBelief.prob[i] = obstBelief[i]

    msghighLevelBelief.targetPosSegway[0] = goalSetAndStateMsg.x
    msghighLevelBelief.targetPosSegway[1] = goalSetAndStateMsg.y
    publisherBelief.publish(msghighLevelBelief)

    # Initialize parameters for while loop
    initFlag = 0
    goalSetAndStateMsg = pubHighLevel(goalSetAndStateMsg, coordXY, boxConstraints, boxNext, t, forecast = True)
    
    if expFlag == 1:
        for i in range(0,500):
            rate.sleep()

    # goalSetAndState_pub.publish(goalSetAndStateMsg) # Uncomment this only for testing high-mid connection


    print("ready to start!!!")
    while (not rospy.is_shutdown()):
        if statateMeasurements.state != []:
            if initFlag == 0: # Publish env position to initialize rviz
                goalSetAndState_pub.publish(goalSetAndStateMsg) 
                publisher.publish(markerArray)
                initFlag = 1
                print("======== Pub")

            # read measurement
            xCurr = statateMeasurements.state
            # Check if goal reached or reached the goal cell
            if (xt[-1] != momdp.goal ) and (xCurr[0] >= boxNext[0]) and (xCurr[0] <= boxNext[1]) and (xCurr[1] >= boxNext[2]) and (xCurr[1] <= boxNext[3]):
                # publish belief
                [action, coordXY, boxConstraints, boxNext] = momdp.updateMOMDP(t, xt[-1], bt[-1]);
                at.append(action)
                xt.append(momdp.propagate(xt[-1], momdp.zt, at[-1]))
                t += 1
                goalSetAndStateMsg = pubHighLevel(goalSetAndStateMsg, coordXY, boxConstraints, boxNext, t, forecast = True)
                goalSetAndState_pub.publish(goalSetAndStateMsg) 

                # Update \alpha for obstacles: set \alpha = 1-prob beeing free
                markerArray, obstBelief = updateObstacles(markerArray, momdp, bt[-1])
                # print("obstBelief: ",obstBelief)
                # Publish the MarkerArray
                publisher.publish(markerArray)
                pSuccess = np.max(np.dot(momdp.J[t-1, xt[-1]].T, bt[-1])) # This is the probability of success
                msghighLevelBelief.probMiss = pSuccess
                for i in range(0, bt[-1].shape[0]):
                    msghighLevelBelief.bt[i] = bt[-1][i]

                for i in range(0, len(obstBelief)):
                    msghighLevelBelief.prob[i] = obstBelief[i]

                msghighLevelBelief.targetPosSegway[0] = goalSetAndStateMsg.x
                msghighLevelBelief.targetPosSegway[1] = goalSetAndStateMsg.y
                publisherBelief.publish(msghighLevelBelief)


                # Update belief
                oMeas = momdp.getObservation(xt[-1], 3)
                bt.append( momdp.updateBelief(xt[-2], xt[-1], at[-1], oMeas, bt[-1]) )


            rate.sleep()
            
# ===============================================================================================================================
# ==================================================== END OF MAIN ==============================================================
# ===============================================================================================================================
def initMarkerArray(markerArray, momdp):
    colorMarker = [0.0, 0.0, 1.0]
    colorMarker = [0.9, 0.6, 0.3]
    [markerArray.markers.append(initMarker(momdp, 0, i, colorMarker)) for i in range(0, momdp.numUnKnownObst)]
    # Initialize marker array for goals
    colorMarker = [0.0, 1.0, 0.0]
    if momdp.unGoal == False:
        markerArray.markers.append(initMarker(momdp, 2, 0, colorMarker))
    else:
        [markerArray.markers.append(initMarker(momdp, 2, i, colorMarker)) for i in range(0, momdp.numUGoal)]
    
    colorMarker = [1.0, 1.0, 0.0]
    colorMarker = [0.5, 0.0, 0.0]
    [markerArray.markers.append(initMarker(momdp, 1, i, colorMarker)) for i in range(0, momdp.numKnownObst)]
    # Set idx for marker array
    idx = 0
    for m in markerArray.markers:
        m.id = idx
        idx += 1
    return markerArray

def updateObstacles(markerArray, momdp, bt):
    obstBelief = []
    [obstBelief.append( momdp.computeBelief(bt, i) ) for i in range(0, momdp.numUnKnownObst)]
    counter = 0
    for obsPr in obstBelief:
        markerArray.markers[counter].color.a = 1-obsPr
        if obsPr<=0.0001:
            markerArray.markers[counter].color.r = 0.5
            markerArray.markers[counter].color.g = 0
            markerArray.markers[counter].color.b = 0

        counter += 1

    if momdp.unGoal == True:
        goalBeliefGoal = []
        [goalBeliefGoal.append( momdp.computeBelief(bt, i+momdp.numObs) ) for i in range(0, momdp.numUGoal)]
        for obsPr in goalBeliefGoal:
            obstBelief.append(obsPr)
            markerArray.markers[counter].color.a = 1-obsPr
            counter += 1

    return markerArray, obstBelief

def initMarker(momdp, knowObst, idx, colorMarker):
    ## Visualization
    if knowObst<2:
        cubeLength = 0.2;
        cubeHeigth = 0.2;
    else:
        cubeLength = 1.0;
        cubeHeigth = 0.00000001;

    marker = Marker()
    marker.header.frame_id = "/world"
    marker.header.stamp = rospy.Time.now()
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = cubeLength
    marker.scale.y = cubeLength
    marker.scale.z = cubeHeigth
    marker.color.a = 1.0
    marker.color.r = colorMarker[0]
    marker.color.g = colorMarker[1]
    marker.color.b = colorMarker[2]
    marker.pose.orientation.w = 1.0

    if knowObst == 0:
        obstCoordinate = (momdp.row_obs, momdp.col_obs)
    elif knowObst == 1:
        obstCoordinate = np.where( (momdp.gridVar < 0) )
    elif knowObst == 2:
        if momdp.unGoal == True:
            obstCoordinate = (momdp.row_goal, momdp.col_goal)
        else:              
            obstCoordinate = np.where( (momdp.gridVar == 1) )

    marker.pose.position.x = obstCoordinate[1][idx] + momdp.cellWidth/2
    marker.pose.position.y = np.shape(momdp.gridVar)[0] - obstCoordinate[0][idx] - momdp.cellHight/2.0
    marker.pose.position.z = 0
    return marker

def pubHighLevel(goalSetAndStateMsg, coordXY, boxConstraints, boxNext, t, forecast = True):
    if forecast == True and len(coordXY) > 2:
        goalSetAndStateMsg.x = (coordXY[1][0] + coordXY[2][0])/2
        goalSetAndStateMsg.y = (coordXY[1][1] + coordXY[2][1])/2
    else:
        goalSetAndStateMsg.x = coordXY[1][0]
        goalSetAndStateMsg.y = coordXY[1][1]
    
    goalSetAndStateMsg.xmin = boxConstraints[0]
    goalSetAndStateMsg.xmax = boxConstraints[1]
    goalSetAndStateMsg.ymin = boxConstraints[2]
    goalSetAndStateMsg.ymax = boxConstraints[3]   

    goalSetAndStateMsg.term_xmin = boxNext[0]
    goalSetAndStateMsg.term_xmax = boxNext[1]
    goalSetAndStateMsg.term_ymin = boxNext[2]
    goalSetAndStateMsg.term_ymax = boxNext[3]   

    goalSetAndStateMsg.highLevTime = t   

    return goalSetAndStateMsg


class StatateMeasurements():
    def __init__(self, x_start, y_start, expFlag, state):
        self.state = []
        self.x_start = x_start
        self.y_start = y_start
        if expFlag == 1:
            rospy.Subscriber("state", state, self.state_callback_exp)
        else:
            rospy.Subscriber("state_true", state, self.state_callback_sim)

    def state_callback_exp(self, msg):
        self.state = [msg.state[0]+self.x_start, msg.state[1]+self.y_start, msg.state[2], msg.state[3], msg.state[4], msg.state[5], msg.state[6]]

    def state_callback_sim(self, msg):
        self.state = [msg.x, msg.y, msg.theta, msg.v, msg.thetaDot, msg.psi, msg.psiDot]

if __name__ == "__main__":

    try:    
        main()
        
    except rospy.ROSInterruptException:
        pass
