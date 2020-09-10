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

from segway_sim.msg import goalSetAndState
import scipy.io as sio
import numpy as np
from MOMDP import MOMDP

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math


homedir = os.path.expanduser("~")    

def main():
    # Initializa ROS node
    rospy.init_node("momdp_python")
    
    count  = 0
    # publisher = rospy.Publisher('/segway_sim/visualization_marker', Marker)
    publisher = rospy.Publisher('/segway_sim/visualization_marker_array', MarkerArray)
    markerArray = MarkerArray()

    count = 0
    MARKERS_MAX = 100


    # Initialize parameters
    loop_rate      = 100.0
    dt             = 1.0/loop_rate
    rate           = rospy.Rate(loop_rate)
    x_start        = rospy.get_param("mpc/x_start")
    y_start        = rospy.get_param("mpc/y_start")
    expFlag        = rospy.get_param("momdp_python/expFlag")

    if expFlag == 1:
        from ambercortex_ros.msg import state
    else:
        from segway_sim.msg import state

    # Initialize subscriber and publisher
    statateMeasurements = StatateMeasurements(x_start, y_start, expFlag, state) # this object read the true state (not the estimated)

    testToTry      = rospy.get_param("momdp_python/testToTry")
    
    goalSetAndState_pub = rospy.Publisher('goalSetAndState', goalSetAndState, queue_size=1)
    goalSetAndStateMsg  = goalSetAndState()   

    # Import matrices for MOMDP
    M, Px, V, J, gridVar = readData(testToTry)


    # Init Environment and
    if testToTry == -1:
        obstOrder  = [1, 0, 2]
        loc        = '101'
        initBelief = [0.75, 0.6, 0.9]
        # loc        = '110'
        # initBelief = [0.25, 0.45, 0.1]
    elif testToTry == -2:
        obstOrder  = [0, 1]
        loc        = '00'
        initBelief = [0.6, 0.9]
    elif testToTry == -3:
        obstOrder  = [1,0]
        loc        = '00'
        initBelief = [0.6, 0.9]
    else:
        obstOrder  = [0]
        loc        = '1'
        initBelief = [0.9]

    xt         = [0]
    t          = 0
    verbose    = 0
    at = []

    # Initialize
    momdp = MOMDP(Px, M, V, J, gridVar, verbose)
    bt = momdp.initBeliefAndZ(loc, initBelief)

    [action, coordXY, coordCo, boxConstraints, boxNext] = momdp.updateMOMDP(t, xt[-1], bt[-1]);

    at.append(action)
    xt.append(momdp.propagate(xt[-1], momdp.zt, at[-1]))
    t =+ 1

    # Initialize marker array
    colorMarker = [1.0, 1.0, 0.0]
    [markerArray.markers.append(initMarker(momdp, 0, i, obstOrder, colorMarker)) for i in range(0, momdp.numUnKnownObst)]
    [markerArray.markers.append(initMarker(momdp, 1, i, obstOrder, colorMarker)) for i in range(0, momdp.numKnownObst)]
    colorMarker = [0.0, 1.0, 0.0]
    markerArray.markers.append(initMarker(momdp, 2, 0, obstOrder, colorMarker))
    idx = 0
    for m in markerArray.markers:
        m.id = idx
        idx += 1


    obstBelief = []
    [obstBelief.append( momdp.computeObstacleBelief(bt[-1], i) ) for i in range(0, momdp.numUnKnownObst)]

    counter = 0
    for obsPr in obstBelief:
        markerArray.markers[counter].color.a = 1 - obsPr
        counter += 1

    # print("Obst 0: ", obstBelief[0])
    # print("Obst 1: ", obstBelief[1])
    # print("Obst 2: ", obstBelief[2])

    # Initialize parameters for while loop
    initFlag = 0
    while (not rospy.is_shutdown()):
        if statateMeasurements.state != []:
            if initFlag == 0:
                goalSetAndState_pub.publish(goalSetAndStateMsg) 
                publisher.publish(markerArray)

                initFlag = 1
                print("======== Pub")


            goalSetAndStateMsg = pubHighLevel(goalSetAndStateMsg, coordXY, coordCo, boxConstraints, boxNext, t)
            goalSetAndState_pub.publish(goalSetAndStateMsg) 
            # read measurement
            xCurr = statateMeasurements.state

            if (xt[-1] != momdp.goal ) and (xCurr[0] >= boxNext[0]) and (xCurr[0] <= boxNext[1]) and (xCurr[1] >= boxNext[2]) and (xCurr[1] <= boxNext[3]):
                
                [action, coordXY, coordCo, boxConstraints, boxNext] = momdp.updateMOMDP(t, xt[-1], bt[-1]);
                at.append(action)
                xt.append(momdp.propagate(xt[-1], momdp.zt, at[-1]))


                goalSetAndStateMsg = pubHighLevel(goalSetAndStateMsg, coordXY, coordCo, boxConstraints, boxNext, t)
                goalSetAndState_pub.publish(goalSetAndStateMsg) 
                t += 1

                print("======== Pub")

                # Renumber the marker IDs
                obstBelief = []
                [obstBelief.append( momdp.computeObstacleBelief(bt[-1], i) ) for i in range(0, momdp.numUnKnownObst)]
                # print("Obst 0: ", obstBelief[0])
                # print("Obst 1: ", obstBelief[1])
                # print("Obst 2: ", obstBelief[2])

                counter = 0
                for obsPr in obstBelief:
                    markerArray.markers[counter].color.a = 1-obsPr
                    counter += 1

                # Publish the MarkerArray
                publisher.publish(markerArray)

                oMeas = momdp.zt # we measure always zt
                bt.append( momdp.updateBelief(xt[-2], xt[-1], at[-1], oMeas, bt[-1]) )

                # publisher.publish(marker)


            rate.sleep()
            

# ===============================================================================================================================
# ==================================================== END OF MAIN ==============================================================
# ===============================================================================================================================
# def updateMarkerBelief(momdp, i, bt):


def initMarker(momdp, knowObst, i, obstOrder, colorMarker):
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
        obstCoordinate = np.where( (momdp.gridVar < 1) & (momdp.gridVar > 0) )
        idx = obstOrder[i]
        print("obstCoordinate: ", obstCoordinate)
    elif knowObst == 1:
        obstCoordinate = np.where( (momdp.gridVar < 0) )
        idx = i
    elif knowObst == 2:
        obstCoordinate = np.where( (momdp.gridVar == 1) )
        idx = 0

    marker.pose.position.x = obstCoordinate[1][idx] + momdp.cellWidth/2
    marker.pose.position.y = np.shape(momdp.gridVar)[0] - obstCoordinate[0][idx] - momdp.cellHight/2.0
    marker.pose.position.z = 0
    return marker

def readData(testToTry):
    folderToOpen = 'test_5x5'
    if testToTry == 1:
        folderToOpen = '2x2_test_1'
    elif testToTry == 2:
        folderToOpen = '2x3_test_2'
    elif testToTry == 3:
        folderToOpen = '2x3_test_3'
    elif testToTry == -2:
        folderToOpen = '7x7'
    elif testToTry == -3:
        folderToOpen = '10x10_test'

    print("Folder to open: ", folderToOpen)

    # Propagation belief matrix M[action, xCurrent, xNext, Observation]
    i = sio.loadmat(sys.path[0] + '/../matFiles/'+folderToOpen+'/M.mat')
    M = i['M'] 

    # Action matrix xNext = Px[action, unobservableState] * xCurr
    i  = sio.loadmat(sys.path[0] + '/../matFiles/'+folderToOpen+'/Px.mat')
    Px = i['Px'] 

    # Action matrix xNext = Px[action, unobservableState] * xCurr
    i  = sio.loadmat(sys.path[0] + '/../matFiles/'+folderToOpen+'/V.mat')
    V  = i['V_app'] 
    i  = sio.loadmat(sys.path[0] + '/../matFiles/'+folderToOpen+'/J.mat')
    J  = i['J_app'] 

    i        = sio.loadmat(sys.path[0] + '/../matFiles/'+folderToOpen+'/gridVar.mat')
    gridVar  = i['gridVar'] 

    return M, Px, V, J, gridVar

def pubHighLevel(goalSetAndStateMsg, coordXY, coordCo, boxConstraints, boxNext, t):

    goalSetAndStateMsg.x = (coordXY[1][0] + coordXY[2][0])/2
    goalSetAndStateMsg.y = (coordXY[1][1] + coordXY[2][1])/2

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
