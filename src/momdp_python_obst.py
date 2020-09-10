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
from segway_sim.msg import state
from segway_sim.msg import goalSetAndState
import scipy.io as sio
import numpy as np
from MOMDP import MOMDP_obst

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
    publisherObst = rospy.Publisher('/segway_sim/visualization_marker', Marker)
    markerObst = initMarkeObst()

    publisher = rospy.Publisher('/segway_sim/visualization_marker_array', MarkerArray)
    markerArray = MarkerArray()

    publisherBelief = rospy.Publisher('/segway_sim/visualization_marker_array_belief', MarkerArray)
    markerArrayBelief = MarkerArray()

    count = 0
    MARKERS_MAX = 100


    # Initialize parameters
    loop_rate      = 100.0
    dt             = 1.0/loop_rate
    rate           = rospy.Rate(loop_rate)
    x_start        = rospy.get_param("/segway_sim/mpc/x_start")
    y_start        = rospy.get_param("/segway_sim/mpc/y_start")

    # Initialize subscriber and publisher
    statateMeasurements = StatateMeasurements(x_start, y_start) # this object read the true state (not the estimated)

    testToTry      = rospy.get_param("/segway_sim/momdp_python/testToTry")
    
    goalSetAndState_pub = rospy.Publisher('/segway_sim/goalSetAndState', goalSetAndState, queue_size=1)
    goalSetAndStateMsg  = goalSetAndState()   

    # Import matrices for MOMDP
    M, Px, V, J, gridVar = readData(testToTry)


    xt         = [0]
    t          = 0
    verbose    = 0
    at = []

    # Initialize
    momdp = MOMDP_obst(Px, M, V, J, gridVar, verbose)
    unobsState   = 0;
    testParam_time = [   2,  9,  13, 50];
    testParam_dir  = [ 's', 'd', 'u','s'];
    obstBelief = np.array([1.0, 1.0, 0.0, 0, 0, 0]);
    obstOrder=[0,1,2,3,4,5]

    bt = momdp.initBeliefAndZ(obstBelief, unobsState)

    [action, coordXY, coordCo, boxConstraints, boxNext] = momdp.updateMOMDP(t, xt[-1], bt[-1]);

    at.append(action)
    xt.append(momdp.propagate(xt[-1], momdp.zt, at[-1]))
    bt.append( momdp.updateBelief(xt[-2], xt[-1], at[-1], unobsState, bt[-1]) )

    # Initialize marker array
    colorMarker = [0.0, 1.0, 0.0]
    markerArray.markers.append(initMarker(momdp, 2, 0, obstOrder, colorMarker))
    colorMarker = [0.0, 1.0, 0.0]
    markerArray.markers.append(initMarker(momdp, 2, 0, obstOrder, colorMarker))

    colorMarker = [0.0, 0.0, 1.0]
    [markerArrayBelief.markers.append(initMarkerBelief(momdp, 0, i, obstOrder, colorMarker)) for i in range(0, momdp.numUnKnownObst)]
    colorMarker = [1.0, 1.0, 0.0]
    [markerArray.markers.append(initMarker(momdp, 1, i, obstOrder, colorMarker)) for i in range(0, momdp.numKnownObst)]

    
    idx = 0
    for m in markerArray.markers:
        m.id = idx
        idx += 1

    print("idx: ", idx)

    idx = 0
    for m in markerArrayBelief.markers:
        m.id = idx
        idx += 1

    obstBelief = []
    [obstBelief.append( 1- bt[-2][i] ) for i in range(0, momdp.numUnKnownObst)]

    counter = 0
    for obsPr in obstBelief:
        markerArrayBelief.markers[counter].color.a = 1 - obsPr
        counter += 1

    print("Obst 0: ", obstBelief[0])
    print("Obst 1: ", obstBelief[1])
    print("Obst 2: ", obstBelief[2])
    obstPos = [unobsState];
    # Initialize parameters for while loop
    initFlag = 0
    t=+1
    print("bt ", bt[-1])
    while (not rospy.is_shutdown()):
        if statateMeasurements.state != []:
            if initFlag == 0:
                goalSetAndState_pub.publish(goalSetAndStateMsg) 
                publisher.publish(markerArray)
                publisherBelief.publish(markerArrayBelief)
                markerObst.pose.position.y = 6 - obstPos[-1] - 0.5
                publisherObst.publish(markerObst)

                initFlag = 1
                print("======== Pub")


            goalSetAndStateMsg = pubHighLevel(goalSetAndStateMsg, coordXY, coordCo, boxConstraints, boxNext, t)
            goalSetAndState_pub.publish(goalSetAndStateMsg) 
            # read measurement
            xCurr = statateMeasurements.state

            if (xt[-1] != momdp.goal ) and (xCurr[0] >= boxNext[0]) and (xCurr[0] <= boxNext[1]) and (xCurr[1] >= boxNext[2]) and (xCurr[1] <= boxNext[3]):
                
                [action, coordXY, coordCo, boxConstraints, boxNext] = momdp.updateMOMDP(t, xt[-1], bt[-1]);
                at.append(action)

                # print("======== Pub")
                print("Curr state  ", xt[-1])

                xt.append(momdp.propagate(xt[-1], obstPos[-1], at[-1]))
                obstPos.append(moveObstacle(t, obstPos[-1], testParam_time, obstBelief, testParam_dir))


                goalSetAndStateMsg = pubHighLevel(goalSetAndStateMsg, coordXY, coordCo, boxConstraints, boxNext, t)
                goalSetAndState_pub.publish(goalSetAndStateMsg) 
                print("Selected Action: ", action)

                # Renumber the marker IDs
                obstBelief = []
                [obstBelief.append( momdp.computeObstacleBelief(bt[-1], i) ) for i in range(0, momdp.numUnKnownObst)]


                counter = 0
                for obsPr in obstBelief:
                    markerArrayBelief.markers[counter].color.a = 1-obsPr
                    counter += 1

                publisher.publish(markerArray)
                publisherBelief.publish(markerArrayBelief)
                # Publish the MarkerArray

                oMeas = obstPos[-1]# we measure always zt
                bt.append( momdp.updateBelief(xt[-2], xt[-1], at[-1], oMeas, bt[-1]) )
                # print("Obstacle State: ", obstPos)
                # print("Next state  ", xt[-1])
                # print("bt ", bt[-2])
                print("bt ", bt[-1])

                markerObst.pose.position.y = 6 - obstPos[-2] - 0.5
                publisherObst.publish(markerObst)
                t += 1


            rate.sleep()
            

# ===============================================================================================================================
# ==================================================== END OF MAIN ==============================================================
# ===============================================================================================================================
# def updateMarkerBelief(momdp, i, bt):


def initMarkeObst():
    ## Visualization
    colorMarker = [0.0, 0.0, 1.0]

    cubeLength = 0.3;
    cubeHeigth = 0.3;

    marker = Marker()
    marker.header.frame_id = "/world"
    marker.header.stamp = rospy.Time.now()
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = cubeLength
    marker.scale.y = cubeLength
    marker.scale.z = cubeHeigth
    marker.color.a = 1.0
    marker.color.r = colorMarker[0]
    marker.color.g = colorMarker[1]
    marker.color.b = colorMarker[2]
    marker.pose.orientation.w = 1.0

    marker.pose.position.x = 4.5
    marker.pose.position.y = 5.5
    marker.pose.position.z = 0.0
    return marker

def initMarkerBelief(momdp, knowObst, i, obstOrder, colorMarker):
    ## Visualization
    if knowObst<2:
        cubeLength = 1.0;
        cubeHeigth = 0.05;
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
    marker.scale.z = 0.0
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
    elif knowObst == 1:
        obstCoordinate = np.where( (momdp.gridVar < 0) )
        print("=========================================== obstCoordinate: ", obstCoordinate)
        idx = i
    elif knowObst == 2:
        obstCoordinate = np.where( (momdp.gridVar == 1) )
        idx = 0

    marker.pose.position.x = obstCoordinate[1][idx] + momdp.cellWidth/2
    marker.pose.position.y = np.shape(momdp.gridVar)[0] - obstCoordinate[0][idx] - momdp.cellHight/2.0
    marker.pose.position.z = 0
    return marker

def moveObstacle(t, unobservableState, testParam_time, obstBelief, testParam_dir):
    stateTransitions = [ max([unobservableState-1, 0]), unobservableState, min([unobservableState+1,np.shape(obstBelief)[0]-1])];

    idx = []
    for i in range(0,len(testParam_time)):
        if testParam_dir[i] == 'd': idx.append(2)
        if testParam_dir[i] == 'u': idx.append(0)
        if testParam_dir[i] == 's': idx.append(1)

    flag = 0;
    for i in range(0,len(testParam_time)):
        if (t < testParam_time[i]) and (flag == 0):
            nextUnobservableState = stateTransitions[idx[i]];
            flag = 1;
            
    return nextUnobservableState
def readData(testToTry):
    folderToOpen = 'movingObstacle'

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

    print(gridVar)

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
    def __init__(self, x_start, y_start):
        self.state = []
        self.x_start = x_start*0
        self.y_start = y_start*0
        rospy.Subscriber("/segway_sim/state_true", state, self.state_callback)

    def state_callback(self, msg):
        self.state = [msg.x + self.x_start, msg.y + self.y_start, msg.theta, msg.v, msg.thetaDot, msg.psi, msg.psiDot]

if __name__ == "__main__":

    try:    
        main()
        
    except rospy.ROSInterruptException:
        pass
