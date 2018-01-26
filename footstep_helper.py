#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import csv


# Simple 2D homogeneous transform
def transform2D(x,y,theta):
    T = np.array([ [    np.cos(theta) , -np.sin(theta) ,  x ],
                   [    np.sin(theta) ,  np.cos(theta) ,  y ],
                   [         0.0      ,       0.0      , 1.0 ]]);
    return T;

# Plot footsteps on current axes using color dictionary
def plot_footsteps(fig, footsteps,colors={"LLeg":'b-',"RLeg":'r'}):

    # Foot boundary in foot frame (add more points for correct shape)
    left_foot  = np.array([ [-0.040, 0.040, 0.050,   0.040, -0.040, -0.040],
                            [ 0.020, 0.020, 0.000,  -0.020, -0.020,  0.020],
                            [ 1.000, 1.000, 1.000,   1.000,  1.000,  1.000]] )

    right_foot = np.array([ [-0.040, 0.040, 0.050,   0.040, -0.040, -0.040],
                            [ 0.020, 0.020, 0.000,  -0.020, -0.020,  0.020],
                            [ 1.000, 1.000, 1.000,   1.000,  1.000,  1.000]] )

    for time, x, y, theta  in footsteps["LLeg"]:
        T=transform2D(x,y,theta)
        foot = np.dot(T,left_foot)
        plt.plot(foot[0,:],foot[1,:],colors["LLeg"],linewidth=2 )
        plt.plot(x, y ,'k+',linewidth=2 )

    for time, x, y, theta  in footsteps["RLeg"]:
        T=transform2D(x,y,theta)
        foot = np.dot(T,right_foot)
        plt.plot(foot[0,:],foot[1,:],colors["RLeg"],linewidth=2 )
        plt.plot(x, y ,'k+',linewidth=2 )


# Plot footsteps on current axes using color dictionary
def plot_body(fig, body,color):
    for time, x, y, theta  in body:
        plt.plot(x,y,color,linewidth=2 )


# Return the next point given velocity and rate of rotations
# Using numerical integration to calculate position
def next_point(x0,y0,theta0,vb,wb, timeStep):

    # Initial conditions
    xb    = x0
    yb    = y0
    theta = theta0
    if (np.abs(wb) < 0.000001):
        # negligible change in orientation
        xb = xb + vb*np.cos(theta)*timeStep
        yb = yb + vb*np.sin(theta)*timeStep
    else:
        # Integrate motion to track proper arc
        t     = 0.0
        while (t < timeStep):
            xb = xb + vb*np.cos(theta)*0.001
            yb = yb + vb*np.sin(theta)*0.001
            theta = theta + wb*0.001
            t += 0.001
    return (xb,yb,theta,timeStep*vb)


# Given point along body center line, calculate the position of the current foot
def  next_step(xb,yb,theta,feet_separation,LegFlag):

    leg = None
    step = None
    dist = None

    if (LegFlag == 0):
        leg ='RLeg'
        dist = -0.5*feet_separation # along body y-axis
    else:
        leg ='LLeg'
        dist = 0.5*feet_separation


    # y-axis of the body frame of reference is along -sin(theta), cos(theta)
    # position step at 1/2 separation distance along the y-axis
    xf = xb - dist*np.sin(theta)
    yf = yb + dist*np.cos(theta)

    # toggle the leg flag for return
    return ((LegFlag+1)%2,leg,(xf,yf,theta))


# Create a global footstep plan given total distance to travel,
#
#     time step between steps (step completion time)
#     vb = forward body velocity = step_length/step time
#     wb = body rotation rate ( vb/ radius of curvature)
#     feet_separation distance
#     startLeg = initial stance leg (move other leg first)
#     q0 = initial configuration of body
#     qStance = stance foot pose in same frame as q0

def createGlobalPlan(desired_distance, timeBetweenStep, vb, wb, \
                     feet_separation, startLeg='RLeg',q0=None, qStance = None):
    LegFlag = None
    LegName = None

    # This is the initial stance foot (start moving the other one)
    if startLeg == 'RLeg':
        LegFlag = 0
        LegName = 'RLeg'
        dist    = -0.5*feet_separation
    else:
        LegFlag = 1
        LegName = 'LLeg'
        dist    = 0.5*feet_separation

    # Define the starting point of the body position
    xb,yb,theta = (0.0,0.0,0.0)
    if (q0 is not None):
        print "  plan from given start pose=",q0
        xb,yb,theta = q0
        if (qStance is None):
            dummy,leg, qStance = next_step(xb,yb,theta,feet_separation,LegFlag)
            print "  set qStance if not given =",qStance
        else:
            print "  given qStance =",qStance, " with defined body pose"
    else:
        print "Set initial stance foot pose relative to origin ..."
        if (qStance is None):
            qStance =(0.0, dist, 0.0)
            print "  set qStance if not given =",qStance
        else:
            print "  given qStance =",qStance, " with initial body at origin"

    timeList = []
    legList = []
    footstepList = []
    distance_traveled = 0.0
    startTime = 0.0

    # Assume we list the initial stance foot as the first point
    LegFlag = (LegFlag+1)%2 # increment from stance foot
    timeList.append(startTime)
    legList.append(LegName)
    footstepList.append(qStance)


    limit_distance =  desired_distance - 0.5*timeBetweenStep*vb

    while distance_traveled < limit_distance:
        # Move to next reference point
        xb,yb,theta,increment = next_point(xb,yb,theta,vb,wb,timeBetweenStep)
        distance_traveled += increment

        # Calculate proper foot position
        LegFlag,leg,step = next_step(xb,yb,theta,feet_separation,LegFlag)

        # Update the vectors
        startTime += timeBetweenStep
        timeList.append(startTime)
        legList.append(leg)
        footstepList.append(step)

    # At the end of the walking, add another 1/2 step to bring legs together
    #   keep same reference point, just bring other foot up
    LegFlag,leg,step = next_step(xb,yb,theta,feet_separation,LegFlag)
    startTime += timeBetweenStep # calcuate postion on 1/2 step, but assume slow finish
    timeList.append(startTime)
    legList.append(leg)
    footstepList.append(step)

    return [legList, footstepList, timeList]


def writeGlobalPlan(leg_array, footstep_array, time_array, experiment_dir, test_dir, filename='global-plan.csv' ):
    with open(experiment_dir+'/'+ test_dir + '/'+filename, 'w+') as csvFile:
        writer = csv.writer(csvFile, delimiter=',')
        writer.writerow([test_dir])
        writer.writerow(['Leg Name', 'Execution Time', 'X', 'Y', 'Theta'])
        length = len(leg_array)
        for x in range(0, length):
            writer.writerow([ leg_array[x], time_array[x], footstep_array[x][0], footstep_array[x][1], footstep_array[x][2] ])


def getLocalPlan(globalLegName, globalFootSteps, globalTimeList, \
                         iStance, qStance, startIndex, endIndex):

    # Stance foot in internal world frame
    Tstance =transform2D(qStance[0],qStance[1],qStance[2])

    # Corresponding stance foot in plan
    Tplan   = transform2D(globalFootSteps[iStance][0],
                          globalFootSteps[iStance][1],
                          globalFootSteps[iStance][2])


    Ti = np.linalg.inv(Tplan)

    # Put actual stance in the plan stance frame (this is accumulated error)
    Terror = np.dot(Ti,Tstance)

    qError = (Terror[0][2],Terror[1][2],np.arctan2(Terror[1][0],Terror[0][0]))
    print " Step Error = ",qError

    # Should sanity check this error
    err = np.sqrt(qError[0]*qError[0] + qError[1]*qError[1])
    if (err > 0.06 or np.abs(qError[2]) > 0.2):
        print "     error is large - err=",err," -- ignore correction!"
        qError = (0., 0., 0.)

    # Get pose of the last unchangeable (startIndex > 0 assumed)
    Tprior   = transform2D(globalFootSteps[startIndex-1][0],
                           globalFootSteps[startIndex-1][1],
                           globalFootSteps[startIndex-1][2])
    Ti = np.linalg.inv(Tprior)

    localLegName=[]
    localFootSteps=[]
    localTimeList = []

    fraction = 0.0
    if (endIndex > startIndex):
        fraction = 1.0/(endIndex-startIndex)

    for ndx in range(startIndex,endIndex):
        Tnext = transform2D(globalFootSteps[ndx][0],
                            globalFootSteps[ndx][1],
                            globalFootSteps[ndx][2])

        Trelative = np.dot(Ti,Tnext)
        Ti = np.linalg.inv(Tnext) # update for the next calc

        # Extract the relevant data
        qRelative = [Trelative[0][2],Trelative[1][2],np.arctan2(Trelative[1][0],Trelative[0][0])]

        # Apply a portion of correction to each step
        qRelative[0] += fraction*qError[0]
        qRelative[1] += fraction*qError[1]
        qRelative[2] += fraction*qError[2]

        localLegName.append(globalLegName[ndx])
        localTimeList.append(globalTimeList[ndx])
        localFootSteps.append(qRelative)

        dist = np.sqrt(qRelative[0]*qRelative[0] + qRelative[1]*qRelative[1])
        print "  relative step = ",qRelative, " dist=",dist

    return (localLegName, localFootSteps, localTimeList)


def createLocalPlanFromGlobal(legList, footstepList, timeList):
    length = len(legList)
    LegFlag = None
    _legList = []
    _footstepList = []
    _timeList = []
    y_value = footstepList[0][1]
    if legList[0] == 'RLeg':
        LegFlag = 0
    else:
        LegFlag = 1
    for it in range(0, length):
        x = 0
        if LegFlag == 0:
            _legList.append('RLeg')
            if it == 0:
                x = footstepList[it][0]
                theta = footstepList[it][0]
                _footstepList.append([x, -y_value, theta])
            else:
                x = footstepList[it][0] - footstepList[it-1][0]
                theta = footstepList[it][2] - footstepList[it-1][2]
                _footstepList.append([x, -y_value, theta])
            LegFlag = 1
        elif LegFlag == 1:
            _legList.append('LLeg')
            if it == 0:
                x = footstepList[it][0]
                theta = footstepList[it][0]
                _footstepList.append([x, -y_value, theta])
            else:
                x = footstepList[it][0] - footstepList[it-1][0]
                theta = footstepList[it][2] - footstepList[it-1][2]
                _footstepList.append([x, y_value, theta])
            LegFlag = 0
    return [_legList, _footstepList, timeList]
