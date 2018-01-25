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
#     starting leg
#     initial configuration
def createGlobalPlan(desired_distance, timeBetweenStep, vb, wb, feet_separation, startLeg='RLeg',q0=None):
    LegFlag = None
    if startLeg == 'RLeg':
        LegFlag = 0
    else:
        LegFlag = 1

    # Define the starting point
    xb,yb,theta = (0.0,0.0,0.0)
    if (q0 is not None):
        xb,yb,theta = q0

    timeList = []
    legList = []
    footstepList = []
    distance_traveled = 0.0
    startTime = 0.0

    if (False):
        # Assume we take a half step to start
        xb,yb,theta,increment = next_point(xb,yb,theta,vb,wb,0.5*timeBetweenStep)
        distance_traveled += increment

        # returns next leg flag, current leg, and current step
        LegFlag,leg,step = next_step(xb,yb,theta,feet_separation,LegFlag)
        startTime += timeBetweenStep # calcuate postion on 1/2 step, but assume slow start
        timeList.append(startTime)
        legList.append(leg)
        footstepList.append(step)


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
