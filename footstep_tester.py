# -*- encoding: UTF-8 -*-

import time
import datetime
import os
import argparse
from footstep_helper import *


def main(filename):
    experiment_dir = 'experiment_data'
    test_dir = ""

    num_steps_to_send = 4
    num_steps_straight_plan = 10
    time_between_step = 0.6
    start_leg = 'LLeg'

    timeBetweenStep  = 0.6
    desiredStepIncrement = 0.06
    vb = desiredStepIncrement/timeBetweenStep
    wb = 0.0 # Straight line walking
    desired_distance = 10*0.06
    rho = None
    #rho=-(4.0*12.0*0.0254) #radius of curvature (ft to meters)
    if (rho is not None):
        wb = vb/rho # steer to
        desired_distance = (np.pi/2.0)*np.abs(rho)

    feet_separation = 0.10
    startLeg = 'LLeg'
    q0 = None
    globalLegName, globalFootSteps, globalTimeList = \
        createGlobalPlan(desired_distance, timeBetweenStep,
                         vb, wb, feet_separation,startLeg,q0)




    writeGlobalPlan(globalLegName, globalFootSteps, globalTimeList, experiment_dir, test_dir, filename)

    footstep_count = 0   # first foot in unchangeable (current stance foot)
    num_steps_to_send = 4
    len_unchangeable = 3
    num_steps_in_plan = len(globalFootSteps)
    startIndex = 0

    while (startIndex < len(globalFootSteps)):
        startIndex = footstep_count+len_unchangeable # first changeable
        endIndex = startIndex + num_steps_to_send    # end of changeable
        if (endIndex > num_steps_in_plan):
            endIndex = num_steps_in_plan

        # Get world pose of stance foot (assume first unchangeable?)
        qStance = globalFootSteps[footstep_count]
        iStance = footstep_count # index of current stance foot in global plan

        print "FootStep count=",footstep_count
        localLegName, localFootSteps, localTimeList = \
            getLocalPlan(globalLegName, globalFootSteps, globalTimeList, \
                         iStance, qStance,startIndex, endIndex)

        #print localLegName
        #print localFootSteps
        #print localTimeList
        footstep_count += 1

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    #parser.add_argument("--file", type=str, help="Output file name")
    parser.add_argument("--file", type=str, default="global-plan-test.csv",
                        help="Global planning test file")
    args = parser.parse_args()
    main(args.file)
