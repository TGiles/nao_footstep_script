# -*- encoding: UTF-8 -*-

import time
import datetime
import os
import argparse
import almath
import csv
from footstep_helper import *
from plot_footsteps import *

from naoqi import ALProxy
robotIP = '192.168.10.110'

def printHelper(verbose, update_flag, currentUnchangeable, currentChangeable,footstepArray, useSensorValues, footstep_count, motionProxy):
    print '  verbose=',verbose, '   update_flag=',update_flag
    print '  Unchangable step ', currentUnchangeable, 'footstep_count', footstep_count
    print '  Changeable step  ', currentChangeable
    print '  Current data:'
    print '     Unchangable:'
    for step in footstepArray[1]:
        print'         ', step
    print '     Changeable:'
    for step in footstepArray[2]:
        print'         ', step
    print '    Vector length - change -> unchange', len(footstepArray[1]), len(footstepArray[2])
    print '    Current world frame robot position:', almath.Pose2D(motionProxy.getRobotPosition(useSensorValues))
    print '\n'


def writeCSVFootstepsExecuted(writerObj, footstep, iteration_num, elapsed_time,
                              currentRobotPose, update_flag, verbose, plan_sent_flag,
                              start_index, end_index, footstep_count):
    ''' Should write:
    iteration num
    elapsed time
    body pose from robot pose
    step data (include foot locations)
    num steps in unchangeable
    unchangeable vector
    1st step unchangeable
    1st step changeable
    'Iteration',
    'World x',
    'World y',
    'World theta'
    'World leftfoot x',
    'World leftfoot y',
    'World leftfoot theta',
    'World rightfoot x',
    'World rightfoot y',
    'World rightfoot theta',
    '# steps in unchangeable',
    'First unchangeable leg',
    'First unchangeable x',
    'First unchangeable y',
    'First unchangeable theta',
    'First changeable leg',
    'First changeable x',
    'First changeable y',
    'First changeable theta'
    'Update flag',
    'Verbose',
    'Plan sent?'
    '''
    # print 'writeCSVFootstepsExecuted'
    # print
    # print footstep[2][0]
    # print
    # print footstep[2][0][2]
    # Need to check that changeable vector length is > 0, otherwise need to print N/A or something
    if len(footstep[2]) > 0:
        writerObj.writerow([
        iteration_num,
        elapsed_time,
        currentRobotPose[0],
        currentRobotPose[1],
        currentRobotPose[2],
        footstep[0][0][0],
        footstep[0][0][1],
        footstep[0][0][2],
        footstep[0][1][0],
        footstep[0][1][1],
        footstep[0][1][2],
        len(footstep[2]),
        footstep[1][0][0],
        footstep[1][0][2][0],
        footstep[1][0][2][1],
        footstep[1][0][2][2],
        footstep[2][0][0],
        footstep[2][0][2][0],
        footstep[2][0][2][1],
        footstep[2][0][2][2],
        update_flag,
        verbose,
        plan_sent_flag,
        start_index,
        end_index,
        footstep_count
        ])
    else:
        writerObj.writerow([
        iteration_num,
        elapsed_time,
        currentRobotPose[0],
        currentRobotPose[1],
        currentRobotPose[2],
        footstep[0][0][0],
        footstep[0][0][1],
        footstep[0][0][2],
        footstep[0][1][0],
        footstep[0][1][1],
        footstep[0][1][2],
        len(footstep[2]),
        footstep[1][0][0],
        footstep[1][0][2][0],
        footstep[1][0][2][1],
        footstep[1][0][2][2],
        'No changeable leg',
        'No changeable x',
        'No changeable y',
        'No changeable theta',
        update_flag,
        verbose,
        plan_sent_flag,
        start_index,
        end_index,
        footstep_count
        ])



def writeSummaryCSV(parameter_list, experiment_dir, test_dir):
    # parameter_list should be set up as
    #   len(footsteps)
    #   timeList[0]
    #   cnt
    #   robotInitialConfig
    #   robotEndConfig
    #   robotMove
    with open(experiment_dir+'/'+ test_dir +'/summary.csv', 'w+') as csvFile:
        writer = csv.writer(csvFile, delimiter=',')
        writer.writerow([test_dir])
        writer.writerow(['Number of steps in plan', 'Time between steps', 'Iterations through main loop',
        'Initial x', 'Initial y', 'Initial theta', 'Final x', 'Final y', 'Final theta',
        'Delta x', 'Delta y', 'Delta theta'])
        writer.writerow([ parameter_list[0], parameter_list[1], parameter_list[2],
        parameter_list[3][0], parameter_list[3][1], parameter_list[3][2],
        parameter_list[4][0], parameter_list[4][1], parameter_list[4][2],
        parameter_list[5].x, parameter_list[5].y, parameter_list[5].theta ])


def main(robotIP, PORT=9559):
    experiment_dir = 'experiment_data'
    test_dir = str(datetime.datetime.now())
    try:
        os.makedirs(experiment_dir + '/' + test_dir)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise
    os.chmod(experiment_dir + '/' + test_dir, 0776)
    # outputFile = str(datetime.datetime.now())
    file_obj = open(experiment_dir+ '/'+ test_dir +'/footstep-execution.csv', 'w+')
    file_writer = csv.writer(file_obj, delimiter=',')
    file_writer.writerow([test_dir])
    ''' Should write:
    iteration num
    body pose from robot pose
    step data (include foot locations)
    num steps in unchangeable
    unchangeable vector
    1st step unchangeable
    1st step changeable
    '''
    file_writer.writerow([
        'Iteration',
        'Elapsed Time',
        'World x',
        'World y',
        'World theta',
        'World LFoot x',
        'World LFoot y',
        'World LFoot theta',
        'World RFoot x',
        'World RFoot y',
        'World RFoot theta',
        '# steps in unchangeable',
        'First unchangeable leg',
        'First unchangeable x',
        'First unchangeble y',
        'First unchangeable theta',
        'First changeable leg',
        'First changeable x',
        'First changeable y',
        'First changeable theta',
        'Update flag',
        'Verbose',
        'New plan sent?',
        'Start index',
        'End index',
        'Footstep count'
    ])
    # file_writer.writerow(['Leg Name', 'Relative x', 'Relative y', 'Relative theta',
    #                                 'World x', 'World y', 'World theta'])
    motionProxy  = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
    # motionProxy.rest()
    # file_obj.close()
    # return
    # Wake up robot
    motionProxy.wakeUp()

    # Send robot to Pose Init
    postureProxy.goToPosture("StandInit", 0.5)
    useSensorValues = True
    # initRobotPosition = almath.Pose2D(motionProxy.getRobotPosition(useSensorValues))
    init_robot_position = motionProxy.getRobotPosition(useSensorValues)
    initRobotPose = almath.Pose2D(init_robot_position)
    print "Robot Position", init_robot_position

    num_steps_to_send = 4
    time_between_step = 0.6 # s
    x_step_length = 0.06 # m
    start_leg = 'LLeg'
    desired_distance = 1.2191 # m, currently 4ft
    vb = x_step_length / time_between_step # m/s
    wb = 0 # rad/s, straight line test
    y_dist_separation = 0.1 # m, default gait value for maxStepY
    # NOTE FootSeparation 0.1 m, MinFootSeparation 0.088 m according to NAO locomotion API

    # init_robot_pose = motionProxy.getRobotPosition(useSensorValues)
    q_stance = None
    i_stance = 0 # since first footsteps, index of the stance foot should be zero
    footstep_count = 0
    start_index = 1 # Should be one
    end_index = start_index + num_steps_to_send
    dist = 10.0
    init_footstep_vector = None
    # should feet_separation be y_dist_separation?
    # while (dist > 0.6*y_dist_separation):
    #     # Since it's a do while type of loop, need initial steps to kick off planning
    #     init_footstep_vector = motionProxy.getFootSteps()
    #     if start_leg == 'LLeg':
    #         q_stance = init_footstep_vector[0][0]
    #     else:
    #         q_stance = init_footstep_vector[0][1]

    #     print "  init robot position =",init_robot_position
    #     print "    qStance=",q_stance

    #     dist = np.sqrt((q_stance[0]-init_robot_position[0])**2 + (q_stance[1]-init_robot_position[1])**2)
    #     print "    dist = ", dist

    #     if (dist > 0.6*y_dist_separation):
    #         print "Error: Invalid starting pose data:"
    #         time.sleep(1.0)
    q_stance = None

    print " Step generation : "
    print "       desired dist=",desired_distance, " vb=",vb," wb=",wb
    print "       separation = ",y_dist_separation, " stanceLeg=",start_leg
    print "       init_position=",init_robot_position, "  q_stance=",q_stance
    globalLegNames, globalFootSteps, globalTimeList = \
        createGlobalPlan(desired_distance, time_between_step, vb, wb,
                         y_dist_separation, start_leg,
                         init_robot_position, q_stance)

    writeGlobalPlan(globalLegNames, globalFootSteps, globalTimeList, experiment_dir, test_dir, 'global-plan.csv')
    q_stance = globalFootSteps[0] # first global footstep
    localLegNames, localFootSteps, localTimeList = \
            getLocalPlan(globalLegNames, globalFootSteps, globalTimeList,
                         i_stance, q_stance, start_index, end_index)

    num_steps_in_global_plan = len(globalFootSteps) # This is limit of stepping loop
    print '   # Steps in Local Plan:', len(localFootSteps)
    print '   # Steps in Global Plan', num_steps_in_global_plan
    clearExisting = True
    # return
    motionProxy.setFootSteps(
        localLegNames,
        localFootSteps,
        localTimeList,
        clearExisting)
    time.sleep(1.0)

    # Set up processing loop variables
    cnt = 0
    footstep_count = -1
    currentUnchangeable = ['none']
    currentChangeable = ['none']

    # Loop assumes a valid plan was given before starting
    # NOTE Fix this later (1/16)
    flag = True
    time_start = time.time()

    while (flag):
        # time.sleep(0.1)
        plan_sent_flag = False
        print 'Iteration', cnt,
        debug = motionProxy.getFootSteps()
        if (debug is not None):
          # print '  len(debug)=',len(debug)
          if(len(debug) > 1):
            # print '  len(debug[1])=',len(debug[1])

            if (len(debug[1]) > 0):
                update_flag = False
                if debug[1][0][1] > 0.18 or (debug[1][0][1] < 0.15 and debug[1][0][1] > 0.08):
                    print 'Update flag set'
                    update_flag = True

                verbose = False
                if currentUnchangeable[0] != debug[1][0][0]:
                    # Leg transition has occurred
                    # Update footstep_count for indexing the
                    # global footstep plan
                    currentUnchangeable = debug[1][0]
                    print '   currentUnchangeable', currentUnchangeable
                    currentRobotPose = motionProxy.getRobotPosition(useSensorValues)
                    footstep_count = footstep_count + 1
                    verbose = True

                if (len(debug[2]) > 0):
                    if currentChangeable[0] != debug[2][0][0]:
                        # First changeable step in queue has changed
                        # Figure out what the new step is and move the pointer
                        currentChangeable = debug[2][0]
                        verbose = True

                if update_flag:
                    startIndex = footstep_count+len(debug[1])
                    endIndex = startIndex + num_steps_to_send
                    if (endIndex > num_steps_in_global_plan):
                        endIndex = num_steps_in_global_plan

                    if startIndex < endIndex:
                        i_stance = footstep_count
                        if currentUnchangeable[0][0] == 'LLeg':
                            q_stance = debug[0][0]
                        else:
                            q_stance = debug[0][1]
                        localLegNames, localFootSteps, localTimeList = \
                                getLocalPlan(globalLegNames, globalFootSteps, globalTimeList,
                                             i_stance, q_stance, startIndex, endIndex)
                        motionProxy.setFootSteps(
                            localLegNames,
                            localFootSteps,
                            localTimeList,
                            True
                        )
                        print '     New plan sent [',startIndex,', ',endIndex,'] - first step ',
                        print localLegNames
                        print localFootSteps
                        print localTimeList
                        plan_sent_flag = True

                if verbose or plan_sent_flag:
                    elapsed_time = time.time() - time_start

                    print 'Current velocity:', motionProxy.getRobotVelocity()
                    writeCSVFootstepsExecuted(file_writer, debug, cnt, elapsed_time,
                                              currentRobotPose, update_flag,
                                              verbose, plan_sent_flag, start_index,
                                              end_index, footstep_count)


            if (len(debug[1]) == 0 and len(debug[2]) == 0):
                # Stop the loop, as there are no footsteps left
                flag = False

        cnt = cnt + 1
    # Go to rest position

    #
    print ' wait for move to finish ... '
    motionProxy.waitUntilMoveIsFinished()

    # Get final data after walking is verified finished
    print ' get final data and write the summary file ...'
    file_writer = None
    file_obj.close()
    endRobotPosition = motionProxy.getRobotPosition(useSensorValues)
    endRobotPose = almath.Pose2D(motionProxy.getRobotPosition(useSensorValues))
    #init frame calc
    robotMove = almath.pose2DInverse(initRobotPose)*endRobotPose
    print '    Robot Move:', robotMove
    print '    End Robot Position:', endRobotPosition
    with open(experiment_dir+"/"+test_dir+"/drift.csv", 'w+') as csvFile:
        writer = csv.writer(csvFile, delimiter=',')
        writer.writerow([test_dir])
        writer.writerow(['Initial x', 'Initial y', 'Initial theta',
                         'Final x'  , 'Final y'  , 'Final theta',
                         'Delta x'  , 'Delta y'  , 'Delta theta'])
        writer.writerow([init_robot_position[0], init_robot_position[1],
        init_robot_position[2], endRobotPosition[0], endRobotPosition[1],
        endRobotPosition[2], robotMove.x, robotMove.y, robotMove.theta])
    writeSummaryCSV([
        len(footSteps),
        timeList[0],
        cnt,
        init_robot_position,
        endRobotPosition,
        robotMove
    ], experiment_dir, test_dir)

    # Robot to crouch position
    print '  Move robot to rest position ... '
    motionProxy.rest()
    plotter(experiment_dir, test_dir)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    #parser.add_argument("--file", type=str, help="Output file name")
    parser.add_argument("--ip", type=str, default="192.168.10.110",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")
    args = parser.parse_args()
    main(args.ip, args.port)
