# -*- encoding: UTF-8 -*-

import time
import datetime
import os
import argparse
import almath
import csv
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

def writeGlobalPlan(leg_array, footstep_array, time_array, experiment_dir, test_dir):
    with open(experiment_dir+'/'+ test_dir + '/global-plan.csv', 'w+') as csvFile:
        writer = csv.writer(csvFile, delimiter=',')
        writer.writerow([test_dir])
        writer.writerow(['Leg Name', 'Execution Time', 'X', 'Y', 'Theta'])
        length = len(leg_array)
        for x in range(0, length):
            writer.writerow([ leg_array[x], time_array[x], footstep_array[x][0], footstep_array[x][1], footstep_array[x][2] ])


def writeCSVFootstepsExecuted(writerObj, footstep, iteration_num, currentRobotPose, update_flag, verbose, plan_sent_flag):
    ''' Should write:
    iteration num
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
        plan_sent_flag
        ])
    else:
        writerObj.writerow([
        iteration_num,
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
        plan_sent_flag
        ])



# Creates a straight line walking path
# NOTE Creates a local plan NOT global plan
def createStraightFootStepPlan(numOfSteps, timeBetweenStep, startLeg="RLeg"):
    LegFlag = None
    if startLeg == "RLeg":
        LegFlag = 0
    else:
        LegFlag = 1

    # x = 0.04
    x= 0.06
    y = 0.11
    # In order to keep a track of which internal footstep is being executed, going to naively use theta
    # until I determine a better route
    theta = 0.0
    legList = []
    footstepList = []
    for it in range(0, numOfSteps):
        if LegFlag == 0:
            legList.append("RLeg")
            footstepList.append([x, -y, theta])
            # theta += 0.001
            LegFlag = 1
        elif LegFlag == 1:
            legList.append("LLeg")
            footstepList.append([x, y, theta])
            # theta += 0.001
            LegFlag = 0
    startTime = timeBetweenStep
    timeList = []
    for it in range(0, numOfSteps):
        timeList.append(startTime)
        startTime = startTime + timeBetweenStep
    return [legList, footstepList, timeList]


def createStraightGlobalPlan(numOfSteps, timeBetweenStep, startLeg='RLeg'):
    LegFlag = None
    if startLeg == 'RLeg':
        LegFlag = 0
    else:
        LegFlag = 1
    x = 0.06
    # y is essentially the distance between the feet, so should remain a constant (I guess)
    y = 0.1
    theta = 0.0
    x_inc = x
    y_inc = y
    theta_inc = theta
    legList = []
    footstepList = []
    for it in range(0, numOfSteps):
        if LegFlag == 0:
            legList.append('RLeg')
            footstepList.append([x, -y, theta])
            x += x_inc
            # Don't need theta for straight lines
            # theta += theta_inc
            LegFlag = 1
        elif LegFlag == 1:
            legList.append('LLeg')
            footstepList.append([x, y, theta])
            x += x_inc
            # Don't need theta for straight lines
            # theta += theta_inc
            LegFlag = 0
    startTime = timeBetweenStep
    timeList = []
    for it in range(0, numOfSteps):
        timeList.append(startTime)
        startTime += timeBetweenStep
    return [legList, footstepList, timeList]


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
        'New plan sent?'
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
    initRobotPosition = motionProxy.getRobotPosition(useSensorValues)
    initRobotPose = almath.Pose2D(initRobotPosition)
    print "Robot Position", initRobotPosition

    num_steps_to_send = 4
    num_steps_straight_plan = 10
    time_between_step = 0.6
    start_leg = 'LLeg'

    # legName, footSteps, timeList = createStraightFootStepPlan(10, 0.6, "LLeg")
    #NOTE using local plan as global to test CSV writer
    globalLegName, globalFootSteps, globalTimeList = createStraightGlobalPlan(
        num_steps_straight_plan,
        time_between_step,
        start_leg)



    writeGlobalPlan(globalLegName, globalFootSteps, globalTimeList, experiment_dir, test_dir)
    legName, footSteps, timeList = createLocalPlanFromGlobal(globalLegName, globalFootSteps, globalTimeList)
    num_steps_in_plan = len(footSteps)
    clearExisting = True
    motionProxy.setFootSteps(legName[0:num_steps_to_send], footSteps[0:num_steps_to_send], timeList[0:num_steps_to_send], clearExisting)
    time.sleep(1.0)
    cnt = 0
    footstep_count = -1
    currentUnchangeable = ['none']
    currentChangeable = ['none']

    # Loop assumes a valid plan was given before starting
    # NOTE Fix this later (1/16)
    flag = True

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
                    if (endIndex > num_steps_in_plan):
                        endIndex = num_steps_in_plan

                    if startIndex < endIndex:
                        motionProxy.setFootSteps(
                            legName[  startIndex: endIndex],
                            footSteps[startIndex: endIndex],
                            timeList[ startIndex: endIndex],
                            True
                        )
                        print '     New plan sent [',startIndex,', ',endIndex,'] - first step ', footSteps[startIndex]
                        plan_sent_flag = True
                        writeCSVFootstepsExecuted(file_writer, debug, cnt, currentRobotPose, update_flag, verbose, plan_sent_flag)

                if verbose or plan_sent_flag:
                    print 'Current velocity:', motionProxy.getRobotVelocity()
                    writeCSVFootstepsExecuted(file_writer, debug, cnt, currentRobotPose, update_flag, verbose, plan_sent_flag)
                    # printHelper(
                    #     verbose,
                    #     update_flag,
                    #     currentUnchangeable,
                    #     currentChangeable,
                    #     debug,
                    #     useSensorValues,
                    #     footstep_count,
                    #     motionProxy)

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
        writer.writerow([initRobotPosition[0], initRobotPosition[1],
        initRobotPosition[2], endRobotPosition[0], endRobotPosition[1],
        endRobotPosition[2], robotMove.x, robotMove.y, robotMove.theta])
    writeSummaryCSV([
        len(footSteps),
        timeList[0],
        cnt,
        initRobotPosition,
        endRobotPosition,
        robotMove
    ], experiment_dir, test_dir)

    # Robot to crouch position
    print '  Move robot to rest position ... '
    motionProxy.rest()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    #parser.add_argument("--file", type=str, help="Output file name")
    parser.add_argument("--ip", type=str, default="192.168.10.110",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")
    args = parser.parse_args()
    main(args.ip, args.port)
