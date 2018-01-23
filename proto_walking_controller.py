# -*- encoding: UTF-8 -*-

import time
import argparse
import almath
import csv
from naoqi import ALProxy
robotIP = "192.168.10.110"
experiment_dir = "experiment_data"

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
    print '    Current absolute robot position:', almath.Pose2D(motionProxy.getRobotPosition(useSensorValues))
    print '\n'

def writeGlobalPlan(output_file, leg_array, footstep_array, time_array):
    with open(experiment_dir+"/"+output_file+"-global-plan.csv", 'w+') as csvFile:
        writer = csv.writer(csvFile, delimiter=',')
        writer.writerow(['Leg Name', 'Execution Time', 'X', 'Y', 'Theta'])
        length = len(leg_array)
        for x in range(0, length):
            writer.writerow([ leg_array[x], time_array[x], footstep_array[x][0], footstep_array[x][1], footstep_array[x][2] ])

def writeCSVFootstepsExecuted(writerObj, footstep, iteration_num, currentRobotPose):
    print writerObj
    print footstep
    print iteration_num
    print currentRobotPose
    writerObj.writerow([footstep[0],footstep[2][0], footstep[2][1], footstep[2][2], currentRobotPose[0], currentRobotPose[1], currentRobotPose[2] ])



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
    for it in range(0, length-1):
        x = 0
        if LegFlag == 0:
            _legList.append('RLeg')
            x = footstepList[it+1][0] - footstepList[it][0]
            theta = footstepList[it+1][2] - footstepList[it][2]
            _footstepList.append([x, -y_value, theta])
            LegFlag = 1
        elif LegFlag == 1:
            _legList.append('LLeg')
            x = footstepList[it+1][0] - footstepList[it][0]
            theta = footstepList[it+1][2] - footstepList[it][2]
            _footstepList.append([x, y_value, theta])
            LegFlag = 0

    return [_legList, _footstepList, timeList]

def main(robotIP, outputFile, PORT=9559):
    print experiment_dir
    print outputFile
    file_obj = open(experiment_dir+ '/'+ outputFile+ '-execution.csv', 'w+')
    file_writer = csv.writer(file_obj, delimiter=',')
    file_writer.writerow(['Leg Name', 'Relative x', 'Relative y', 'Relative theta', 
                                    'World x', 'World y', 'World theta'])
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
    legName, footSteps, timeList = createStraightFootStepPlan(10, 0.6, "LLeg")
    #NOTE using local plan as global to test CSV writer
    globalLegName, globalFootSteps, globalTimeList = createStraightGlobalPlan(10, 0.6, 'LLeg')
    writeGlobalPlan(outputFile, legName, footSteps, timeList)
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
                    writeCSVFootstepsExecuted(file_writer, currentUnchangeable, cnt, currentRobotPose)
                    footstep_count = footstep_count + 1
                    verbose = True

                if (len(debug[2]) > 0):
                    if currentChangeable[0] != debug[2][0][0]:
                        # First changeable step in queue has changed
                        # Figure out what the new step is and move the pointer
                        currentChangeable = debug[2][0]
                        verbose = True

                if verbose or update_flag:
                    pass
                    # printHelper(
                    #     verbose, 
                    #     update_flag,
                    #     currentUnchangeable,
                    #     currentChangeable,
                    #     debug,
                    #     useSensorValues,
                    #     footstep_count,
                    #     motionProxy)

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

            if (len(debug[1]) == 0 and len(debug[2]) == 0):
                # Stop the loop, as there are no footsteps left
                flag = False
                file_writer = None
                file_obj.close()
                endRobotPosition = motionProxy.getRobotPosition(useSensorValues)
                endRobotPose = almath.Pose2D(motionProxy.getRobotPosition(useSensorValues))
                robotMove = almath.pose2DInverse(initRobotPose)*endRobotPose
                print '    Robot Move:', robotMove
                print '    End Robot Position:', endRobotPosition
                with open(experiment_dir+"/"+outputFile+"-drift.csv", 'w+') as csvFile:
                    writer = csv.writer(csvFile, delimiter=',')
                    writer.writerow(['Initial x', 'Initial y', 'Initial theta', 
                                     'Final x'  , 'Final y'  , 'Final theta', 
                                     'Delta x'  , 'Delta y'  , 'Delta theta'])
                    writer.writerow([initRobotPosition[0], initRobotPosition[1],
                    initRobotPosition[2], endRobotPosition[0], endRobotPosition[1],
                    endRobotPosition[2], robotMove.x, robotMove.y, robotMove.theta])
        cnt = cnt + 1
    # Go to rest position

    motionProxy.waitUntilMoveIsFinished()
    motionProxy.rest()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", type=str, help="Output file name")
    parser.add_argument("--ip", type=str, default="192.168.10.110",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")
    args = parser.parse_args()
    main(args.ip, args.file, args.port)
