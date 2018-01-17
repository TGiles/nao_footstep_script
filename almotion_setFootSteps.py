# -*- encoding: UTF-8 -*-

import time
import argparse
from naoqi import ALProxy
robotIP = "192.168.10.110"

# Creates a straight line walking path
def createFootStepPlan(numOfSteps, timeBetweenStep, startLeg="RLeg"):
    LegFlag = None
    if startLeg == "RLeg":
        LegFlag = 0
    else:
        LegFlag = 1
    
    x = 0.04
    y = 0.1
    # In order to keep a track of which internal footstep is being executed, going to naively use theta
    # until I determine a better route
    theta = 0.001
    legList = []
    footstepList = []
    for it in range(0, numOfSteps):
        if LegFlag == 0:
            legList.append("RLeg")
            footstepList.append([x, -y, theta])
            theta += 0.001
            LegFlag = 1
        elif LegFlag == 1:
            legList.append("LLeg")
            footstepList.append([x, y, theta])
            theta += 0.001
            LegFlag = 0
    startTime = 0.6
    timeList = [0.6]
    for it in range(0, numOfSteps):
        startTime = startTime + timeBetweenStep
        timeList.append(startTime)
    return [legList, footstepList, timeList]
    


def main(robotIP, PORT=9559):

    motionProxy  = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)

    # Wake up robot
    motionProxy.wakeUp()

    # Send robot to Pose Init
    postureProxy.goToPosture("StandInit", 0.5)

    # A small step forwards and anti-clockwise with the left foot
    # legName  = ["LLeg"]
    # X        = 0.04
    # Y        = 0.1
    # Theta    = 0.3
    # footSteps = [[X, Y, Theta]]
    # timeList = [0.6]
    # clearExisting = False
    # motionProxy.setFootSteps(legName, footSteps, timeList, clearExisting)

    # motionProxy.waitUntilMoveIsFinished()
    #time.sleep(1.0)

    # A small step forwards and anti-clockwise with the left foot
    # legName = ["LLeg", "RLeg"]
    # X       = 0.04
    # Y       = 0.1
    # Theta   = 0.0
    # footSteps = [[X, Y, Theta], [X, -Y, Theta]]
    # timeList = [0.6, 2.4]
    # clearExisting = False
    # motionProxy.setFootSteps(legName, footSteps, timeList, clearExisting)
    # debug = motionProxy.getFootSteps()
    # print('Step 0')
    # # print(debug[0])
    # print('Unchangeable', debug[1])
    # print('Changeable', debug[2])
    # print('\n')
    # time.sleep(0.6)
    # debug = motionProxy.getFootSteps()
    # print('Step 1')
    # # print(debug[0])
    # print('Unchangeable', debug[1])
    # print('Changeable', debug[2])
    # print('\n')
    # time.sleep(0.6)
    # # debug = motionProxy.getFootSteps()
    # # print(debug[0])
    # # print(debug[1])
    # # print(debug[2])
    # # print('\n')
    # motionProxy.waitUntilMoveIsFinished()
    # time.sleep(4.0)
	# 4 footsteps with getFootSteps() printing out 'hopefully' relevant information

    # legName = ["LLeg", "RLeg", "LLeg", "RLeg", "LLeg", "RLeg", "LLeg", "RLeg", "LLeg", "RLeg", "LLeg", "RLeg", "LLeg", "RLeg", "LLeg", "RLeg"]
    # X = 0.04
    # Y = 0.1
    # Theta = 0.0
    # footSteps = [
    #     [X, -Y, 0.001], [X, Y, 0.002], [X, Y, 0.003], [X, -Y, 0.004], [X, Y, 0.005], [X, -Y, 0.006], [X, Y, 0.007], [X, -Y, 0.008],
    #     [X, Y, 0.009], [X, -Y, 0.010], [X, Y, 0.011], [X, -Y, 0.012], [X, Y, 0.013], [X, -Y, 0.014], [X, Y, 0.015], [X, -Y, 0.016]
    #     ]
    # # timeList = [0.6, 1.2, 1.8, 2.4]
    # timeList = [0.4, 0.8, 1.2, 1.6, 2.2, 2.8, 3.4, 4.0, 4.6, 5.2, 5.8, 6.4, 7.0, 7.6, 8.2, 8.6]
    # # timeList = [0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4]
    # # timeList = [0.4]
    # newLegName = ["LLeg", "RLeg", "LLeg"]
    # # newTimeList = [2.8, 3.4, 4.0]
    # newTimeList = [0.6, 1.2, 1.8]
    # newFootSteps = [
    #     [X, Y, -0.001], [X, -Y, -0.002], [X, Y, -0.003]
    # ]
    # clearExisting = False
    # print legName
    # print '\n'
    # print footSteps
    # print '\n'
    # print timeList
    # motionProxy.setFootSteps(legName, footSteps, timeList, clearExisting)
    # time.sleep(1.0) # needed for some reason
    # motionProxy.waitUntilMoveIsFinished()
    # motionProxy.rest()
    # return
    # # time.sleep(0.2)
    # flag = True
    #for x in range(0, 35):
    num_steps_to_send = 4
    legName, footSteps, timeList = createFootStepPlan(20, 0.6, "LLeg")
    # print legName[0:num_steps_to_send]
    # print '\n'
    # print footSteps[0:num_steps_to_send]
    # print '\n'
    # print timeList[0:num_steps_to_send]
    # print '\n'
    # aLegName = legName[0:num_steps_to_send]
    # aFootSteps = footSteps[0:num_steps_to_send]
    # aTimeList = timeList[0:num_steps_to_send]
    clearExisting = True
    # motionProxy.setFootSteps(aLegName, aFootSteps, aTimeList, clearExisting)
    motionProxy.setFootSteps(legName[0:num_steps_to_send], footSteps[0:num_steps_to_send], timeList[0:num_steps_to_send], clearExisting)
    time.sleep(1.0)
    cnt = 0
    footstep_count = 0
    
    current = None
    # TESTING
    # flag = False
    # Loop assumes a valid plan was given before starting
    # NOTE Fix this later (1/16)
    flag = True
    while (flag):
        print('Iteration', cnt)
        # time.sleep(0.1)
        debug = motionProxy.getFootSteps()
        # print('Unchangeable', debug[1])
        # print('Changeable', debug[2])
        # print('Unchangeable:')
        # for thing in debug[1]:
        #     print(thing)
        # print('Changeable:')
        # for thing in debug[2]:
        #     print(thing)
        # time.sleep(0.1)
        # print(debug)
        # print(len(debug))
        # if footstep_count == 3:
        #     motionProxy.setFootSteps(newLegName, newFootSteps, newTimeList, True)
        #     print('Add new footstep plan')
        #     footstep_count = footstep_count + 1000
        if (debug is not None and len(debug) > 1):
            if (len(debug[1]) > 0):
                update_flag = False
                if debug[1][0][1] > 0.08:
                    # TODO implement footstep update
                    # print('we can send update')
                    print 'Update flag set'
                    print debug[1][0]
                    print debug[1][0][1]
                    update_flag = True
                if current != debug[1][0][0]:
                    # Leg transition has occurred
                    # Update footstep_count for indexing the
                    # global footstep plan
                    current = debug[1][0][0]
                    footstep_count = footstep_count + 1
                    print('Leg changed, number', footstep_count)
                elif current == debug[1][0][0] and update_flag:
                    if footstep_count == len(footSteps):
                        continue
                    # Need to talk to DC about what this elif should be
                    # NOTE: only sends plan when the previous plan is exhausted which isn't intended behavior
                    elif footstep_count % num_steps_to_send == 0:
                        motionProxy.setFootSteps(
                            legName[footstep_count:footstep_count+num_steps_to_send],
                            footSteps[footstep_count:footstep_count+num_steps_to_send],
                            timeList[footstep_count:footstep_count+num_steps_to_send],
                            True
                        )
                        print('New plan sent')
            if (len(debug[1]) == 0 and len(debug[2]) == 0):
                # Stop the loop, as there are no footsteps left
                flag = False
            #else:
            # print (debug[1])
            # print (debug[2])
            # if (len(debug[1]) > 0 and current[0] != )
            # print (debug[1][0][0], 'should be leg name')
            # if (len(debug[1]) > 0 and current != debug[1][0][0]):
            #     print('-------------------------')
            #     print(current, 'Previous Leg')
            #     current = debug[1][0][0]
            #     print(current, 'Next Leg')
            #     print('-------------------------\n')
            #     footstep_count = footstep_count + 1
            # if (len(debug[1]) > 0 and current != debug[1][0]):
            #     print('-------------------------')
            #     print(current, 'PREVIOUS CURR')
            #     current = debug[1][0]
            #     print(current, 'CURRENT CURR')
            #     print('-------------------------\n')
        cnt = cnt + 1
    # Go to rest position

    motionProxy.waitUntilMoveIsFinished()
    motionProxy.rest()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.10.110",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")

    args = parser.parse_args()
    main(args.ip, args.port)
