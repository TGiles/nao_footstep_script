import os, errno
import datetime
import itertools
def createFootStepPlan(numOfSteps, timeBetweenStep, startLeg='RLeg'):
    LegFlag = None
    if startLeg == 'RLeg':
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
            legList.append('RLeg')
            footstepList.append([x, y, theta])
            theta += 0.001
            LegFlag = 1
        elif LegFlag == 1:
            legList.append('LLeg')
            footstepList.append([x, -y, theta])
            theta += 0.001
            LegFlag = 0
    startTime = 0.0
    timeList = [0.0]
    for it in range(0, numOfSteps):
        startTime = startTime + timeBetweenStep
        timeList.append(startTime)
    return [legList, footstepList, timeList]

# _legList, _footstepList, _timeList = createFootStepPlan(20, 0.05, 'LLeg')
# print('Leg List')
# for x in _legList:
#     print (x)
#     # print ('\n')
# print('Footstep List')
# for x in _footstepList:
#     print (x)
#     #print ('\n')
# print('Time list')
# for x in _timeList:
#     print (x)
#     # print ('\n')

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
    for thing in footstepList:
        print thing
    for it in range(0, length):
        print 'iterator', it
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


# data = [['LLeg', 'RLeg', 'LLeg', 'RLeg', 'LLeg', 'RLeg', 'LLeg', 'RLeg', 'LLeg', 'RLeg'], [0.6, 1.2, 1.7999999999999998, 2.4, 3.0, 3.6, 4.2, 4.8, 5.3999999999999995, 5.999999999999999], [[0.06, 0.11,0.0], [0.06, -0.11, 0.0], [0.06, 0.11, 0.0], [0.06, -0.11, 0.0], [0.06, 0.11, 0.0], [0.06, -0.11, 0.0], [0.06, 0.11, 0.0], [0.06, -0.11, 0.0], [0.06, 0.11, 0.0], [0.06, -0.11, 0.0]]]
# # print data
# # flat_data = list(itertools.chain.from_iterable(data))
# # print flat_data 
# # length = len(data[0])
# # print length
# legList, footstepList, timeList = createStraightGlobalPlan(10, 0.6, 'LLeg')

# print 'Global legList', legList
# print 'Global footstepList', footstepList
# print 'Global timeList', timeList
# a, b, c = createLocalPlanFromGlobal(legList, footstepList, timeList)
# print
# print 'Transform legList', a
# print 'Transform footstepList', b
# print 'Transform timeList', c
# print
# d, e, f = createStraightFootStepPlan(10, 0.6, 'LLeg')
# print 'Local legList', d
# print 'Local footstepList', e
# print 'Local timeList', f

legList, footstepList, timeList = createStraightGlobalPlan(10, 0.6, 'LLeg')
print 'global footstepList length', len(footstepList)
_leg, _foot, _time = createLocalPlanFromGlobal(legList, footstepList, timeList)
print 'local footstepList length', len(_leg)