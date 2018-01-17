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

_legList, _footstepList, _timeList = createFootStepPlan(20, 0.05, 'LLeg')
print('Leg List')
for x in _legList:
    print (x)
    # print ('\n')
print('Footstep List')
for x in _footstepList:
    print (x)
    #print ('\n')
print('Time list')
for x in _timeList:
    print (x)
    # print ('\n')
