# -*- encoding: UTF-8 -*-

import almath
import argparse
from naoqi import ALProxy

def main(robotIP, PORT=9559):
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)

    # Wake up robot
    motionProxy.wakeUp()

    # Send robot to Pose Init
    postureProxy.goToPosture("StandInit", 0.5)

    # Example showing how to get a simplified robot position in world.
    useSensorValues = False
    result = motionProxy.getRobotPosition(useSensorValues)
    print "Simple Robot Position", result

    # Example showing how to use this information to know the robot's diplacement.
    useSensorValues = True
    initRobotPosition = almath.Pose2D(motionProxy.getRobotPosition(useSensorValues))
    print "Robot Position", initRobotPosition
    # Make the robot move
    # motionProxy.moveTo(0.1, 0.0, 0.2)

    # 4 foot (4 Ft, 48 in)
    # x = 1.2192
    # y = 0
    # theta = 0
    # print x, y, theta
    # motionProxy.moveTo(x, y, theta)

    # 2 foot (2 ft, 24in)
    x = 0.6096
    y = 0
    theta = 0
    print "Move to commands:", x, y, theta
    motionProxy.moveTo(x, y, theta)

    endRobotPosition = almath.Pose2D(motionProxy.getRobotPosition(useSensorValues))

    # Compute robot's' displacement
    robotMove = almath.pose2DInverse(initRobotPosition)*endRobotPosition
    print "Robot Move:", robotMove
    print "End Robot Position:", motionProxy.getRobotPosition(useSensorValues)

    # Go to rest position
    motionProxy.rest()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")

    args = parser.parse_args()
    main(args.ip, args.port)
