#!/usr/bin/python

import csv
import sys
import numpy as np
import matplotlib.pyplot as plt
from  footstep_helper import *


global_steps_file = "experiment_data/global-plan.csv"

steps={}
steps["LLeg"]=[]
steps["RLeg"]=[]


print "Load the global footstep plan ... "
hdr = 0
with open(global_steps_file,'rt') as csvfile:
    print "opened"
    global_steps_reader = csv.reader(csvfile,delimiter=',')
    print "got main reader ..."
    for row in global_steps_reader:
        if (hdr < 2):
            hdr += 1
            print "hdr",hdr," >",row
            continue

        print ">",row
        #      leg                  time           x              y           theta
        steps[row[0]].append([float(row[1]),float(row[2]),float(row[3]),float(row[4])])


print "drawing the global footstep plan ..."
fig=plt.figure(1)
fig.set_tight_layout(True)
fig.gca().axis('equal')
plot_footsteps(fig,steps)

plt.draw() # draw the initial windows


print "Read in the run data ... "

footstep_execution_file = "experiment_data/footstep-execution.csv"
hdr = 0
itr = -1
Tinit = None
steps={}
steps["LLeg"]=[]
steps["RLeg"]=[]
body = []

itr_ndx = 0
time_ndx = 1
body_ndx = 1
lleg_ndx = 4
rleg_ndx = 7

with open(footstep_execution_file,'rt') as csvfile:
    print "opened"
    footstep_execution_reader = csv.reader(csvfile,delimiter=',')
    print "got global steps reader ..."
    for row in footstep_execution_reader:
        if (hdr < 2):
            hdr += 1
            print "hdr",hdr," >",row
            continue

        #print ">",row
        if (Tinit is None):
            print "Initializing the global transform ..."
            T = transform2D(float(row[body_ndx]),float(row[body_ndx+1]),float(row[body_ndx+2]))
            print "  T=",T
            Tinit = np.linalg.inv(T)
            print "  Ti=",Tinit

        cnt = int(row[0])
        if (itr == cnt):
            print "skipping same iteration data ",itr
        else:
            itr = cnt
            time = 0.6*itr # float(row[time_ndx])

            # Get the body pose
            xb  = float(row[body_ndx])
            yb  = float(row[body_ndx+1])
            Zb  = float(row[body_ndx+2])
            T = np.dot(Tinit, transform2D(xb,yb,Zb))
            body.append([time, T[0][2], T[1][2],np.arctan2(T[1][0],T[0][0])])

            # Get the step data
            xb  = float(row[lleg_ndx])
            yb  = float(row[lleg_ndx+1])
            Zb  = float(row[lleg_ndx+2])
            T = np.dot(Tinit, transform2D(xb,yb,Zb))
            steps["LLeg"].append([time, T[0][2], T[1][2],np.arctan2(T[1][0],T[0][0])])

            xb  = float(row[rleg_ndx])
            yb  = float(row[rleg_ndx+1])
            Zb  = float(row[rleg_ndx+2])
            T = np.dot(Tinit, transform2D(xb,yb,Zb))
            steps["RLeg"].append([time, T[0][2], T[1][2],np.arctan2(T[1][0],T[0][0])])


plot_footsteps(fig,steps,{"LLeg":'c:',"RLeg":'m:'})
plot_body(fig, body,'g.')

plt.draw() # draw the initial windows

plt.show() # keep windows open