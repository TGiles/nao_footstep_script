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

def print_helper(verbose, update_flag, current_unchangeable, current_changeable, footstep_array, use_sensor_values, footstep_count, motion_proxy):
    print '  verbose=',verbose, '   update_flag=',update_flag
    print '  Unchangable step ', current_unchangeable, 'footstep_count', footstep_count
    print '  Changeable step  ', current_changeable
    print '  Current data:'
    print '     Unchangable:'
    for step in footstep_array[1]:
        print'         ', step
    print '     Changeable:'
    for step in footstep_array[2]:
        print'         ', step
    print '    Vector length - change -> unchange', len(footstep_array[1]), len(footstep_array[2])
    print '    Current world frame robot position:', almath.Pose2D(motion_proxy.getRobotPosition(use_sensor_values))
    print '\n'


def write_CSV_footsteps_executed(writer_obj, footstep, iteration_num, elapsed_time,
                              current_robot_pose, update_flag, verbose, plan_sent_flag,
                              start_index, end_index, first_updated_leg,
                              footstep_count, i_stance, stance_leg, step_time_remaining):
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
    'start_index'
    'end_index'
    'first_updated_leg'
    'footstep_count'
    'i_stance'
    'stance_leg'
    'step_time_remaining'
    '''
    # print 'write_CSV_footsteps_executed'
    # print
    # print footstep[2][0]
    # print
    # print footstep[2][0][2]
    # Need to check that changeable vector length is > 0, otherwise need to print N/A or something
    if len(footstep[2]) > 0:
        writer_obj.writerow([
        iteration_num,
        elapsed_time,
        current_robot_pose[0],
        current_robot_pose[1],
        current_robot_pose[2],
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
        first_updated_leg,
        footstep_count,
        i_stance,
        stance_leg,
        step_time_remaining
        ])
    else:
        writer_obj.writerow([
        iteration_num,
        elapsed_time,
        current_robot_pose[0],
        current_robot_pose[1],
        current_robot_pose[2],
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
        first_updated_leg,
        footstep_count,
        i_stance,
        stance_leg,
        step_time_remaining
        ])



def write_summary_CSV(parameter_list, experiment_dir, test_dir):
    # parameter_list should be set up as
    #   len(footsteps)
    #   timeList[0]
    #   cnt
    #   robotInitialConfig
    #   robotEndConfig
    #   robot_move
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


def main(robotIP, PORT=9559, desired_distance=0.5, rho=None):
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
        'First Leg Update',
        'Footstep count',
        'i_stance',
        'stance_leg',
        'step_time_remaining'
    ])
    # file_writer.writerow(['Leg Name', 'Relative x', 'Relative y', 'Relative theta',
    #                                 'World x', 'World y', 'World theta'])
    motion_proxy  = ALProxy("ALMotion", robotIP, PORT)
    posture_proxy = ALProxy("ALRobotPosture", robotIP, PORT)
    # motion_proxy.rest()
    # file_obj.close()
    # return
    # Wake up robot
    motion_proxy.wakeUp()

    # Send robot to Pose Init
    posture_proxy.goToPosture("StandInit", 0.5)
    use_sensor_values = True
    # initRobotPosition = almath.Pose2D(motion_proxy.getRobotPosition(use_sensor_values))
    init_robot_position = motion_proxy.getRobotPosition(use_sensor_values)
    init_robot_pose = almath.Pose2D(init_robot_position)
    print "Robot Position", init_robot_position

    print "Desired distance=",desired_distance
    print "Desired radius of curvature =",rho

    num_steps_to_send = 4
    time_between_step = 0.6 # s
    x_step_length = 0.06 # m
    start_leg = 'LLeg'
    vb = x_step_length / time_between_step # m/s
    wb = 0 # rad/s, straight line test
    y_dist_separation = 0.1 # m, default gait value for maxStepY
    # NOTE FootSeparation 0.1 m, MinFootSeparation 0.088 m according to NAO locomotion API
    if (rho is not None):
        wb = vb/rho # steer to
        desired_distance = (np.pi/2.0)*np.abs(rho)
        print "Recalculate turning rate and distance if given radius of curvature!"
        print "Desired distance=",desired_distance
    print " vb=",vb,"  wb=",wb


    # init_robot_pose = motion_proxy.getRobotPosition(use_sensor_values)
    q_stance = None
    i_stance = 0 # since first footsteps, index of the stance foot should be zero
    footstep_count = 0
    start_index = 1 # Should be one
    end_index = start_index + num_steps_to_send
    dist = 10.0
    init_footstep_vector = None
    q_stance = None # Base stance foot off of the robot pose until we start getting feet data

    print " Step generation : "
    print "       desired dist=",desired_distance, " vb=",vb," wb=",wb
    print "       separation = ",y_dist_separation, " stanceLeg=",start_leg
    print "       init_position=",init_robot_position, "  q_stance=",q_stance
    global_leg_names, global_footsteps, global_time_list = \
        create_global_plan(desired_distance, time_between_step, vb, wb,
                         y_dist_separation, start_leg,
                         init_robot_position, q_stance)

    write_global_plan(global_leg_names, global_footsteps, global_time_list, experiment_dir, test_dir, 'global-plan.csv')
    q_stance = global_footsteps[0] # first global footstep based on body pose
    local_leg_names, local_footsteps, local_time_list = \
            get_local_plan(global_leg_names, global_footsteps, global_time_list,
                         i_stance, q_stance, start_index, end_index)

    num_steps_in_global_plan = len(global_footsteps) # This is limit of stepping loop
    print '   # Steps in Local Plan:', len(local_footsteps)
    print '   # Steps in Global Plan', num_steps_in_global_plan
    clear_existing = True
    # return

    # Send the initial plan to robot starting with the first foot in motion
    motion_proxy.setFootSteps(
        local_leg_names,
        local_footsteps,
        local_time_list,
        clear_existing)
    time.sleep(1.0)

    # Set up processing loop variables
    cnt = 0
    i_stance       = -1 # Increment on first feedback to 0
    footstep_count =  0 # Increment on first change to 1 index of first step sent

    current_unchangeable = ['none']
    current_changeable = ['none']

    # Loop assumes a valid plan was given before starting
    # NOTE Fix this later (1/16)
    run_flag = True
    continue_updating_steps_flag = True # set false when we get to final 2 steps
    time_start = time.time()

    while (run_flag):
        # time.sleep(0.1)
        plan_sent_flag = False
        print 'Iteration', cnt,
        footstep_vectors = motion_proxy.getFootSteps()
        if (footstep_vectors is not None):
          # print '  len(footstep_vectors)=',len(footstep_vectors)
          if(len(footstep_vectors) > 1):
            # @TODO - change footstep_vectors to foostep_feedback (or other relevant name)
            # @TODO - add comments here that show the structure of the data
            #           e.g. just copy a print of the data structure

            # print '  len(footstep_vectors[1])=',len(footstep_vectors[1])

            if (len(footstep_vectors[1]) > 0):

                # Grab the latest robot body pose
                current_robot_pose = motion_proxy.getRobotPosition(use_sensor_values)

                verbose = False # Flag to log any significant change
                if current_unchangeable[0] != footstep_vectors[1][0][0]:
                    # Leg transition has occurred
                    # Update footstep_count for indexing the
                    # global footstep plan
                    current_unchangeable = footstep_vectors[1][0]
                    print '   current_unchangeable', current_unchangeable
                    footstep_count = footstep_count + 1
                    i_stance = i_stance + 1
                    verbose = True

                if (len(footstep_vectors[2]) > 0):
                    if current_changeable[0] != footstep_vectors[2][0][0]:
                        # First changeable step in queue has changed
                        # Figure out what the new step is and move the pointer
                        current_changeable = footstep_vectors[2][0]
                        verbose = True

                # See if it is safe to update the step plan
                # Feet move from changeable to unchangeable during the
                #   0.15 < time remaining < 0.18 interval, so we will only
                #   send plan updates when not in this update zone
                update_flag = False
                step_time_remaining = footstep_vectors[1][0][1]
                if ( (step_time_remaining> 0.18) or (step_time_remaining < 0.15 and step_time_remaining > 0.08) ):
                    #print 'Update flag set'
                    update_flag = True

                if update_flag:
                    # Safe to update the step plan
                    start_index = footstep_count+len(footstep_vectors[1])
                    end_index   = start_index + num_steps_to_send
                    if (end_index > num_steps_in_global_plan):
                        end_index = num_steps_in_global_plan
                        print "  end of step array with ",start_index,"  ",end_index
                if update_flag and continue_updating_steps_flag:
                    if ( (end_index-start_index) < 2):
                        print "  this is the last step plan update ",start_index,"  ",end_index
                        continue_updating_steps_flag = False
                    # There is data in the queue to send
                    if start_index < end_index:
                        q_stance = None
                        if (current_unchangeable[0][0] == global_leg_names[i_stance+1]):
                            if global_leg_names[i_stance] == 'LLeg':
                                q_stance = footstep_vectors[0][0]
                            else:
                                q_stance = footstep_vectors[0][1]
                        else:
                            print " stance foot [",i_stance,"]=",global_leg_names[i_stance]," same as first unchangeable?"
                            print "         ",current_unchangeable[0][0]

                        # Covert global plan into relative local footsteps for update
                        # Apply correction if (and only if) first unchangeable = stance foot
                        local_leg_names, local_footsteps, local_time_list = \
                                get_local_plan(global_leg_names, global_footsteps, global_time_list,
                                             i_stance, q_stance, start_index, end_index)

                        # Send update steps to the robot
                        motion_proxy.setFootSteps(
                            local_leg_names,
                            local_footsteps,
                            local_time_list,
                            True
                        )

                        print "  i_stance=",i_stance," footstep_count=",footstep_count, " length unchangeable=",len(footstep_vectors[1])
                        print '     New plan sent [',start_index,', ',end_index,'] - first step ',
                        print local_leg_names
                        print local_footsteps
                        print local_time_list
                        plan_sent_flag = True

                if verbose or plan_sent_flag:
                    elapsed_time = time.time() - time_start

                    print 'Current velocity:', motion_proxy.getRobotVelocity()
                    write_CSV_footsteps_executed(file_writer, footstep_vectors, cnt, elapsed_time,
                                              current_robot_pose, update_flag,
                                              verbose, plan_sent_flag, start_index,
                                              end_index, local_leg_names[0],
                                              footstep_count,i_stance,
                                              global_leg_names[i_stance], step_time_remaining
                                              )


            if (len(footstep_vectors[1]) == 0 and len(footstep_vectors[2]) == 0):
                # Stop the loop, as there are no footsteps left
                run_flag = False

        cnt = cnt + 1
    # Go to rest position

    #
    print ' wait for move to finish ... '
    motion_proxy.waitUntilMoveIsFinished()

    # Get final data after walking is verified finished
    print ' get final data and write the summary file ...'
    file_writer = None
    file_obj.close()
    end_robot_position = motion_proxy.getRobotPosition(use_sensor_values)
    end_robot_pose = almath.Pose2D(motion_proxy.getRobotPosition(use_sensor_values))
    #init frame calc
    robot_move = almath.pose2DInverse(init_robot_pose)*end_robot_pose
    print '    Robot Move:', robot_move
    print '    End Robot Position:', end_robot_position
    with open(experiment_dir+"/"+test_dir+"/drift.csv", 'w+') as csvFile:
        writer = csv.writer(csvFile, delimiter=',')
        writer.writerow([test_dir])
        writer.writerow(['Initial x', 'Initial y', 'Initial theta',
                         'Final x'  , 'Final y'  , 'Final theta',
                         'Delta x'  , 'Delta y'  , 'Delta theta'])
        writer.writerow([init_robot_position[0], init_robot_position[1],
        init_robot_position[2], end_robot_position[0], end_robot_position[1],
        end_robot_position[2], robot_move.x, robot_move.y, robot_move.theta])
    write_summary_CSV([
        len(global_footsteps),
        global_time_list[0],
        cnt,
        init_robot_position,
        end_robot_position,
        robot_move
    ], experiment_dir, test_dir)

    # Robot to crouch position
    print '  Move robot to rest position ... '
    motion_proxy.rest()
    plotter(experiment_dir, test_dir)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    #parser.add_argument("--file", type=str, help="Output file name")
    parser.add_argument("--ip", type=str, default="192.168.10.110",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")
    parser.add_argument("--desired_distance", type=float, default=0.5,
                        help="Desired walking distance (meters)")
    parser.add_argument("--desired_radius", type=float, default=None,
                        help="Desired radius of curvature (meters) (default =None)")
    args = parser.parse_args()
    main(args.ip, args.port, args.desired_distance, args.desired_radius)
