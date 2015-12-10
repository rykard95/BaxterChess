#! /usr/bin/env python
from itertools import izip
import sys, argparse
import numpy as np
import random as rand

import rospy
import cv2 as v
import tf, moveit_commander
from baxter_interface import gripper as baxter_gripper
import baxter_interface

from moveit_msgs.msg import (
    OrientationConstraint, Constraints, PositionConstraint )
from geometry_msgs.msg import PoseStamped
from project.msg import *



# the name of the world frame
BASE_FRAME = 'base'

# perturb tolerances.
PERTURB_TOLS = [0.05, 0.05, 0.05]

# go 7cm above desired board positions, then down
OFFSET = np.array([0,0, 7/100])

CLOSE_AMOUNT = 100.0
hld_frc = 1
mv_frc = 25

def assign_xyz(arr, xyz):
    xyz.x = arr[0]
    xyz.y = arr[1]
    xyz.z = arr[2]
    if hasattr(xyz, 'w'):
        xyz.w = arr[3]
    return xyz

def assign_arr(xyz, arr=None):
    has_w = hasattr(xyz, 'w')
    if arr is None:
        arr = np.zeros(4 if has_w else 3)
    arr[0] = xyz.x
    arr[1] = xyz.y
    arr[2] = xyz.z
    if has_w: arr[3] = xyz.w
    return arr

def goto(trans, rot=np.array([0,-1,0,0]), left=False):
    planner = right_planner
    if left:
        planner = left_planner

    goal = PoseStamped()
    goal.header.frame_id = BASE_FRAME

    assign_xyz(trans, goal.pose.position)
    assign_xyz(rot, goal.pose.orientation)

    # find a plan to get there
    planner.set_pose_target(goal)
    planner.set_start_state_to_current_state()
    plan = planner.plan()

    # go there
    planner.execute(plan)
    rospy.sleep(0.5)


def lookup_transform(name):
    return tfl.lookupTransform(BASE_FRAME, name, rospy.Time(0))

def grasp():
    grip.close(block=True)

def release():
    grip.command_position(OPEN_AMOUNT, block=True)

def pickup(position):
    goto(position + OFFSET)
    goto(position)
    grasp()
    goto(position + OFFSET)


def putdown(position):
    goto(position + OFFSET)
    goto(position)
    release()
    goto(position + OFFSET)


def goto_action_pose():
    """Position Baxter's arms in the default action pose."""
    right_goal = default_action_pose[0][0]
    left_goal = default_action_pose[1][0]
    goto(left_goal, left=1)
    goto(right_goal)


def goto_image_pose():
    """Position Baxter's arms in the default imaging pose."""
    right_goal = default_image_pose[0][0]
    left_goal = default_image_pose[1][0]
    goto(right_goal)
    goto(left_goal, left=1)


def perturb():
    """Position the Camera Arm randomnly within current(x,y,z) +/- PERTURB_TOLS(0,1,2)."""
    current_pose = lookup_transform('left_hand')
    x = rand.uniform(-PERTURB_TOLS[0], PERTURB_TOLS[0]) 
    y = rand.uniform(-PERTURB_TOLS[1], PERTURB_TOLS[1])
    z = rand.uniform(-PERTURB_TOLS[2], PERTURB_TOLS[2])
    goto(tuple(x,y,z), left=1)


def cele():
    """Baxter displays prowess over his mortal opponent with a magical celebration."""
    goto(default_image_pose[0][0])
    goto(default_action_pose[1][0], left=1)


def callback(move):
    # check whether move is 'no move' or not
    if move.type == 0 or move.type == 1: # pickup-putdown request: 0 = normal, 1 = trash
        print(sty.bld + "Executing move " + move.type + ":" + sty.clr + " pickup-putdown...")
        # need to pick up a piece at strt, drop it off at dest
        strt = assign_arr(tuple(move.source.x, move.source.y, move.source.z))
        if move.type:
            dest = assign_arr(trash_bin[0])
        else:
            dest = assign_pull(tuple(move.destination.x, move.destination.y, move.destination.z))
        #  put arms in action pose, pick up at strt, put down at dest,
        #  and return to the imaging position
        goto_action_pose()
        pickup(strt) 
        putdown(dest) 
        if move.type:
            goto_action_pose()
        else:
            goto_image_pose()
    elif move.type == 2: # perturb request
        print(sty.bld + "Executing move 2:" + sty.clr + " perturb...")
        goto_image_pose()
        perturb()
    elif move.type == 3:
        print(sty.bld + "Executing move 3:" + sty.clr + " goto default default image pose...")
        goto_image_pose()
    elif move.type == 4:
        print(sty.bld + "Executing move 4:" + sty.clr + " Victory! Time to celebrate!")
        cele()
    else:
        print(sty.error + "SOMETHING TERRIBLE HAS HAPPENED!!!" + sty.clr)
    print(sty.bld + "Finished executing move." + sty.clr)


def print_tf(transform, id):
    """Given a tf transform and an associated id, prints the transform
    in an easily readable format"""
    print("TF of " + id + ":\n" + sty.bld + "trans: " + sty.clr + \
        str(transform[0]) + sty.bld + "\nrot: " + sty.clr + str(transform[1]))


def init_calib():
    global default_action_pose, default_image_pose, trash_bin
    print(sty.bld + "Initializing Baxter's Default Arm Positions:" + sty.clr)
    while 1:
        while 1:
            print("Please put Baxter's arms in the default " + sty.bld + "ACTION" + sty.clr + " pose.")
            raw_input("Press Enter to record the pose...")
            default_action_pose = [lookup_transform('right_hand'), lookup_transform('left_hand')]
            print_tf(default_action_pose[0], "Baxter's Gripper Arm, " + sty.bld + "ACTION" + sty.clr)
            print_tf(default_action_pose[1], "Baxter's Camera Arm, " + sty.bld + "ACTION" + sty.clr)
            # if 'check the angle of the shoulder':
            #     print("The the angle of the user defined gripper arm shoulder position is out of bounds.\n\
            #     Baxter will find an alternative pose...")
            #     # find pose
            #     print("Moving Baxter into the new pose...")
            #     default_action_pose()
            #     rospy.sleep(0.5)
            recal = raw_input("Recalibrate?" + sty.bld +"[y/n]" + sty.clr + ":")
            if recal == 'n':
                break

        while 1:
            print("Please put Baxter's arms in the default " + sty.bld + "IMAGING" + sty.clr + " pose.")
            raw_input("Press Enter to record the pose...")
            default_image_pose = [lookup_transform('right_hand'), lookup_transform('left_hand')]
            print_tf(default_image_pose[0], "Baxter's Gripper Arm, " + sty.bld + "IMAGING" + sty.clr)
            print_tf(default_image_pose[1], "Baxter's Camera Arm, " + sty.bld + "IMAGING" + sty.clr)
            recal = raw_input("Recalibrate?" + sty.bld +"[y/n]" + sty.clr + ":")
            if recal == 'n':
                break

        while 1: 
            print("Please put Baxter's Gripper Arm in the default " + sty.bld + "TRASH" + sty.clr + " pose.")
            raw_input("Press Enter to record the pose...")
            trash_bin = lookup_transform('right_hand')
            print_tf(trash_bin, "Baxter's Gripper Arm, " + sty.bld + "TRASH" + sty.clr)
            recal = raw_input("Recalibrate?" + sty.bld +"[y/n]" + sty.clr + ":")
            if recal == 'n':
                break

        while 1:
            test = raw_input("Would you like to goto a default pose " + sty.bld + "(a/i/t/n)" + sty.clr + "?:")
            if test == 'a':
                print("Default ACTION pose...")
                print_tf(default_action_pose[0], "Baxter's Gripper Arm, " + sty.bld + "ACTION" + sty.clr)
                print_tf(default_action_pose[1], "Baxter's Camera Arm, " + sty.bld + "ACTION" + sty.clr)
                goto_action_pose()
            elif test == 'i':
                print("Default IMAGE pose...")
                print_tf(default_image_pose[0], "Baxter's Gripper Arm, " + sty.bld + "IMAGING" + sty.clr)
                print_tf(default_image_pose[1], "Baxter's Camera Arm, " + sty.bld + "IMAGING" + sty.clr)
                goto_image_pose()
            elif test == 't':
                print("Default TRASH pose...")
                print_tf(trash_bin, "Baxter's Gripper Arm, " + sty.bld + "TRASH" + sty.clr)
                goto(trash_bin[0])
            elif test == 'n':
                break
            else:
                print(test + "is not a valid option! Please choose from " + sty.bld + "(a/i/t/n)" + sty.clr + ".")
        reinit = raw_input("Reinitialize Both Poses " + sty.bld + "(y/n)" + sty.clr + "?:")
        if reinit == 'n':
            break
    print(sty.bld + "Initialization Complete! \n Charging lasers... \n\
        Ready to exterminate my inferior competition!" + sty.clr)


class sty:
    bld = '\033[1m'
    clr = '\033[0m'
    error = '\033[31m'


if __name__ == '__main__':
    rospy.init_node('arm')

    # parse command-line arguments
    desc = 'Node to handle movement commands and positioning'
    parser = argparse.ArgumentParser(description=desc)
    parser.add_argument('-m', '--move_topic', required=True,
                        help='move message topic')
    args = parser.parse_args(rospy.myargv()[1:])

    # set up MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # set planners
    right_arm = baxter_interface.Limb('right')
    left_arm = baxter_interface.Limb('left')
    left_planner = moveit_commander.MoveGroupCommander('left_arm')
    right_planner = moveit_commander.MoveGroupCommander('right_arm')
    left_planner.set_planner_id('RRTConnectkConfigDefault')
    left_planner.set_planning_time(5)
    right_planner.set_planner_id('RRTConnectkConfigDefault')
    right_planner.set_planning_time(20)
    right_gripper = baxter_gripper.Gripper('right')
    right_gripper.calibrate()
    right_gripper.set_holding_force(hld_frc)
    right_gripper.set_moving_force(mv_frc)

    # add the object boundaries to the scene
    # ubp = [0, 0, 0] # upper box position
    # ubd = [0, 0, 0] # '       ' dimension
    # lbp = [0, 0 ,0] # lower box position
    # lbd = [0, 0, 0] # '       ' dimesnion
    # scene.addBox('upper_box', ubd[0], ubd[1], ubd[2], ubp[0], ubp[1], ubp[2], wait=False)
    # scene.addBox('lower_box', lbd[0], lbd[1], lbd[2], lbp[0], lbp[1], lbp[2], wait=False)
    # scene.waitForSync()

    # set up TF
    tfl = tf.TransformListener()

    # initialzie baxter's safe arm positions
    init_calib()

    # create our node and its listeners and publishers
    rospy.Subscriber(args.move_topic, MoveMessage, callback)
    rospy.spin()

