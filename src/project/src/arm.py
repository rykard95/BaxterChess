#! /usr/bin/env python
from itertools import izip
import sys, argparse
import numpy as np
import random as rand
import pickle
import os

import rospy
import cv2 as v
import tf, moveit_commander
from baxter_interface import gripper as baxter_gripper
import baxter_interface

from moveit_msgs.msg import (
    OrientationConstraint, Constraints, PositionConstraint )
from moveit_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from project.msg import *
from std_msgs.msg import Int8


# the name of the world frame
BASE_FRAME = 'base'

# perturb tolerances.
PERTURB_TOLS = [0.05, 0.05, 0.05]

# go 7cm above desired board positions, then down
OFFSET = np.array([0,0, 0.07])

CLOSE_AMOUNT = 100.0
OPEN_AMOUNT = 100.0
hld_frc = 1
mv_frc = 25

def lookup_transform(name):
    trans, rot = tfl.lookupTransform(BASE_FRAME, name, rospy.Time(0))
    x, y, z, rott = get_pose(name)
    trans = np.array(trans) + np.array([x,y,z])
    rot = np.array(rot) + np.array(rott)
    return trans/2, rot/2

def get_z():
    return rarm_coord[0][2]

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
    pub.publish(1)
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

def get_pose(arm):
    arm = left_arm if arm=='left_hand' else right_arm
    pose = arm.endpoint_pose()
    pos = pose['position']
    Q = pose['orientation']
    return pos.x, pos.y, pos.z, [Q.x, Q.y, Q.z, Q.w]


def grasp():
    right_gripper.close(block=True)
    # TODO: check if close() can take in CLOSE_AMOUNT
    # update: apparently it does not take in a close amount arg. 
    # http://api.rethinkrobotics.com/baxter_interface/html/baxter_interface.gripper.Gripper-class.html#close

def release():
    right_gripper.command_position(OPEN_AMOUNT, block=True)

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
    pub.publish(1)
    right_goal = default_action_pose[0][0]
    left_goal = default_action_pose[1][0]
    goto(left_goal, left=1)
    goto(right_goal)


def goto_image_pose():
    """Position Baxter's arms in the default imaging pose."""
    pub.publish(1)
    right_goal = default_image_pose[0][0]
    left_goal = default_image_pose[1][0]
    goto(right_goal)
    goto(left_goal, left=1)
    rospy.sleep(1)
    pub.publish(0)

def goto_trash_pose():
    pub.publish(1)
    """Position Baxter's arms in the default trash bin pose"""
    right_goal = trash_bin[0][0]
    left_goal = trash_bin[1][0]
    goto(left_goal, left=1)
    goto(right_goal)
    release()

def perturb():
    """Position the Camera Arm randomnly within current(x,y,z) +/- PERTURB_TOLS(0,1,2)."""
    pub.publish(1)
    current_pose = lookup_transform('left_hand')
    x = rand.uniform(-PERTURB_TOLS[0], PERTURB_TOLS[0]) 
    y = rand.uniform(-PERTURB_TOLS[1], PERTURB_TOLS[1])
    z = rand.uniform(-PERTURB_TOLS[2], PERTURB_TOLS[2])
    goto(tuple(x,y,z), left=1)
    rospy.sleep(1)
    pub.publish(0)


def cele():
    """Baxter displays prowess over his mortal opponent with a magical celebration."""
    pub.publish(1)
    goto(default_image_pose[0][0])
    goto(default_action_pose[1][0], left=1)


def callback(move):
    pub.publish(1)
    rospy.sleep(1)
    move.source.z += z_coord
    move.destination.z += z_coord
    # check whether move is 'no move' or not
    if move.type == 0 or move.type == 5: # pickup-putdown request: 0 = normal, 1 = trash
        print(sty.st + "Executing move " + str(move.type) + ":" + sty.clr + " pickup-putdown...")
        # need to pick up a piece at strt, drop it off at dest
        strt = assign_arr(move.source)
        dest = assign_arr(move.destination)
        #  put arms in action pose, pick up at strt, put down at dest,
        #  and return to the imaging position
        goto_action_pose()
        pickup(strt) 
        putdown(dest)
        goto_action_pose()
        if move.type != 5:
            goto_image_pose()
    elif move.type == 1:
        print(sty.st + "Executing move 1:" + sty.clr + " pickup-put_in_trash...")
        strt = assign_arr(move.source)
        goto_action_pose()
        pickup(strt)
        goto_trash_pose()
        goto_action_pose()
    elif move.type == 2: # perturb request
        print(sty.st + "Executing move 2:" + sty.clr + " perturb...")
        goto_image_pose()
        perturb()
    elif move.type == 3:
        print(sty.st + "Executing move 3:" + sty.clr + " goto default default image pose...")
        goto_image_pose()
    elif move.type == 4:
        print(sty.st + "Executing move 4:" + sty.clr + " Victory! Time to celebrate!")
        cele()
    else:
        print(sty.er + "SOMETHING TERRIBLE HAS HAPPENED!!!" + sty.clr)
    print(sty.fin + "Finished executing move " + str(move.type) + " ." + sty.clr)


def print_tf(transform, id):
    """Given a tf transform and an associated id, prints the transform
    in an easily readable format"""
    print(sty.bld + "TF of " + id + ":\n" + sty.bld + "trans: " + sty.clr + \
        str(transform[0]) + sty.bld + "\nrot: " + sty.clr + str(transform[1]))


def print_pose(pose):
    if pose == trash_bin:
        print_tf(trash_bin[0], sty.clr + "Baxter's " + sty.kw + "Gripper Arm, TRASH" + sty.clr)
        print_tf(trash_bin[1], sty.clr + "Baxter's " + sty.kw + "Camera Arm, TRASH" + sty.clr)
    elif pose == default_action_pose:
        print_tf(pose[0], sty.clr + "Baxter's " + sty.kw + "Gripper Arm, ACTION" + sty.clr)
        print_tf(pose[1], sty.clr + "Baxter's " + sty.kw + "Camera Arm, ACTION" + sty.clr)
    elif pose == default_image_pose:
        print_tf(pose[0], sty.clr + "Baxter's " + sty.kw + "Gripper Arm, IMAGING" + sty.clr)
        print_tf(pose[1], sty.clr + "Baxter's " + sty.kw + "Camera Arm, IMAGING" + sty.clr)
    else:
        print(sty.er + "Not a default pose!")


def def_poses_test():
    while 1:
        test = raw_input("Would you like to goto a default pose?" + sty.bld + " [a/i/t/n]" + sty.clr + ":")
        if test == 'a':
            print_pose(default_action_pose)
            goto_action_pose()
        elif test == 'i':
            print_pose(default_image_pose)
            goto_image_pose()
        elif test == 't':
            print_pose(trash_bin)
            goto_trash_pose()
        elif test == 'n':
            break
        else:
            print(sty.er + test + " is not a valid option! Please choose from " + sty.bld + " [a/i/t/n]" + sty.clr + ".")


def init_calib(def_poses_file):
    global default_action_pose, default_image_pose, trash_bin

    print(sty.st+"Calibrating z-coordinate:"+sty.clr + "\nPlace Baxter's Gripper Arm on the chess table.")
    raw_input("Press Enter to record position:")
    global z_coord
    z_coord = lookup_transform('right_hand')[0][2]
    print("The z_coord is: " + sty.kw + str(z_coord) + sty.clr)

    print(sty.st + "Initializing Baxter's Default Arm Positions:" + sty.clr)
    file_choice = raw_input("Current default pose file is " + sty.fl + def_poses_file + sty.clr + 
        ". Would you like to change files?" + sty.yn)

    if file_choice == 'y':
        new_file = raw_input("Enter the desired default pose file:")
        def_poses_file = new_file

    if not os.path.isfile(def_poses_file):
        open(def_poses_file, 'w+')
        print(sty.fl + def_poses_file + sty.clr + " has been created.")

    done = False
    def_poses_arr = [None, None, None]
    if os.stat(def_poses_file).st_size == 0:
        print(sty.fl + def_poses_file + sty.clr + " is empty. Please continue with the initialization.")
    else:
        f = open(def_poses_file, 'rb')
        def_poses_arr = pickle.load(f)
        default_action_pose, default_image_pose, trash_bin = def_poses_arr[0], def_poses_arr[1], def_poses_arr[2]

        view = raw_input("Would you like to view the current default poses in " + sty.fl + def_poses_file + sty.clr + "?" + sty.yn)
        if view == 'y':
            print_pose(default_action_pose)
            print_pose(default_image_pose)
            print_pose(trash_bin)

        test = raw_input("Test current the default poses?" + sty.yn)
        if test == 'y':
            def_poses_test()

        new_init = raw_input("Use the current default poses?" + sty.yn)
        if new_init == 'y':
            done = True
        f.close()

    while not done:
        default_action_pose = None
        default_image_pose = None
        trash_bin = None

        while 1:
            print("Please put Baxter's arms in the default " + sty.kw + "IMAGING" + sty.clr + " pose.")
            raw_input("Press " + sty.blk + "Enter" +  sty.clr + " to record the pose:")
            default_image_pose = [lookup_transform('right_hand'), lookup_transform('left_hand')]
            print_pose(default_image_pose)
            recal = raw_input("Recalibrate?" + sty.yn)
            if recal == 'n':
                break

        while 1:
            print("Please put Baxter's arms in the default " + sty.kw + "ACTION" + sty.clr + " pose.")
            raw_input("Press " + sty.blk + "Enter" +  sty.clr + " to record the pose:")
            default_action_pose = [lookup_transform('right_hand'), lookup_transform('left_hand')]
            print_pose(default_action_pose)
            # if 'check the angle of the shoulder':
            #     print("The the angle of the user defined gripper arm shoulder position is out of bounds.\n\
            #     Baxter will find an alternative pose...")
            #     # find pose
            #     print("Moving Baxter into the new pose...")
            #     default_action_pose()
            #     rospy.sleep(0.5)
            recal = raw_input("Recalibrate?" + sty.yn)
            if recal == 'n':
                break

        while 1: 
            print("Please put Baxter's Gripper Arm in the default " + sty.kw + "TRASH" + sty.clr + " pose.")
            raw_input("Press " + sty.blk + "Enter" +  sty.clr + " to record the pose:")
            trash_bin = [lookup_transform('right_hand'), lookup_transform('left_hand')]
            print_pose(trash_bin)
            recal = raw_input("Recalibrate?" + sty.yn)
            if recal == 'n':
                break

        def_poses_test()
        reinit = raw_input("Reinitialize All Poses?" + sty.yn)
        if reinit == 'n':
            def_poses_arr[0] = default_action_pose
            def_poses_arr[1] = default_image_pose
            def_poses_arr[2] = trash_bin
            break
    pickle.dump(def_poses_arr, open(def_poses_file, 'wb')) 
    print("Default poses have been save to " + sty.fl + def_poses_file + sty.clr + ".")

    print(sty.fin + "Initialization Complete!" + sty.kw + "\nCharging lasers..."
        +"Ready to exterminate my inferior competition!" + sty.clr)
    goto_image_pose()

class sty:
    clr = '\033[0m'
    bld = clr + '\033[1m'
    er = bld + '\033[37m' + '\033[41m'
    yn = bld + " [y/n]" + clr + ":"
    kw = bld + '\033[36m'
    st = bld + '\033[35m'
    fin = bld + '\033[32m' 
    blk = bld + '\033[5m'
    fl = kw + '\033[4m'


if __name__ == '__main__':
    rospy.init_node('arm')

    # parse command-line arguments
    desc = 'Node to handle movement commands and positioning'
    parser = argparse.ArgumentParser(description=desc)
    parser.add_argument('-m', '--move_topic', required=True,
                        help='move message topic')
    parser.add_argument('-e', '--eye_topic', required=True)
    parser.add_argument('-f', '--def_poses_file', default='def_poses.txt')
    args = parser.parse_args(rospy.myargv()[1:])

    # set up MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    #scene_publisher = rospy.Publisher('planning_scene', moveit_msgs.msg.PlanningScene)


    # set planners
    right_arm = baxter_interface.Limb('right')
    left_arm = baxter_interface.Limb('left')
    left_planner = moveit_commander.MoveGroupCommander('left_arm')
    right_planner = moveit_commander.MoveGroupCommander('right_arm')
    left_planner.set_planner_id('RRTConnectkConfigDefault')
    left_planner.set_planning_time(20)
    right_planner.set_planner_id('RRTConnectkConfigDefault')
    right_planner.set_planning_time(20)
    right_gripper = baxter_gripper.Gripper('right')
    right_gripper.calibrate()
    right_gripper.set_holding_force(hld_frc)
    right_gripper.set_moving_force(mv_frc)

    # add the object boundaries to the scene
    # ubp = [0, 0, 0] # upper box position
    # ubd = [0, 0, 0] # '       ' dimension
    # lbp = [0, -0.5, -1.1] # lower box position
    # lbd = [1, 1, 1] # '       ' dimesnion
    # scene.addBox('upper_box', ubd[0], ubd[1], ubd[2], ubp[0], ubp[1], ubp[2], wait=False)
    #pose = PoseStamped()
    #pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = [0,0,-.1]
    #pose.pose.orientation.w = 1
    #scene.add_plane('table_plane', pose, (0,0,1))
    # set up TF
    tfl = tf.TransformListener()

    # create our node and its listeners and publishers
    rospy.Subscriber(args.move_topic, MoveMessage, callback)
    pub = rospy.Publisher(args.eye_topic, Int8, queue_size=5)

    # initialzie baxter's safe arm positions
    pub.publish(1)
    init_calib(args.def_poses_file)
    pub.publish(0)

    rospy.spin()

