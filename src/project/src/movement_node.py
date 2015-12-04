#! /usr/bin/env python
from itertools import izip
import sys

import rospy
import cv2 as v
import tf
from geometry_msgs.msg import PoseStamped

import moveit_commander
from moveit_msgs.msg import (
    OrientationConstraint, Constraints, PositionConstraint )

from baxter_interface import gripper as baxter_grip
import baxter_interface

from msg import sb_msg



# the name of the world frame
BASE_FRAME = 'base'

# go 7cm above desired board positions, then down
OFFSET = np.array([0,0, 7/100])


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

def goto(trans, rot=np.array([0,-1,0,0])):
    goal = PoseStamped()
    goal.header.frame_id = BASE_FRAME

    assign_xyz(trans, goal.pose.position)
    assign_xyz(rot, goal.pose.orientation)

    # find a plan to get there
    right_planner.set_pose_target(goal)
    right_planner.set_start_state_to_current_state()
    plan = right_planner.plan()

    # go there
    right_planner.execute(plan)

    # make sure we're really there
    names = plan.joint_trajectory.joint_names
    poses = plan.joint_trajectory.points[-1].positions
    pose = dict(izip(names, poses))
    right_arm.set_joint_positions(pose)
    return pose



def lookup_transform(name):
    return tfl.lookupTransform(BASE_FRAME, frame, rospy.Time(0))

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



def move_callback(move):
    # need to pick up a piece at strt, drop it off at dest
    strt = assign_arr(move.source.translation)
    dest = assign_arr(move.destination.translation)

    # remember current position, pick up at strt, put down at dest,
    #  and return to the starting position
    trans, rot = lookup_transform('right_arm')
    pickup(strt)
    putdown(dest)
    goto(trans, rot)


if __name__ == '__main__':
    global right_planner, right_arm
    global left_planner, left_arm
    global tfl, grip

    # set up MoveIt
    moveitcommander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    left_planner = moveit_commander.MoveGroupCommander('left_arm')
    left_planner.set_planner_id('RRTConnectkConfigDEfault')
    left_planner.set_planning_time(10)
    right_planner = moveit_commander.MoveGroupCommander('right_arm')
    right_planner.set_planner_id('RRTConnectkConfigDEfault')
    right_planner.set_planning_time(10)


    # parse command-line arguments
    desc = 'Node to handle movement commands and positioning'
    parser = argparse.ArgumentParser(description=desc)
    parser.add_argument('-m', '--move', required=True,
                        help='move message topic')
    args = parser.parse_args()


    # set up Baxter and his grippers
    right_arm = baxter_interface.Limb('right')
    left_arm = baxter_interface.Limb('left')

    grip = baxter_grip.Gripper('right')
    grip.calibrate()


    # set up TF
    tfl = tf.TransformListener()


    # create our node and its listeners and publishers
    rospy.init_node('movement_node')
    rospy.Subscriber(args.move_topic, sb_msg, callback)

    rospy.spin()

