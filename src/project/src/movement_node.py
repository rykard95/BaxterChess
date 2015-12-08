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
from msg import BoradMessage



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

def goto(arm=right_arm, trans, rot=np.array([0,-1,0,0])):
    planner = right_planner
    if arm == left_arm:
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

    # make sure we're really there
    names = plan.joint_trajectory.joint_names
    poses = plan.joint_trajectory.points[-1].positions
    pose = dict(izip(names, poses))
    arm.set_joint_positions(pose)
    rospy.sleep(0.5)
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


def default_action_pos():
    # TODO: put both arms in correct positions. TFs for each can be obtained from default_action_pose array.
    right_goal = assign_arr(default_action_pose[0][0])
    left_goal = assign_arr(default_action_pose[1][0])
    goto(right_goal)
    goto(left_arm, left_goal)


def default_image_pos():
    # TODO: put both arms in correct positions. TFs for each can be obtained from default_action_pose array.
    right_goal = assign_arr(default_image_pose[0][0])
    left_goal = assign_arr(default_image_pose[1][0])
    goto(right_goal)
    goto(left_arm, left_goal)


def perturb():
    # TODO: move the camera arm every so slightly until checkerboard is in view.


def move_callback(move):
    # check whether move is 'no move' or not
    if move.type == 0 || move.type == 1: # pickup-putdown request: 0 = normal, 1 = trash
        print("Executing move " + move.type + ": pickup-putdown...")
        # need to pick up a piece at strt, drop it off at dest
        strt = assign_arr(move.source.translation)
        dest = assign_arr(move.destination.translation)

        #  put arms in action pose, pick up at strt, put down at dest,
        #  and return to the starting position
        default_action_pos():
        pickup(strt) 
        putdown(dest) 
        if move.type:
            default_action_pos()
        else:
            default_image_pos()
        print("Finished move!")
    elif move.type == 2: # perturb request
        print("Executing move 2: perturb...")
        default_image_pos()
        perturb()
        print("Finished move!")
    else:
        print("SOMETHING TERRIBLE HAS HAPPENED!!!")


def print_tf(transform, id):
    """Given a tf transform and an associated id, prints the transform
    in an easily readable format"""
    trans, rot = transform[0], transform[1]
    print("TF of " + id + ":\n" + "trans: " + trans + "\nrot: " + rot)


def init_calib():
    print("Initializing Baxter's Default Arm Positions:")
    while 1:
        print("Please put Baxter's arms in the default ACTION pose.")
        raw_input("Press Enter to record the pose...")
        default_action_pose = lookup_transform('right_arm'), lookup_transform('left_arm')
        print_tf(default_action_pose[0], "Baxter's Gripper Arm, User Defined: ")
        print_tf(default_action_pose[1], "Baxter's Camera Arm, User Defined:")
        # if 'check the angle of the shoulder':
        #     print("The the angle of the user defined gripper arm shoulder position is out of bounds.\n\
        #     Baxter will find an alternative pose...")
        #     # find pose
        #     print("Moving Baxter into the new pose...")
        #     default_action_pose()
        #     rospy.sleep(0.5)
        recal = raw_input("Recalibrate (y/n)?:")
        if recal == 'n':
            break

    while 1:
        print("Please put Baxter's arms in the default IMAGING pose.")
        raw_input("Press Enter to record the pose...")
        default_image_pose = lookup_transform('right_arm'), lookup_transform('left_arm')
        print_tf(default_iamge_pose[0], "Baxter's Gripper Arm")
        print_tf(default_iamge_pose[1], "Baxter's Camera Arm")
        recal = raw_input("Recalibrate (y/n)?:")
        if recal == 'n':
            break

    print("Stand clear; testing default positions...")
    print("Default ACTION pose...")
    default_action_pose()
    rospy.sleep(2.0)
    print("Default IMAGE pose...")
    default_image_pose()
    rospy.sleep(2.0)

if __name__ == '__main__':
    global right_planner, right_arm
    global left_planner, left_arm
    global tfl, grip
    global default_action_pose
    global default_image_pose

    
    # set up MoveIt
    moveitcommander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    
    # add the object boundaries to the scene
    ubp = [0, 0, 0] # upper box position
    ubd = [0, 0, 0] # '       ' dimension
    lbp = [0, 0 ,0] # lower box position
    lbd = [0, 0, 0] # '       ' dimesnion
    scene.addBox('upper_box', ubd[0], ubd[1], ubd[2], ubp[0], ubp[1], ubp[2], wait=False)
    scene.addBox('lower_box', lbd[0], lbd[1], lbd[2], lbp[0], lbp[1], lbp[2], wait=False)
    scene.waitForSync()

    
    # set planners
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

    
    # initialzie baxter's safe arm positions
    init_calib()


    # create our node and its listeners and publishers
    rospy.init_node('movement_node')
    rospy.Subscriber(args.move_topic, sb_msg, move_callback)
    rospy.spin()

