#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints, PositionConstraint
from geometry_msgs.msg import PoseStamped
from baxter_interface import gripper as baxter_gripper
import baxter_interface
import cv2
import tf

CLOSE_AMOUNT = 70.0
TABLE_HEIGHT = 0.0
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_node')

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
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
listener = tf.TransformListener()

def move(x=None,y=None,z=None, Q=None, right_planner=right_planner):
    trans, rot = get_coord(silent=True)
    if Q is None: Q = rot
    if x is None: x = trans[0]
    if y is None: y = trans[1]
    if z is None: z = trans[2]

    goal = PoseStamped()
    goal.header.frame_id = "base"
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z
    goal.pose.orientation.x = Q[0]
    goal.pose.orientation.y = Q[1]
    goal.pose.orientation.z = Q[2]
    goal.pose.orientation.w = Q[3]

    print 'Going to ({},{},{}) with orientation {}'.format(x,y,z, Q)

    right_planner.set_pose_target(goal)
    right_planner.set_start_state_to_current_state()
    
    right_plan = right_planner.plan()

    raw_input('Press <Enter> to move the left arm to goal pose 1 (path constraints are never enforced during this motion): ')
    
    right_planner.execute(right_plan)

    names = right_plan.joint_trajectory.joint_names
    positions = right_plan.joint_trajectory.points[-1].positions
    pose = {}
    for i in range(len(names)):
        pose[names[i]] = positions[i]

    right_arm.set_joint_positions(pose)

    return pose


def grasp(right_gripper=right_gripper):
    right_gripper.close(block=True)

def drop(right_gripper=right_gripper):
    right_gripper.command_position(CLOSE_AMOUNT, block=True)

def get_coord(silent = False):
    try:
        (trans, rot) = listener.lookupTransform('base', 'right_hand', rospy.Time(0))
        if not silent: print("The coordiantes are: " + str(trans))
        return trans, rot
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        if not silent: print("There is something wrong")
        else: raise
        
def pickup():
    move(z=TABLE_HEIGHT)
    grasp()
    move(z=TABLE_HEIGHT + 0.05)

def putdown():
    move(z=TABLE_HEIGHT)
    drop()
    move(z=TABLE_HEIGHT + 0.05)
    
def reorient():
    move(Q = (0,-1,0,0))

def get_pose(arm):
    pose = arm.endpoint_pose()
    pos = pose['position']
    Q = pose['orientation']
    return pos.x, pos.y, pos.z, [Q.x, Q.y, Q.z, Q.w]
