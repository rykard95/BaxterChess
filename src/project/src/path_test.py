#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints, PositionConstraint
from geometry_msgs.msg import PoseStamped
from baxter_interface import gripper as baxter_gripper
import cv2

def pickup(hand):
    hand.command_position(0.0)
    rospy.sleep(1.0)

def putdown(hand):
    print('Opening...')
    hand.command_position(100.0)
    rospy.sleep(1.0)

def main():
    #Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    #Start a node
    rospy.init_node('moveit_node')

    #Initialize both arms
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    right_arm = moveit_commander.MoveGroupCommander('right_arm')
    left_arm.set_planner_id('RRTConnectkConfigDefault')
    left_arm.set_planning_time(5)
    right_arm.set_planner_id('RRTConnectkConfigDefault')
    right_arm.set_planning_time(10)

    #Set up the left gripper
    left_gripper = baxter_gripper.Gripper('right')

    #Calibrate the gripper (other commands won't work unless you do this first)
    print('Calibrating...')
    left_gripper.calibrate()
    rospy.sleep(2.0)

    #First goal pose ------------------------------------------------------
    goal_1 = PoseStamped()
    goal_1.header.frame_id = "base"
    while True:
        try:
            x = float(raw_input("Enter an x: "))
        except:
            print("Same x")
        try:
            y = float(raw_input("Enter a y: "))
        except:
            print("Same y")
        try:
            z = float(raw_input("Enter a z: "))
        except:
            print("Same z")
        #while True:
        #x, y, and z position
        goal_1.pose.position.x = x
        goal_1.pose.position.y = y
        goal_1.pose.position.z = z
        
        #Orientation as a quaternion
        goal_1.pose.orientation.x = 0.0
        goal_1.pose.orientation.y = -1.0
        goal_1.pose.orientation.z = 0.0
        goal_1.pose.orientation.w = 0.0

        # print('Opening...')
        # left_gripper.open(block=True)
        # rospy.sleep(1.0)
        # orien_const = OrientationConstraint()
        # orien_const.link_name = "left_gripper";
        # orien_const.header.frame_id = "base";
        # orien_const.orientation.y = -1.0;
        # consts = Constraints()
        # consts.orientation_constraints = [orien_const]
        # left_arm.set_path_constraints(consts)

        #Set the goal state to the pose you just defined
        right_arm.set_pose_target(goal_1)

        #Set the start state for the left arm
        right_arm.set_start_state_to_current_state()

        #Plan a path
        for __ in range(20):
            right_plan = right_arm.plan()
            if right_plan != []:
                break

        #Execute the plan
        raw_input('Press <Enter> to move the left arm to goal pose 1 (path constraints are never enforced during this motion): ')
        
        #putdown(left_gripper)
        right_arm.execute(right_plan)

        

        #pickup(left_gripper)
       

        #Close the left gripper
        # print('Closing...')
        # left_gripper.close(block=True)
        # rospy.sleep(1.0)

        #Open the left gripper
        # print('Opening...')
        # left_gripper.open(block=True)
        # rospy.sleep(1.0)
        # print('Done!')


    #Second goal pose -----------------------------------------------------
    # rospy.sleep(2.0)
    # goal_2 = PoseStamped()
    # goal_2.header.frame_id = "base"

    # #x, y, and z position
    # goal_2.pose.position.x = 0.5
    # goal_2.pose.position.y = 0.5
    # goal_2.pose.position.z = 0.2
    
    # #Orientation as a quaternion
    # goal_2.pose.orientation.x = 0.0
    # goal_2.pose.orientation.y = -1.0
    # goal_2.pose.orientation.z = 0.0
    # goal_2.pose.orientation.w = 0.0

    # #Set the goal state to the pose you just defined
    # left_arm.set_pose_target(goal_2)

    # #Set the start state for the left arm
    # left_arm.set_start_state_to_current_state()

    # # #Create a path constraint for the arm
    # # #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
    
    # # orien_const.absolute_x_axis_tolerance = 0.2;
    # # orien_const.absolute_y_axis_tolerance = 0.2;
    # # orien_const.absolute_z_axis_tolerance = 0.2;
    # # orien_const.weight = 1.0;
    

    # pos_const = PositionConstraint()

    # #Plan a path
    # left_plan = left_arm.plan()

    # #Execute the plan
    # raw_input('Press <Enter> to move the left arm to goal pose 2: ')
    # left_arm.execute(left_plan)


    # #Third goal pose -----------------------------------------------------
    # rospy.sleep(2.0)
    # goal_3 = PoseStamped()
    # goal_3.header.frame_id = "base"

    # #x, y, and z position
    # goal_3.pose.position.x = 0.0
    # goal_3.pose.position.y = 0.7
    # goal_3.pose.position.z = 0.0
    
    # #Orientation as a quaternion
    # goal_3.pose.orientation.x = 0.0
    # goal_3.pose.orientation.y = -1.0
    # goal_3.pose.orientation.z = 0.0
    # goal_3.pose.orientation.w = 0.0

    # #Set the goal state to the pose you just defined
    # left_arm.set_pose_target(goal_3)

    # #Set the start state for the left arm
    # left_arm.set_start_state_to_current_state()

    # #Create a path constraint for the arm
    # # #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
    # # orien_const = OrientationConstraint()
    # # orien_const.link_name = "left_gripper";
    # # orien_const.header.frame_id = "base";
    # # orien_const.orientation.y = -1.0;
    # # orien_const.absolute_x_axis_tolerance = 0.1;
    # # orien_const.absolute_y_axis_tolerance = 0.1;
    # # orien_const.absolute_z_axis_tolerance = 0.1;
    # # orien_const.weight = 1.0;
    # # consts = Constraints()
    # # consts.orientation_constraints = [orien_const]
    # # left_arm.set_path_constraints(consts)

    # #Plan a path
    # left_plan = left_arm.plan()

    # #Execute the plan
    # raw_input('Press <Enter> to move the left arm to goal pose 3: ')
    # left_arm.execute(left_plan)

if __name__ == '__main__':
    main()