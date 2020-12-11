#!/usr/bin/env python

## @package motion_controller
#
# control the position of the robot in the map respecting the behaviour

import rospy
import random
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from tf import transformations
import math
import actionlib
import actionlib.msg
from exp_assignment2.msg import PlanningAction, PlanningActionGoal 

act_c = None
behaviour = None
# home position
home = [rospy.get_param('home_x'),rospy.get_param('home_y')]
home_reached = False
goal_pos = PlanningActionGoal()

## function get_random_position
#
# get a random position on the map
def get_random_position():
    randX = random.randint(-8,8) 
    randY = random.randint(-8,8) 
    randPos = [randX,randY]
    return randPos


## function get_behaviour
#
# subscriber callback to the behaviour topic
def get_behaviour(state):
    global behaviour
    behaviour = state.data

## function move_normal
#
# movement in the NORMAL state
def move_normal():
    # get a random position
    pos = get_random_position()

    # set robot goal position 
    goal_pos.goal.target_pose.pose.position.x = pos[0]
    goal_pos.goal.target_pose.pose.position.y = pos[1]
    goal_pos.goal.target_pose.pose.position.z = 0

    # send robot position and wait that the goal is reached within 15 seconds
    act_c.send_goal(goal_pos.goal)
    rospy.loginfo("Robot goal position sent!")
    rospy.loginfo(goal_pos.goal.target_pose.pose.position)
    act_c.wait_for_result(rospy.Duration.from_sec(60.0))
    rospy.loginfo("Robot has reached the goal in time")

## function move_sleep
#
# movement in the SLEEP state
def move_sleep():
    global home_reached
    
    ## go tho the home position
    if not home_reached:
        # set robot goal position 
        goal_pos.goal.target_pose.pose.position.x = home[0]
        goal_pos.goal.target_pose.pose.position.y = home[1]
        goal_pos.goal.target_pose.pose.position.z = 0

        # send robot position and wait that the goal is reached within 15 seconds
        act_c.send_goal(goal_pos.goal)
        rospy.loginfo("Robot goal position sent!")
        rospy.loginfo(goal_pos.goal.target_pose.pose.position)
        act_c.wait_for_result(rospy.Duration.from_sec(60.0))
        rospy.loginfo("Robot has reached the goal in time, now sleeps")

        home_reached = True
    
    


## function main
#
def main():
    # init node
    rospy.init_node("motion_controller")
    rate = rospy.Rate(20)
    global act_c, home_reached

    rospy.Subscriber("/behaviour", String, get_behaviour)

    # initialize action client
    act_c = actionlib.SimpleActionClient('/robot/reaching_goal', PlanningAction)
    # wait for the initialization of the server for five seconds
    act_c.wait_for_server(rospy.Duration(5))

    while not rospy.is_shutdown():
        if behaviour == "sleep":
            move_sleep()
        elif behaviour == "normal":
            # wait random time
            rospy.sleep(random.randint(1,5))
            move_normal()

        # reinitialize home_reached
        home_reached = False
        

        rate.sleep()


if __name__ == "__main__":
    main()