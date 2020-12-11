#!/usr/bin/env python

## @package human_interaction_generator
#
# a pointing gesture generator (a IntList) with random delays

import rospy
import random
from geometry_msgs.msg import PoseStamped
import actionlib
import actionlib.msg
from exp_assignment2.msg import PlanningAction, PlanningActionGoal


act_c = None
goal_pos = PlanningActionGoal()
## function get_random_position
#
# get a random position on the map for the ball
def get_random_position():
    randX = random.randint(-8, 8)
    randY = random.randint(-8, 8)
    randZ = 1
    randPos = [randX, randY, randZ]
    return randPos

## function move_ball
#
# move the ball in a random position on the map
def move_ball():
    # get a random position
    pos = get_random_position()

    # set ball goal position 
    goal_pos.goal.target_pose.pose.position.x = pos[0]
    goal_pos.goal.target_pose.pose.position.y = pos[1]
    goal_pos.goal.target_pose.pose.position.z = pos[2]

    # send ball position and wait that the goal is reached within 15 seconds
    g_state = act_c.send_goal_and_wait(goal_pos.goal,rospy.Duration(15),rospy.Duration(10))
    rospy.loginfo("Goal state: %s" % g_state)
    #rospy.loginfo("Ball goal position published!")


## function ball_disappear
#
# make the ball disappear under the map
def ball_disappear():
    # get a random position
    pos = get_random_position()

    # set ball goal position under the map
    goal_pos.goal.target_pose.pose.position.x = pos[0]
    goal_pos.goal.target_pose.pose.position.y = pos[1]
    goal_pos.goal.target_pose.pose.position.z = -1

    # send ball position and wait that the goal is reached within 15 seconds
    g_state = act_c.send_goal_and_wait(goal_pos.goal,rospy.Duration(15),rospy.Duration(10))
    rospy.loginfo("Goal state: %s" % g_state)
    #rospy.loginfo("Ball should disappear!")


## function main
#
def main():
    # init node
    rospy.init_node("human_interaction_generator")
    rate = rospy.Rate(20)
    global act_c

    # initialize action client
    act_c = actionlib.SimpleActionClient('/reaching_goal_ball', PlanningAction)
    # wait for the initialization of the server for five seconds
    act_c.wait_for_server(rospy.Duration(5))

    while not rospy.is_shutdown():
        # wait random time
        rospy.sleep(random.randint(10,15))

        if random.randint(1,2) == 1:
            # move the ball
            move_ball()
        else:
            # make the ball disappear
            ball_disappear()

        rate.sleep()


if __name__ == "__main__":
    main()
