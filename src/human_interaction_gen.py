#!/usr/bin/env python

## @package human_interaction_generator
#
# a pointing gesture generator (a IntList) with random delays
 
import rospy
import random
from geometry_msgs.msg import PoseStamped 
import exp_assignment2.msg


pubPos = None
goal_pos = exp_assignment2.msg.PlanningActionGoal()

## function get_random_position
# 
# get a random position on the map for the ball
def get_random_position():
    randX = random.randint(-8,8) 
    randY = random.randint(-8,8) 
    randZ = 1
    randPos = [randX,randY,randZ]
    return randPos
        
## function main
#
def main():
    ## init node
    rospy.init_node("human_interaction_generator")
    rate = rospy.Rate(20)

    pubPos = rospy.Publisher("/reaching_goal/goal", exp_assignment2.msg.PlanningActionGoal, queue_size=1)
    pos = None

    ## get the timescale parameter to adjust simulation speed
    #timescale = rospy.get_param('timescale')
    timescale = 1

    while not rospy.is_shutdown():
        ## wait random time
        rospy.sleep(timescale*random.randint(1,10))

        pos = get_random_position()

        ## publish ball position
        goal_pos.goal.target_pose.pose.position.x = pos[0]
        goal_pos.goal.target_pose.pose.position.y = pos[1]
        goal_pos.goal.target_pose.pose.position.z = pos[2]

        rospy.loginfo(goal_pos.goal.target_pose.pose)

        pubPos.publish(goal_pos)
        rospy.loginfo("Ball goal position published!")

        rate.sleep()


if __name__ == "__main__":
    main()