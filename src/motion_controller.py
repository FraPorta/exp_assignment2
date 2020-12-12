#!/usr/bin/env python3

## @package motion_controller
#
# control the position of the robot in the map respecting the behaviour

import sys
import time
import rospy
import random
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import CompressedImage
import math
import actionlib
import actionlib.msg
from exp_assignment2.msg import PlanningAction, PlanningActionGoal 

import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2
import imutils

VERBOSE = False

# default behaviour
behaviour = "normal"
# home position
home = [rospy.get_param('home_x'),rospy.get_param('home_y')]
# Action client goal init
goal_pos = PlanningActionGoal()
# action client init
act_c = None

# publishers for home poisition reaching and ball detection
pubHome = rospy.Publisher("/home_reached", Bool, queue_size = 1)
pubBall = rospy.Publisher("/ball_detected", Bool, queue_size = 1)
# home_reached publisher init
home_reached = False
ball_detected = False
near_ball = False

# cv2 circle parameters
center = None
radius = None

# ball tracking related publishers
vel_pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)
#image_pub = rospy.Publisher("/robot/output/image_raw/compressed", CompressedImage, queue_size=1)

# ball tracking related subscibersr
sub_camera = None

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

    # send robot position and wait that the goal is reached within 60 seconds
    act_c.send_goal(goal_pos.goal)
    rospy.loginfo("Robot goal position sent:")
    rospy.loginfo(goal_pos.goal.target_pose.pose.position)
    rospy.loginfo(act_c.get_state())
    # while the goal is ACTIVE, check if the behaviour changes
    while act_c.get_state() == 1:
        if behaviour != 'normal':
            act_c.cancel_goal()
            break

    # if the goal has been reached
    if act_c.get_state() == 3:
        rospy.loginfo("Robot has reached the goal in time")

## function move_sleep
#
# movement in the SLEEP state
def move_sleep():
    global home_reached
    
    # set robot goal position 
    goal_pos.goal.target_pose.pose.position.x = home[0]
    goal_pos.goal.target_pose.pose.position.y = home[1]
    goal_pos.goal.target_pose.pose.position.z = 0

    # send robot position and wait that the goal is reached within 60 seconds
    act_c.send_goal(goal_pos.goal)
    rospy.loginfo("Robot goal position sent!")
    rospy.loginfo(goal_pos.goal.target_pose.pose.position)
    act_c.wait_for_result(rospy.Duration.from_sec(60.0))
    rospy.loginfo("Robot has reached the home position in time, now sleeps")
    home_reached = True

## function move_play
#
# movement in the PLAY state
def move_play():
    # if the ball is detected go towards it and start following it
    if ball_detected:
        # if near enough to the ball start following it
        if near_ball:
            vel = Twist()
            vel.angular.z = 0.002*(center[0]-400)
            vel.linear.x = -0.01*(radius-100)
            vel_pub.publish(vel)
        # if not near enough go towards the ball
        else:
            vel = Twist()
            vel.linear.x = 0.5
            vel_pub.publish(vel)
    # if the ball is not detected search it
    else:
        vel = Twist()
        vel.angular.z = 0.5
        vel_pub.publish(vel)



def callback(ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE:
            print ('received image of type: "%s"' % ros_data.format)
        
        global ball_detected, near_ball

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

        greenLower = (50, 50, 20)
        greenUpper = (70, 255, 255)

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # set ball detected to True
            ball_detected = True 

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)

                near_ball = True
            else:
                near_ball = False

        else:
            ball_detected = False

        # publish if the ball has been detected
        pubBall.publish(ball_detected)

        # update the points queue
        # pts.appendleft(center)
        cv2.imshow('window', image_np)
        cv2.waitKey(2)


## function main
#
def main():
    # init node
    rospy.init_node("motion_controller")
    rate = rospy.Rate(20)
    global act_c, home_reached

    # ball tracking related subscibersr
    sub_camera = rospy.Subscriber("/robot/camera1/image_raw/compressed", CompressedImage, callback,  queue_size=1)

    # subscriber to current behaviour
    rospy.Subscriber("/behaviour", String, get_behaviour)
    
    # initialize action client
    act_c = actionlib.SimpleActionClient('/robot/reaching_goal', PlanningAction)
    # wait for the initialization of the server for 10 seconds
    act_c.wait_for_server(rospy.Duration(10))

    while not rospy.is_shutdown():
        # move the robot according to behaviour
        if behaviour == "sleep":
            if not home_reached:
                move_sleep()
                if home_reached:
                    pubHome.publish(home_reached)
        else:   
            # reinitialize home_reached
            home_reached = False

            if behaviour == "normal":
                # wait random time
                rospy.sleep(random.randint(1,5))
                move_normal()
        
            elif behaviour == "play":
                move_play()


        

if __name__ == "__main__":
    main()