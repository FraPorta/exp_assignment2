#!/usr/bin/env python

## @package motion_controller
#
# control the position of the pet in the map respecting the behaviour

import rospy
import random
from std_msgs.msg import String
from pet_behaviour.msg import IntList
from lib.pet_map import PetMap

## initialization of the map and variables
pet_map = PetMap()
goal_position = None
behaviour = None
timescale = rospy.get_param('timescale')
home = False
pub = rospy.Publisher("/actual_position",IntList,queue_size=5)
sub = None

## function get_random_position
#
# get a random position on the map
def get_random_position():
    randX = random.randint(0,rospy.get_param("map_dimension_x")) 
    randY = random.randint(0,rospy.get_param("map_dimension_y")) 
    randPos = [randX,randY]
    return randPos

## function get_position
#
# subscriber callback position
def get_position(position):
    global goal_position
    goal_position = position.data
    
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
    ## move randomly on the map
    randPos=get_random_position()
    pet_map.updateMap(randPos[0],randPos[1])
    
    ## wait random time to simulate reaching the point
    rospy.sleep(timescale*random.randint(5,15))

## function move_sleep
#
# movement in the SLEEP state
def move_sleep():
    global home
    ## go tho the home position
    if not home:
        ## wait random time to simulate reaching the point
        rospy.sleep(timescale*random.randint(5,15))
        pet_map.updateMap(rospy.get_param("home_x"),rospy.get_param("home_y"))
        home = True
    
        
## function move_to_person
#
# movement in the PLAY state
def move_to_person():
    ## go to the person position and waits for a pointing position 
    rospy.sleep(timescale*random.randint(5,15))
    pet_map.updateMap(rospy.get_param("person_x"),rospy.get_param("person_y"))

## function move_to_goal
#
# move to the point given by the user
def move_to_goal():    
    ## go to the pointed position 
    rospy.sleep(timescale*random.randint(5,15))
    pet_map.updateMap(goal_position[0],goal_position[1])
    
        



## main function
#
def main():
    rospy.init_node("motion_controller")
    ## subscribers
    rospy.Subscriber("/behaviour",String, get_behaviour)
    rospy.Subscriber("/pointing_position",IntList, get_position)
    
    rate = rospy.Rate(100)
    global home
    global goal_position

    ## pub initial position
    pub.publish([pet_map.actualX,pet_map.actualY])

    ## move according to the behaviour
    while not rospy.is_shutdown():
        ## temporary store actual position to avoid publishing the position when pet is still
        x = pet_map.actualX
        y = pet_map.actualY

        if(behaviour == "sleep"):
            move_sleep()
            ## ignore pointing command
            if not goal_position == None:
                goal_position = None
        else:
            home = False
            if(behaviour == "normal"):
                move_normal()
                ## ignore pointing command
                if not goal_position == None:
                    goal_position = None
            else:
                if(behaviour == "play"):
                    if not ((pet_map.actualX,pet_map.actualY)  == (rospy.get_param('person_x'),rospy.get_param('person_y'))):
                        move_to_person()
                    else:
                        if not goal_position == None:
                            move_to_goal()
                            goal_position = None

        
        ## publish the actual position
        if not (behaviour == None):
            ## if position is not changed don't publish it
            if not ((pet_map.actualX == x) & ((pet_map.actualY == y))):
                pub.publish([pet_map.actualX,pet_map.actualY])

        rate.sleep()


if __name__ == "__main__":
    main()

