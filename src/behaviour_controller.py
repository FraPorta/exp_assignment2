#!/usr/bin/env python

## @package behaviour_controller
#
# state machine to control the behaviour of the pet

import rospy
import smach
import smach_ros
import random
from std_msgs.msg import String


pub_state = rospy.Publisher("/behaviour",String,queue_size=5)

## class state Normal
#
# normal behaviour of the pet
class Normal(smach.State):
    ## method init
    #
    # state initialization
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['go_to_sleep','play_command']
                            )
        
        self.command_received = False
        self.rate = rospy.Rate(20)  # Loop at 100Hz

    ## method execute
    #
    # state execution
    def execute(self, userdata):
        pub_state.publish("normal")
        ## check if a voice command is received
        rospy.Subscriber("/voice_command", String, self.get_command)
        
        while not rospy.is_shutdown():  
            ## go to sleep at random (1/1000 chances per iteration -> 100 iterations per second -> 1/10 chance per second passed in Normal state)
            if(random.randint(1,1000) == 1):
                    return 'go_to_sleep'
            else:
                ## If the robot sees the ball goes to the play behaviour
                if():
                    
                    return 'play_command' 
            self.rate.sleep()
    
    ## method get_command
    #
    # subscriber callback for voice command
    def get_command(self, command):
        if(command.data=="play"):
            self.command_received = True
            


## class state Sleep
#
# Sleep behaviour of the pet
class Sleep(smach.State):
    ## method init
    #
    # state initialization
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['wake_up']
                            )
        self.position = [-1,-1]
        self.rate = rospy.Rate(20)
        
    ## method execute
    #
    # state execution
    def execute(self, userdata):
        pub_state.publish("sleep")
        ## get the timescale parameter to adjust simulation speed
        timescale = rospy.get_param('timescale')

        while not rospy.is_shutdown():  
            # check if the pet is in home position
            if(self.position == (rospy.get_param('home_x'),rospy.get_param('home_y'))):
                ## wait some time to wake up
                rospy.sleep(timescale*random.randint(30,60))
                return 'wake_up'
            self.rate.sleep
        
    ## method get_position
    #
    # subscriber callback, gets actual position of the robot
    def get_position(self,position):
        self.position = position.data

## class state Play
#
# Play behaviour of the pet
class Play(smach.State):
    ## method init
    #
    # state initialization
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['stop_play'],
                            )
        self.position = [-1,-1]
        self.rate = rospy.Rate(20)

    ## method execute
    #
    # state execution
    def execute(self,userdata):
        #rospy.loginfo('Executing state PLAY')
        pub_state.publish("play")
        timescale = rospy.get_param('timescale')
        
        rospy.Subscriber("/actual_position", IntList, self.get_position)

        rospy.sleep(timescale*random.randint(60,120))
        return 'stop_play'
            
        

    ## method get_position
    # subscriber callback, gets actual position of the robot
    def get_position(self,position):
        self.position = position.data

    

    
def main():
    rospy.init_node("behaviour_controller")

    ## Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])

    ## Open the container
    with sm:
        ## Add states to the container
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'go_to_sleep':'SLEEP', 
                                            'play_command':'PLAY'})

        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'wake_up':'NORMAL'})

        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'stop_play':'NORMAL'})

    ## Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    ## Execute the state machine
    outcome = sm.execute()

    ## Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()



if __name__ == "__main__":
    main()

