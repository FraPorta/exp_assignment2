# Assignment 2 Experimental Robotics 

## Introduction
This architecture is intended to spawn a robot in a simulated environment, where a human and a ball are spawned too, and make it follow three different behaviours: Normal, Sleep and Play. The dummy human present on the map represents the user who controls the movements of the ball in the environment (which is in reality done by a software component of the architecture).

## Software architecture and state diagram
### Architecture

<p align="center"> 
<img src="https://github.com/FraPorta/Itslit/blob/master/exp_assignment2_v2.png?raw=true">
</p>

#### Components
* Human Interaction Generator
* Behaviour Controller
* Motion Controller
* Ball Tracking 

#### Description
The main architecture is composed by four components, three of them are related to the robot control (one for the behaviour and two for the movement in the map) and the other one represents the user who gives commands to the ball (move in a certain point or go underground). 

The **Human Interaction Generator** component gives goal positions to the ball, using a SimpleActionClient and blocks until the goal is reached, then it sleeps for a random number of seconds between 5 and 10, so that the ball remains still.\
There are two possibilities: sending a random goal position over the ground and sending the ball underground (always in the same position [0 0 -1]). This choice is made randomly, and at each iteration there is a 25% possibility for the ball to disappear under the ground.

The **Behaviour Controller** component contains the finite state machine and is responsible of changing the behaviour of the robot publishing the state on a topic every time it changes, so that the other components can change their behaviour accordingly. The three behaviours are: Normal (which is the initial one), Sleep and Play. The details will be covered in the State Machine section. It subscribes to the */ball_detected* topic in order to change from the Normal to the Play state and to the */home_reached* topic to change between Sleep and Normal.

The **Motion Controller** component handles the robot motion when the behaviour is set to Normal or Sleep. It subscribes to the */behaviour* topic in order to get the current state. Moreover it instantiates a SimpleActionClient that communicates to the Action Server *Go to point robot* in order to request the goal positions that the robot should reach.
In the Normal state, it chooses a random position in the environment, getting a random number for the x and y position between -8 and 8, which are the maximum dimensions of the simulated map. Then it sends the goal to the Server and waits for it to be achieved. If the state changes when a goal has yet to be reached, using the callback of the *send_goal* function it cancels the current goal so that the robot can change its behaviour accordingly.\
In the Sleep state the motion controller sends the home position (retrieved from the Ros parameters) as the goal to the aAction Server and waits. When the goal is reached it publishes on the */home_reached* topic to alert the Behaviour controller that the robot is at home.

The **Ball tracking** component implements the openCv algorithm to detect the ball (more precisely the color of the ball) and makes the robot follow the ball when the actual behaviour is Play. It subscribes to the robot camera topic (*/robot/camera1/image_raw/compressed*) and, inside the subscriber callback, it uses the OpenCv libraries to detect the ball in the environment. When the ball is detected it immediately sends a message on the */ball_detected* topic for the Behaviour controller. Then when the state is transitioned to the Play one, it publishes velocities to the */robot/cmd* topic in order to make the robot follow the ball. When the ball stops, and so also the robot stops, it stops tracking the ball and publish commands to the */robot/joint_position_controller/command* topic to make the head revolute joint of the robot move to the right, then to the left and then back to the default position. After having finished it returns to track the ball and follow it until the state changes or the ball stops again.

#### Action Servers
* Go to point robot
* Go to point ball

The **Go to point robot** is a node that is used by the Motion controller, which implements a SimpleActionClient instance as already mentioned. The robot SimpleActionServer receives a goal from the Motion controller and publish velocities to the */robot/cmd_vel* topic and gives feedbacks until the goal is reached. It has three states: one to fix the robot yaw towards the goal, one to go straight ahead and the last one to stop when the goal is reached.

The **Go to point ball** is very similar to the robot one, but it controls the ball movement, publishing on the topic */ball/cmd_vel*, which consists only in controlling its linear velocities along x, y and z, so there is no yaw control in this case.
The states are only two: one for the movement towards the goal and one to stop when the goal is reached.

#### Ros Parameters
* home_x &rarr; home x position on the map
* home_y &rarr; home y position on the map

#### Ros Topics 
I will list only the ros topics directly related to the code that I developed (not the ones only related to gazebo, ros_control, or the ones created by the Action Servers)
* /behaviour &rarr; topic on which the current state is published by the behaviour controller
* /ball_detected &rarr; topic on which it is published when the ball is detected or not using a Bool
* /home_reached &rarr; topic on which it is published when the home is reached or not during the Sleep behaviour using a Bool
* /robot/joint_position_controller/command &rarr; topic used to move the robot head joint
* /robot/camera1/image_raw/compressed &rarr; (/robot/camera in the diagram) topic used to retrieve the images from the robot camera 
* /robot/cmd_vel &rarr; topic used to publish velocities to the robot by the ball tracking component and the Go to point robot Action Server
* /robot/odom &rarr; topic used to get the robot odometry in the Go to point robot Action Server
* /robot/reaching_goal &rarr; family of topics of the action server for the robot
* /ball/cmd_vel &rarr; topic used to publish velocities to the ball by the Go to point ball Action Server
* /ball/reaching_goal &rarr; family of topics of the action server for the ball


### State Machine
This is the state machine inside the Behaviour Controller component
<p align="center"> 
<img src="https://github.com/FraPorta/Itslit/blob/master/state_diagram_2.png?raw=true">
</p>

The **Normal** behaviour consists in moving randomly around the map. Whenever the ball is detected by the camera, it goes to the Play behaviour, otherwise it can randomly go to the Sleep state after at least 30 seconds have passed in the Normal behaviour.

The **Sleep** behaviour consists in going to the home position and staying there for some time. The transition to the Normal state happens after a random time period (20-40 seconds), that starts after the robot has reached the home position. 

In the **Play** behaviour the robot simply follows the ball. When the ball stops (and so also the robot stops) it moves his head 45 degrees on one side, wait some seconds, move it to the other side, wait some seconds and return to the zero position (this is managed by the ball_tracking node). When the ball is not detected anymore, a counter starts and if it reaches 15 seconds without seeing the ball again, it returns to the Normal state. This time has been chosen because it is approximately the time that the robot takes to make a 360 degree turn around himself, such that if the ball is not underground, it will be detected again.

## Contents of the repository
Here the content of the folders contained in this repository is explained
### Action
Contains the definition of a custom action message
### Config
Contains the yaml configuration file for the joint_position_controller and joint_state_controller, managed by the ros_control plugin
### Documentation
Contains the html documentation of the project (in order to see it, open the *index.html* file in a web browser like Chrome or Firefox)
### Launch
Contains two launch files. One (*gazebo_world.launch*) is for showing on gazebo the simulated world and spawning  the human, the robot, the ball and their relative action servers and joint controller.
The other one is for the behaviour architecture that manages the robot and ball movements.
### Scripts
Contains the two action servers python files: *go_to_point_ball.py* and *go_to_point_robot.py*
### Src
Contains the four python files (the components) of the main architecture: *human_interaction_gen.py*, *behaviour_controller.py*, *motion_controller.py* and *ball_tracking.py*
### Urdf
Contains the descriptions of the robot model, the ball and their relative gazebo files, and the description of the human. The description of the robot has been modified to include two new links and corresponding joints, a fixed one for the neck and a revolute for the head. Moreover a transmission motor has been included to make the robot head position controllable using the ros_control plugin.
### Worlds
in the worlds folder there is the description of the simulation world that will be loaded on gazebo.


## Installation and running procedure
The first thing to do, after having cloned the repository in the Ros workspace, is to build the package, using the following command in the workspace:
    
```console
catkin_make
```
In order to run the system, you have to launch the two following launch files in this order, the first one loads the gazebo world and runs the action servers, the second one runs the rest of the architecture:

```console
roslaunch exp_assignment2 gazebo_world.launch
roslaunch exp_assignment2 behaviour_architecture.launch 
```
You need to have the ros_control and gazebo ros_control related packages, if not some problems may arise in the Play behaviour.

## Working hypothesis and environment

## System’s features

## System’s limitations

## Possible technical improvements


## Rqt_graph
### Main Architecture and Gazebo Simulation
<p align="center"> 
<img src="https://github.com/FraPorta/Itslit/blob/master/rosgraph_exp_ass2.png?raw=true">
</p>

## Author
Francesco Porta\
E-mail: francy857@gmail.com\
ID: 4376330
