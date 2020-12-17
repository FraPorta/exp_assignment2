# Assignment 2 Experimental Robotics 

## Introduction
This architecture is intended to spawn a robot in a simulated environment, where a human and a ball are spawned too, and make it follow three different behaviours: Normal, Sleep and Play. The dummy human present on the map represents the user who controls the movements of the ball in the environment (which is in reality done by a software component of the architecture).

## Software architecture and state diagram
### Architecture

<p align="center"> 
<img src="">
</p>

#### Components
* Human Interaction Generator
* Behaviour Controller
* Motion Controller
* Ball Tracking

#### Description
The main architecture is composed by four components, three of them are related to the robot control (one for the behaviour and two for the movement in the map) and the other one represents the user who gives commands to the ball (move in a certain point or go underground). 

The **Human Interaction Generator** component simulates a User who sends a pointing position to the Motion controller at random intervals, using a Ros message. It is coded in order to simulate a "stupid" user who doesn't wait for the pet to be able to accomplish its request, but sends the positions totally randomly. During the developement the component was initially coded to publish messages only when the pet's behaviour was in Play, then I modified it for testing purpouses, making it completely random.

The **Voice Command Generator** component simulates a User who gives voice commands to the robot at random intervals, using a Ros message. It uses the same assumptions and development sequence of the other User component.

The **Behaviour Controller** component contains the finite state machine and is responsible of changing the behaviour of the pet publishing the state on a topic every time it changes, so that the other components change their behaviour accordingly. The three behaviours are: Normal (which is the initial one), Sleep and Play. The details will be covered in the State Machine section. It subscribes to the Voice Command topic in order to change from the Normal to the Play state.

The **Motion Controller** component simulates the robot movements with corresponding random delays according to the current state of the state machine, retrieved from the behaviour topic. It also instantiates the Map and send on a topic the actual position of the robot every time it changes.\
In the Normal state, it simulates the pet moving randomly on the Map.\ 
In the Sleep state it simulates the pet moving to the home position, and staying there until the state return to Normal.\
In the Play state, it simulates the pet going to the position of the user, waiting for a pointing position, and then reaching it. 

#### Simulator
The **Simulator** is a component that subscribes to all the topics of the architecture and shows on the command window the actual state, position of the robot and the user commands when they are published.

#### Ros Parameters
* home_x &rarr; home x position on the map
* home_y &rarr; home y position on the map

#### Ros Topics
* /behaviour &rarr; topic on which the current behaviour is published when modified

### State Machine
This is the state machine inside the Behaviour Controller component
<p align="center"> 
<img src="https://github.com/FraPorta/Itslit/blob/master/state_diagram.png?raw=true">
</p>

The **Normal** behaviour consists in moving randomly around the map. If the ball is detected by the camera, it goes to the Play behaviour, otherwise it can randomly go to the Sleep state after at least 30 seconds have passed in the Normal behaviour.

The **Sleep** behaviour consists in going to the home position and staying there for some time. The transition to the Normal state happens after a random time period (20-40 seconds), that starts after the robot has reached the home position.

The **Play** behaviour is the most complex, 


## Contents of the repository
### Launch
Contains three launch files, which will be explained in the Installation and Running procedure paragraph
### Msg
This folder contains the ".msg" file needed for the positions topics (IntList).
### Src
Contains the four python files (the components) of the architecture and two subfolders, *Lib* and *Simulator*
#### Lib
Contains the PetMap class, which is used to keep the position of the robot updated 
#### Simulator
Contains the simulator python file
### Documentation
Contains the html documentation of the project (in order to see it, open the *index.html* file in a web browser like Chrome or Firefox)
## Installation and running procedure
The first thing to do, after having cloned the repository in the Ros workspace, is to build the package and install it, using the following commands in the workspace:
    
```console
catkin_make
```
You can run the whole system, including the main architecture and the simulator, with this command:
    
```console
roslaunch pet_behaviour_ pet_and_simulator.launch 
```
Otherwise you can run separately the main architecture and the simulator using two commands:

```console
roslaunch pet_behaviour_ simulator.launch
roslaunch pet_behaviour_ pet_launcher.launch 
```
You can modify the simulation speed by changing the timescale parameter in the two launchfiles above (example: timescale = 0.5 -> simulation time halved)
The simulator launchfile also contains the Smach_viewer launch which allows to visualize better the current state and the transitions between states.

## Working hypothesis and environment
The main working hypotesis that influences all the project is that the robot and the user are purely and simply simulated by software components: for the user the voice commands and the pointing gestures are simply represented by Ros messages sent at random, for the robot the movements on the map are only a random sleep time and a couple of int values that represent its position on the map, updated when the sleep time is over.\
Moreover the map is simulated using a couple of int values for the total length and width and a couple of values for each important position on the map (home, user, robot).

## System’s features
The main feature of the system is the state machine which controls the behaviour of the pet, which is coded in such a way that it can be used also in a more complex and close to reality application.\
Another feature is the fact that even simulating at a high speed (like 100 times the real time) the system does not fail and, even if this is irrealistic, it works fine and it is possible to observe a lot of iterations in a short time that give an idea of the average behaviour of the pet. Doing this, it can be observed that even if the randomness of the system is quite high, the architecture does del with that well and does not make errors in retrieveng and executing the commands given by the user.

## System’s limitations
The user and home positions on the map are predefined before starting the architecture and they do not change during the execution.\
The transition from Normal to Sleep behaviour can be improved, the probability of going to the Sleep state is not optimal because sometimes a long time passes before going to Sleep again, while some other times the robot goes to Sleep consecutively in a range of few seconds.\
If the pet is in the Normal state and the behaviour changes, it has to first reach the position it was randomly going to before actually changing the behaviour and execute the next commands.\ 
When user commands arrive in a moment where the robot can not handle it, like when it is Sleeping or moving to a pointed gesture or towards the user in the Play state, the architecture sinply ignores them.

## Possible technical improvements
If we consider the working hypotesis and the actual scenario not changing, I think that the part that can be improved consistenly is the motion controller and the simulator, which are very basic, as the specifications required, and they should be changed drastically in order to resemble a real robot behaviour: in fact, for the time being, the pet movements are not really simulated. A little adjustment that could be made without overturning the whole project is to make the waiting times for the robot movements proportional to the distance that the robot has to cover, in order to have a more realistic feeling of the behaviour of the architecture in a real scenario, specifically from the point of view of the State Machine which is actually the core of this project.
Moreover, also the simulator node can be improved in order to show the actual movement of the robot, instead of only showing when a point is reached.\
User commands that are ignored in this version of the archtiecture can be for example stored and executed when it will be possible for the robot.


## Rqt_graph
### Main Architecture and Gazebo Simulation
<p align="center"> 
<img src="https://github.com/FraPorta/Itslit/blob/master/rosgraph_exp_ass2.png?raw=true">
</p>

## Author
Francesco Porta\
E-mail: francy857@gmail.com\
ID: 4376330
