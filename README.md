# Experimental Robotics Laboratory - Assignment 3

The goal of this assignment is to develop a software architecture able to simulate an autonomous Cluedo game. In particular the robot has to explore an environment looking for aruco, that, once detected, correspond to specific IDs. To each ID is associated an hint used to reason about a possible solution of the game. On top, the robot should perceive hints until the consistent and correct hypothesis is found. For this purpose, a navigation algorithm to find aruco has been implemented assisted by cv_bridge and aruco libraries to collect IDs and so detect hints.

## Compatibility

The project is developed using the provided docker container. In the event that it is not possible to test the project on this image, it is also possible to use ROS Noetic on Ubuntu 20.04.2, after installing the ARMOR components on this operating system. In any case, the project is compatible with ROS Noetic, and consequently may not work using a different ROS distribution.

# Description of the package

## Robot behaviour and software architecture

At first, the robot must reach the center of a specific room. As soon as this position is reached, the robot reachs 5 random targets within the same room and rotates around itself once one these targats is reached. Then, the robot moves towards the next room. This behaviour is repeated whenever all the rooms have been visited. When the robot moves, it continously look for aruco subcribing to the cameras' topic. When a new hint is perceived it interfaces with armor to reason about that hint. Detect hint behavior and navigation are concurrent.

## ROS Nodes

The nodes are:

- [ArmorInterface.py](https://github.com/RiccardoZuppetti/exprob_ass3/blob/main/erl_assignment3/scripts/ArmorInterface.py), that is used to interface with ARMOR. The interaction can be different: load the ontology; check if the hypothesis is correct; check if there is a new consistent hypothesis; add a new hint to the ontology once it has been perceived. The referring ontology is `cluedo_ontology.owl`.
- [AnnounceHypotesis.py](https://github.com/RiccardoZuppetti/exprob_ass3/blob/main/erl_assignment3/scripts/AnnounceHypotesis.py), called for announcing to the oracle the new consistent hypothesis found.
- [FSM.py](https://github.com/RiccardoZuppetti/exprob_ass3/blob/main/erl_assignment3/scripts/FSM.py), that is the finite state machine used to synchronize the entire simulation. This node is responsible of the navigation of the robot. Moreover in order to perceive ID it subscribes to `/id_aruco` topic, once perceived it send the id as request to `/oracle_hint` service and retrieve as response the hint corresponding to that ID. On top, is a service client of `/armor_interface` service and call it when a new hint is perceived. If the hint is perceived correctly it call it to check if there is a new consistent hypothesis, and if true it call it again to check if it is also correct.
- [aruco_reader.cpp](https://github.com/RiccardoZuppetti/exprob_ass3/blob/main/aruco_ros/aruco_ros/src/aruco_reader.cpp) it is a node of the `aruco_ros` package used to perceived and detect the aruco, and the correspondent IDs are published on the `/id_aruco` topic
- [simulation.cpp](https://github.com/RiccardoZuppetti/exprob_ass3/blob/main/erl_assignment3/src/simulation.cpp), that implements the `oracle_hint` service, that takes as request an id and respond with an hint which can be correctly perceived or malformed. On top, the oracle responds if the hypothesis is correct or not.

Instead, the services are:

- [ArmorInterface.srv](https://github.com/RiccardoZuppetti/exprob_ass3/blob/main/erl_assignment3/srv/ArmorInterface.srv), which has as request the mode that the client wants in order to interact with ARMOR (0 to load ontology, 1 to check correct, 2 to check consistency, 3 to perceive an hint) and the ID of the hypothesis to be checked. The response is composed by the same mode and ID in addition to a success field, which is true if the action has been accomplished correctly.
- [Announcement.srv](https://github.com/RiccardoZuppetti/exprob_ass3/blob/main/erl_assignment3/srv/Announcement.srv), which has as request an hypothesis (composed of [who, what, where]), while the response is a boolean to state that the action has been completed correctly.
- [Marker.srv](https://github.com/RiccardoZuppetti/exprob_ass3/blob/main/erl_assignment3/srv/Marker.srv), which has as request the ID retrieved, while the response is the hint associated to the aforementioned ID.

## UML diagram

![uml](https://user-images.githubusercontent.com/89387809/200359412-4e9cc053-cda1-4bdb-aab2-28b9743fcc36.jpg)

## Rqt-graph

<img width="1743" alt="final_rqt" src="https://user-images.githubusercontent.com/89387809/200359730-c1b0e198-43b2-4468-be26-0b570767c575.png">

# How to Run

## Requirements

In order to compile this project, the following ROS packages are needed:

- [armor](https://github.com/EmaroLab/armor)
- [planning](https://github.com/CarmineD8/planning)
- [erl2](https://github.com/CarmineD8/erl2)

## How to compile the code

Move to your `catkin_ws/src/armor/armor` folder and digit (only the first time):

```
./gradlew deployApp
```

Then, move to your `catkin_ws/src` folder and clone this repository:

```
git clone https://github.com/RiccardoZuppetti/exprob_ass3.git experimental_assignment3
```

and build the workspace with the following command:

```
catkin_make
```

Copy the content of the `models` folder in the `/root/.gazebo/models` path.

## Description of the execution

To launch the project it is needed to open four shells.

In the first shell, digit:

```
roscore &
```

and then

```
rosrun armor execute it.emarolab.armor.ARMORMainService
```

In the second one digit:

```
roslaunch erl_assignment3 init_launcher.launch 2>/dev/null
```

In the third terminal digit the following command:

```
roslaunch erl_assignment3 gazebo_launcher.launch 2>/dev/null
```

Finally, in the last shell digit:

```
roslaunch erl_assignment3 game_launcher.launch 2>/dev/null
```

In the following there are videos that shos how to launch the project and the behaviour of the robot that explores the environment.

https://user-images.githubusercontent.com/89387809/200366097-3788dfc6-699d-44ce-8493-7f52a5a432d5.mp4

https://user-images.githubusercontent.com/89387809/200366573-68a95b15-2c9d-45eb-963f-ba7047e9b8d9.mp4

https://user-images.githubusercontent.com/89387809/200366636-87d0fc16-a714-4d0f-b7da-7c60b0af5f4b.mp4

There is also a screenshot that shows the exploration of the environment.

<img width="1132" alt="screen" src="https://user-images.githubusercontent.com/89387809/200367065-99c1d975-675a-4332-bb1c-9396f24eef39.png">

# Working hypothesis and environment

## System's features

The overall architecture has been designed to be as modular as possible. In particular the overall system have a single module devoted to navigation, aruco detection and ontology.

Aruco detection is implemented by the aruco_reader node of `aruco_ros` package. In particular this node read a ROS image from `/robot/camera1/image_raw` and `/robot/camera2/image_raw` topics, convert in in a openCV image using the cv_bridge and detect the aruco id using the aruco library. When the subscriber reads the data from the devoted topic the image transport is used. The image transport allows to transport the image in low-bandwidth compressed format. So it is used for reduce the consume of resources. An instance is generated and used to declare publishers and subscribers to a certain topic. In that case only a subscriber for each camera is used. A publischer of `/id_aruco` topic has been added in order to communicate the id of the perceived topic. In that case a `std_msgs/Int64` message has been used. The robot doesn't distinguish if the aruco are detected on the floor or on the wall, and for this reason only one single topic `/id_aruco` is used for both the camera.

The robot is equipped with a laser, needed for a reactive navigation. The mapping of the environment is done through Gmapping (FastSLAM). In order to use this algorithm it is necessary a robot with laser and odometry. In order to localize the robot and to build the map the gmapping subscribes data from `/tf` and `/scan`. The localization is done computing the tree of frames with respect to the map frame. As path planning the move_base approach has been used. It consists in a reactive-deliberative navigation. It has at same time local gand global planner. The robot explores goes at the centre of each room using move base. Than it rotates, then for five times in goes to a random target using move base and once arrived it rotates. When it has reach a target and it has found an hint it stops for querying the ontology. In the randomly exploring room behaviour the robot uses move_base since the apartment is quite complex and the random target can be generated near walls.

The ontology is the core of the whole game. It allows to reason about hypotesis. The hypothesis are make of hints. Hints can belong to three different classes: PERSON, PLACE and WEAPON. The perceived hints as ErlOracle message are the A-box of these concepts. The hypotesis belong to the COMPLETE class if they has at least one PERSON, one WEAPON and one PLACE, while they belong also to the INCONSISTENT class if they have more than one PERSON or PLACE or WEAPON. Consequently the consistent hypotesis are the ones that has only one entity for each hint class and they are the ones which belong to the COMPLETE class and not to the INCOSISTENT one. The INCORRECT class, instead, has as instances all the hypothesis already checked as not-correct.

## System's limitations

Concerning the system's limitations, it is possible to notice the following ones:

- the model of the robot
- the navigation algorithm, since when the robot is in proximity of a goal it blocks for few seconds
- a random target could be generated out of the referring room
- the robot could not perceive correctly an ID.
- slowness of the robot during the navigation.

## System's technical improvements

Can be labeled as improvement to what has been done:

- usage of ROSPlan instead of the implemented finite state machine
- use an already modeled robot
- divide the system in two different behaviours: in the first one the robot should explore randomly the environment in order to find the aruco; once the aruco are found, the (x,y) coordinates in which the robot is located are associated to the ID of the hint; as soon as all hints are found, the second behaviour start and the robot goes directly on the specific position to perceive again the hints.

# Author and contacts

Author: Riccardo Zuppetti

Contacts: riccardo.zuppetti@icloud.com
