<div id="top"></div>




<!-- PROJECT SHIELDS -->
<!--
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]





<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/fedehub/ExperimentalRoboticsAssignment2">
    <img src="media/miscellaneous/logo-black.png" alt="Logo" width="200" height="100">
  </a>

  <h3 align="center">Experimental Robotics Laboratory</h3>

  <p align="center">
    First assignment for the Experimental Robotics laboratory course 
    <br />
    <a href="https://github.com/fedehub/ExperimentalRoboticsAssignment2/doc"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/fedehub/ExperimentalRoboticsAssignment2/demo">View Demo</a>
    ·
    <a href="https://github.com/fedehub/ExperimentalRoboticsAssignment2/issues">Report Bug</a>
    ·
    <a href="https://github.com/fedehub/ExperimentalRoboticsAssignment2/issues">Request Feature</a>
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#assignments-prerequisites">Assignment's prerequisites</a></li>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#installation-procedure">Installation procedure</a></li>
        <li><a href="#workspace-building-and-launch">Workspace building and launch</a></li>
        <li><a href="#running-pocedure">Running procedure</a></li>
      </ul>
    </li>
    <li>
      <a href="#usage">Usage</a>
    </li>
    <li>
      <a href="#ros-node-description-an-overview">ROS node description: An overview</a>
      <ul>
        <li><a href="#the-main.py-node ">the main.py node</a></li>
        <li><a href="#the-cluedo_kb.py-node ">the cluedo_kb.py node</a></li>
        <li><a href="#the-action_interface.cpp-node">the action_interface.cpp node</a></li>
        <li><a href="#the-manipulation.cpp-node ">the manipulation.cpp node</a></li>
        <li><a href="#the-my_simulation.cpp-node ">the my_simulation.cpp node </a></li>
        <li><a href="#rqt_graph">rqt_graph</a></li>
        <li><a href="#uml-temporal-diagram">UML temporal diagram</a></li>
        <li><a href="#about-detectibot-model">About detectiBot model</a></li>
      </ul>
    <li>
        <a href="#Working-hypothesis-and-environment">Working hypothesis and environment</a>
    </li>
      <ul>
        <li><a href="#System's features">System's features</a></li>
        <li><a href="#System's limitations">System's limitations</a></li>
        <li><a href="#Possible technical Improvements">Possible technical Improvements</a></li>
      </ul>
    <li>
      <a href="#roadmap">Roadmap</a>
    </li>
    <li>
      <a href="#contributing">Contributing</a>
    </li>
    <li>
      <a href="#license">License</a>
    </li>
    <li>
      <a href="#contact">Contact</a>
    </li>
    <li>
      <a href="#acknowledgments">Acknowledgments</a>
    </li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

This project consists in building a simulation where our detectibot takes on the appereance of a real robot, roaming into a phisical environment (differentlwy from what occured in the first project ) containing hints.

There are four different positions in the environment (x,y,z respectively) that contains hints:
If the *cluedo_link* of the robot is reasonably close, this will trigger the oracle for the generation of a hint 

> **REMARK** x and y coordinates where known a priori as shown in the table below   

  | room  | x,y coordinates  | z coordinate |
  |--|--|--|
  | FirstMarkerPosition | ( -3,0 ) | 0.75 v 1.25  |
  | SecondMarkerPosition | ( +3,0 ) | 0.75 v 1.25 |
  | ThirdMarkerPosition | ( 0,-3 ) | 0.75 v 1.25  |
  | FourthMarkerPosition | ( 0, +3 ) | 0.75 v 1.25 |
 
Having differen values for z, it is needed that detectibot reaches both quotes with its cluedo_link

Concerining the simulation environment, there are small walls around the robot aimed at impeding the movements of its mobile base 

Hence the robot moves from one «hint» coordinate to another one, while receiving hints. This holds until it has a complete
and consistent hypothesis

<!-- Note about  consistent huèothesis -->

 Please consider that **consistent hypothesis** have been defined as COMPLETED but NOT INCONSISTENT 

> *REMARK* A consistent hypothesis is  defined as *completed* when there occurs one role for each class (i.e., one occourence of what, one occourence for who, one occourence for where ). 
A straightforward example of such hypothesis is [ID2][12], whose definition is here below reported

```txt
ID2_1: ['where', 'Study']
ID2_2: ['who', 'Col.Mustard']
ID2_3: ['what', 'Rope']
```

> *REMARK* An hypothesis, is defined as *inconsistent* when there occurs more than one role for each class (i.e. 2 or more occourences of who, where, what) 

A clear example of such hypothesis is ID4  whose definition is here below reported

```txt
ID4_1: ['where', 'Library']
ID4_2: ['who', 'Mrs.White']
ID4_3: ['what', 'LeadPipe']
ID4_4: ['where', 'Diningroom']
```
### Assignment's prerequisites

As in the first assignment:
- only one ID source is the trustable one. 
- Whenever a robot gets a complete hypothesis, it should go in the center of the arena 
- Once the center has been reached, it should «tell» its solution (as in the first assignment). 
- If the solution is the **correct one**, the game ends
- it is strictly required to use ROSPlan here, to plan the behaviour of your
robot. 
- create a pddl domain, problem and a set of actions 
- the robot of the model has no limitations, meaning that it can be modelled in whatever fashion 


<p align="right">(<a href="#top">back to top</a>)</p>



### Built With 🏗️

<!-- PLEASE INSERT HERE -->

* [ROS][4]
* [ROSPlan Framework][5]
* [MoveIt Frameowrk][6]
<!-- *[move_base][7] -->


<p align="right">(<a href="#top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started

Under the following sections, the software architecture is briefly introduced, along with the prerequisites and installation procedures. Then, a quick video demonstration showing the overall functioning is provided and system’s limitations are discussed

### Installation procedure

> :warning: To avoid further issues, please use this docker image provided by our professors 

```sh
docker run -it -p 6080:80 -p 5900:5900 --name MyDockerContainer carms84/exproblab

```
Also remember to update and upgrade the container

```sh
sudo apt get update
sudo apt get upgrade
```
Then run `catkin_make` on your workspace; in my case:

- Navigate to your ROS workspace
  ```sh
  cd /home/ros_ws/
  ```
- Run catkin
  ```sh
  catkin_make
  ```

You can now download the repository inside the `src` folder 

```sh 
cd /home/ros_ws/src
git clone https://github.com/fedehub/ExperimentalRoboticsAssignment2
```
Also download `MoveIt 1.1.5` 
```sh
git clone https://github.com/ros-planning/moveit.git
cd moveit
git checkout 2b881e5e3c4fd900d4d4310f4b12f9c4a63eb5dd
cd ..
git clone https://github.com/ros-planning/moveit_resources.git
cd moveit_resources
git checkout f6a7d161e224b9909afaaf621822daddf61b6f52
cd ..
git clone https://github.com/ros-planning/srdfdom.git
cd srdfdom
git checkout b1d67a14e45133928f9793e9ee143998219760fd
cd ..
apt-get install -y ros-noetic-rosparam-shortcuts
cd ..
catkin_make
catkin_make
catkin_make
```

## Workspace building e launch

Navigate to you workspace

```sh
cd /home/ros_ws/

```

- clone the repository

```sh
https://github.com/fedehub/ExperimentalRoboticsAssignment2

```

- source your workspace by typing

```sh
source devel/setup.bash

```

<p align="right">(<a href="#top">back to top</a>)</p>

### Running procedure

<!-- including all the steps to display the robot's behavior -->

To test the project, first of all:

- Open a shell and run:
  
```sh
roslaunch erl_moveit_pkg run_detectibot.launch 2>/dev/null

```

- Open a second shell and run 

```sh
roslaunch erl_assignment_2 run_rosplan.launch

```

- Open a third shell and type:

```sh
rosrun erl_assignment_2 main.py


```


<!-- USAGE EXAMPLES -->
## Usage

The most relevant aspects of the project and a brief video tutorial on how to launch the simulation can be found here below


https://user-images.githubusercontent.com/61761835/187249845-1b03e627-d32e-4464-b7d3-0f172419d2f9.mp4


<!-- A commented small video, a GIF or screenshots showing the relevant parts of the running code. -->





<p align="right">(<a href="#top">back to top</a>)</p>

## ROS node description: An overview 

Here there is the UML components diagram of the project

<img src="https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/component_diagrams/v3/component_diagram.jpg" >

Some remarks about the aformentioned components diagram:
- The main node simply keeps replanning until the mistery gets solved
- The rosplan block represents a set of nodes provided by the Rosplan framework
- The component action_interface represents a moltitude of nodes, each implementing one pddl action 

As shown in the above component diagram, this software architechture relies on the synergy of varius modules: 



- [cluedo_kb.py][20]              <!-- PLEASE INSERT HERE -->

- [go_to_point.py][21]            <!-- PLEASE INSERT HERE -->

- [main.py][22]                   <!-- PLEASE INSERT HERE -->

- [action_interface.cpp][20]      <!-- PLEASE INSERT HERE -->

- [manipulation.cpp][21]          <!-- PLEASE INSERT HERE -->


Here below we can find the nodes devoted for testing purposes 

- [my_simulation.cpp][26]
- [test_nav.py][23]               
  

### the go_to_point.py node  🪢

Let's start with the `go_to_point.py` node

<p align="center">
<img src="https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/component_diagrams/v1/erl_assignment_go_to_point_py.jpg" width= 500 height=500>
</p>

It implements a ROS service, whose purpose is that of piloting the robot toward a specific target by following a straight line. As it is shown by the component diagram here reported, it subscribes to the `/odom` topic for retrieving the current robot position and once the robot orientation among x and y coordinates has been computed with respect to the target position (obtained by means of the ros parameter server), it publishes on the `/cmd_vel` topic

Concerning the ros parameters:
`des_pos_x` and `des_pos_y` are used for keeping track of the target goal to be assigned to the robot in the go_to_point.py node 

Node interfaces: 
```Plain txt
Node [/go_to_point]
Publications: 
 * /cmd_vel [geometry_msgs/Twist]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /odom [nav_msgs/Odometry]

Services: 
 * /go_to_point
 * /go_to_point/get_loggers
 * /go_to_point/set_logger_level

```
### the main.py node 🪢

<p align="center">
<img src="https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/component_diagrams/v1/erl_assignment_2_main_py.jpg" width= 500 height=500>
</p>

This node represents three core structures, even the "brain" of our achitechture. Being ROSPlan a framework  that owns a variety of nodes which encapsulate planning, problem generation and plan execution, a set of clients have been initialised to, subsequently:

- generate a problem: a [pddl problem][115] is published on a topic 
- establish a plan: a planner is called for  publishing the plan to a topic 
- parse a plan: At this stage the PDDL plan is converted into ROS messages, ready to be executed
- dispatch a plan:  for being then executed 

There is also the possibility to update the Knowledge base (being it the main responsible for the PDDL domain model amd current problem istance stroage)

Indeed, if detectibot is not able to solve the mistery at the first round, it is possible to count on a "replanning phase", after which it starts roaming around the environment for gathering hints held by the markers

Node Interfaces:
```Plain txt 
Node [/main]
Publications: 
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]

Services: 
 * /main/get_loggers
 * /main/set_logger_level

```
### the cluedo_kb.py node 🪢

Concerning the `cluedo_kb.py` node:

<p align="center">
<img src="https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/component_diagrams/v1/erl_assignment_2_cluedo_kb_py.jpg" width= 500 height=500>
</p>

cluedo_KB is a node that acts as a dedicated ontology for the problem under investigation; it provides a processing/reasoning system that provides the functionalities of:

- registering the clues
- building and processing hypotheses based on the added information
- finding possible solutions to the case
- rejecting hypotheses

> ***REMARK*** the KB listens in on the oracle's topic and as soon as the oracle transmits the clue, the KB adds the message to the ontology without the need for an explicit request

Node interfaces:
```Plain txt
Node [/cluedo_kb]
Publications: 
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /oracle_hint [erl2/ErlOracle]

Services: 
 * /cluedo_kb/get_loggers
 * /cluedo_kb/set_logger_level
 * /get_id
 * /mark_wrong_id
```

### the action_interface.cpp node  🪢

Concerning the `action_interface.cpp` node:

<p align="center">
<img src="https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/component_diagrams/v3/erl_assignment_2_action_interface_cpp.jpg" width= 500 height=500>
</p>

action_interface.cpp implements all rosplan actions in a single ROS node, moreover:

- the same node can be run replicated for all actions specified in the pddl
- topics and services only get allocated when the action is called for the first time via the rosplan action dispatcher
- the node interacts with the navigation and manipulation systems to move the robot and the arm
- The node also interacts with the KB and the oracle for clue and hypothesis processing operations

Regarding the pddl, it is possible to see their logical implementation within the domain file, inside the [detectibot_pddl][116] folder. There you can find both the predicates and seven actions, namely:

1. leave_temple
2. shift_gripper
3. gather_hint
4. go_to_wp
5. reach_temple
6. check_consistent_hypo
7. query_hypo

here below it is possible to see the conntent of the soultion found. If you want to take a look at the file itself, just [click here][117] 

``` Plain txt
; States evaluated: 54
; Cost: 14.013
; Time 0.00
0.000: (leave_temple tp wp1)  [1.000]
1.001: (shift_gripper wp1)  [1.000]
2.002: (gather_hint wp1)  [1.000]
3.003: (go_to_wp wp1 wp2)  [1.000]
4.004: (shift_gripper wp2)  [1.000]
5.005: (gather_hint wp2)  [1.000]
6.006: (go_to_wp wp2 wp3)  [1.000]
7.007: (shift_gripper wp3)  [1.000]
8.008: (gather_hint wp3)  [1.000]
9.009: (go_to_wp wp3 wp4)  [1.000]
10.010: (shift_gripper wp4)  [1.000]
11.011: (gather_hint wp4)  [1.000]
12.012: (reach_temple wp4 tp)  [1.000]
12.012: (check_consistent_hypo wp1)  [1.000]
13.013: (query_hypo tp)  [1.000]
```

Node interfaces:
```Plain txt
Node [/rpi_leave_temple]
Publications: 
 * /rosout                                         [rosgraph_msgs/Log]
 * /rosplan_knowledge_base/pddl_action_parameters  [rosplan_knowledge_msgs/DomainFormula]
 * /rosplan_plan_dispatcher/action_feedback        [rosplan_dispatch_msgs/ActionFeedback]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /rosplan_plan_dispatcher/action_dispatch [rosplan_dispatch_msgs/ActionDispatch]

Services: 
 * /rpi_leave_temple/get_loggers
 * /rpi_leave_temple/set_logger_level

--------------------------------------------------------------------------------
Node [/rpi_shift_gripper]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /rosplan_knowledge_base/pddl_action_parameters  [rosplan_knowledge_msgs/DomainFormula]
 * /rosplan_plan_dispatcher/action_feedback        [rosplan_dispatch_msgs/ActionFeedback]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /rosplan_plan_dispatcher/action_dispatch        [rosplan_dispatch_msgs/ActionDispatch]

Services: 
 * /rpi_shift_gripper/get_loggers
 * /rpi_shift_gripper/set_logger_level

--------------------------------------------------------------------------------
Node [/rpi_gather_hint]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /rosplan_knowledge_base/pddl_action_parameters  [rosplan_knowledge_msgs/DomainFormula]
 * /rosplan_plan_dispatcher/action_feedback        [rosplan_dispatch_msgs/ActionFeedback]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /rosplan_plan_dispatcher/action_dispatch        [rosplan_dispatch_msgs/ActionDispatch]

Services: 
 * /rpi_gather_hint/get_loggers
 * /rpi_gather_hint/set_logger_level

--------------------------------------------------------------------------------
Node [/rpi_go_to_wp]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /rosplan_knowledge_base/pddl_action_parameters [rosplan_knowledge_msgs/DomainFormula]
 * /rosplan_plan_dispatcher/action_feedback       [rosplan_dispatch_msgs/ActionFeedback]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /rosplan_plan_dispatcher/action_dispatch       [rosplan_dispatch_msgs/ActionDispatch]

Services: 
 * /rpi_go_to_wp/get_loggers
 * /rpi_go_to_wp/set_logger_level

--------------------------------------------------------------------------------
Node [/rpi_reach_temple]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /rosplan_knowledge_base/pddl_action_parameters [rosplan_knowledge_msgs/DomainFormula]
 * /rosplan_plan_dispatcher/action_feedback       [rosplan_dispatch_msgs/ActionFeedback]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /rosplan_plan_dispatcher/action_dispatch       [rosplan_dispatch_msgs/ActionDispatch]

Services: 
 * /rpi_reach_temple/get_loggers
 * /rpi_reach_temple/set_logger_level

--------------------------------------------------------------------------------
Node [/rpi_check_consistent_hypo]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /rosplan_knowledge_base/pddl_action_parameters  [rosplan_knowledge_msgs/DomainFormula]
 * /rosplan_plan_dispatcher/action_feedback        [rosplan_dispatch_msgs/ActionFeedback]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /rosplan_plan_dispatcher/action_dispatch        [rosplan_dispatch_msgs/ActionDispatch]

Services: 
 * /rpi_check_consistent_hypo/get_loggers
 * /rpi_check_consistent_hypo/set_logger_level

--------------------------------------------------------------------------------
Node [/rpi_query_hypo]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /rosplan_knowledge_base/pddl_action_parameters [rosplan_knowledge_msgs/DomainFormula]
 * /rosplan_plan_dispatcher/action_feedback       [rosplan_dispatch_msgs/ActionFeedback]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /rosplan_plan_dispatcher/action_dispatch       [rosplan_dispatch_msgs/ActionDispatch]

Services: 
 * /rpi_query_hypo/get_loggers
 * /rpi_query_hypo/set_logger_level

```

### the manipulation.cpp node 🪢

Concerning the `manipulation_cpp` node:

<p align="center">
<img src="https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/component_diagrams/v1/erl_assignment_2_manipulation_cpp.jpg" width= 500 height=500>
</p>

This node is simply devoted to control the Detectibot's manipulator by directly interacting with the MoveIt! framework


Node interfaces:
```Plain txt
Node [/manipulation]
Publications: 
 * /attached_collision_object [moveit_msgs/AttachedCollisionObject]
 * /execute_trajectory/cancel [actionlib_msgs/GoalID]
 * /execute_trajectory/goal [moveit_msgs/ExecuteTrajectoryActionGoal]
 * /move_group/cancel [actionlib_msgs/GoalID]
 * /move_group/goal [moveit_msgs/MoveGroupActionGoal]
 * /pickup/cancel [actionlib_msgs/GoalID]
 * /pickup/goal [moveit_msgs/PickupActionGoal]
 * /place/cancel [actionlib_msgs/GoalID]
 * /place/goal [moveit_msgs/PlaceActionGoal]
 * /rosout [rosgraph_msgs/Log]
 * /trajectory_execution_event [std_msgs/String]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /execute_trajectory/feedback [moveit_msgs/ExecuteTrajectoryActionFeedback]
 * /execute_trajectory/result [moveit_msgs/ExecuteTrajectoryActionResult]
 * /execute_trajectory/status [actionlib_msgs/GoalStatusArray]
 * /move_group/feedback [moveit_msgs/MoveGroupActionFeedback]
 * /move_group/result [moveit_msgs/MoveGroupActionResult]
 * /move_group/status [actionlib_msgs/GoalStatusArray]
 * /pickup/feedback [moveit_msgs/PickupActionFeedback]
 * /pickup/result [moveit_msgs/PickupActionResult]
 * /pickup/status [actionlib_msgs/GoalStatusArray]
 * /place/feedback [moveit_msgs/PlaceActionFeedback]
 * /place/result [moveit_msgs/PlaceActionResult]
 * /place/status [actionlib_msgs/GoalStatusArray]
 * /tf [tf2_msgs/TFMessage]
 * /tf_static [tf2_msgs/TFMessage]

Services: 
 * /manipulation
 * /manipulation/get_loggers
 * /manipulation/set_logger_level
```
### my_simulation.cpp node 🪢

<p align="center">
<img src="https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/component_diagrams/v1/erl2_my_simulation_cpp.jpg" width= 500 height=500>
</p>

This is the node provided by professor with some simplification in order to make the siumulation run faster and test wheter the detectibot would have carry out the investigation entirely. 

Node interfaces:
```Plain txt
Node [/my_simulation]
Publications: 
 * /oracle_hint [erl2/ErlOracle]
 * /rosout [rosgraph_msgs/Log]
 * /visualization_marker [visualization_msgs/MarkerArray]

Subscriptions: 
 * /gazebo/link_states [gazebo_msgs/LinkStates]

Services: 
 * /my_simulation/get_loggers
 * /my_simulation/set_logger_level
 * /oracle_solution

```



### rqt_graph

In the figure below, circles represent nodes and squares represent topic messages. The arrow instead, indicates the transmission of the message!

<img src= "https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/rqt/rosgraph_nodes_topics_all.png" />

### UML temporal diagram

> :warning: Due to the size of the original image, it is shown only a raw preview, opened with the standard pc's imahe viewer. If you want to download the original file, you can find it [here][111]

<img src= "https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/miscellaneous/raw_preview.png" />

### About detectiBot model 

In order to realise the robot model, some in-depth studies were carried out within the following repositories 

- [robot_modelling][8]
- [explab_2nd][9] 

In the [first repo][8], there are some notes aimed at gaining a brief and essential knowledge of robot modelling using xacro and urdf files. 
Some test examples are also shown, culminating in a potential robot model that can be deployed as part of the assignment. 

For reasons essentially related to computational cost, the model employed is much simpler and draws inspiration from the same prototype used during the course, which may be found and tested by downloading the [second repo][9]  

<a href="https://github.com/fedehub/explab_2nd">
  <p align="center">
    <img width="600" height="300" src="https://user-images.githubusercontent.com/61761835/189926720-e3ac2ce1-e352-497f-a7ad-3e5e9af35498.png">
  </p>
</a>

<a href="https://github.com/fedehub/robot_modelling">
  <p align="center">
    <img width="600" height="300" src="https://user-images.githubusercontent.com/61761835/189926873-89918134-8458-42e7-a1e4-ad1b32a813b4.png">
  </p>
</a>



## Working hypothesis and environment 

The  architecture  is designed for providing a raw simplification of the Cluedo Game. Hints are set a-priori and the True hipothesis is  randomly chosen before starting the game. 

Detectibot (the robot involved in the investigation), moves in a obstacle-free environment charachterised by a perfectly flat floor (without irregularities), within a square-shaped indoor environment 

Concerning the markers we can say that they are positioned in such  a way that the arm can reach them 

All choices were made with the aim of making the system as modular and flexible as possible. Despite this, certain limitations make the system quite unrealistic but functional.

### System's features

Most of them have been already discussed in the Software architecture’s section. 

The project implements the robot behaviour so that it can keep roaming around, looking for clues. This serves for solving the case. 

Indeed, while it navigates through the environment it tries to combine them in order to find a solution. This is where the reasoning & AI module, represented by the [cluedo_kb.py][20], comes imto play

Concerning the architecture, it is centralised and designed in such a way that individual components can be replaced as long as they meet the same required interface 

### System's limitations

Here below, some of the major system limitations are listed:

- the navigation module is not suitable for environments with obstacles as it needs to make a straight line between the starting point and the target.
- since there is no unit that deals explicitly with the marker topology, changing such a topology requires the modification of several parts of the architecture including:
  - the main node that takes into account the topology for being able to do replanning
  - the pddl models, in particular the problem file
  - the oracle, represented by the simulation.cpp node, having hard-coded markers, must be modified to support a new topology
- the architecture could also be executed in a distributed manner by dividing the components over various devices. However, this possibility was not considered during the design of the system. It is therefore necessary to identify possible criticalities in the communication protocol (i.e. to better manage service calls that fail based on the quality of the connection) and to treat them appropriately

### Possible technical Improvements

As for the system limitations, some of the most relevant potential techincal improvements:

- The current KB can be modified to implement the same functionalities on a different ontology system (i.e. Armor); the component can be extended for more accurate  hypotheses processing or for providing, for instance, a ontology backup feature
  
- The current navigation system is rather poor; it should be replaced with a more elaborate navigation system. In particular, the new navigation system should make it possible to achieve a certain orientation as well as a final position.
  
- The manipulation could be replaced with a more advanced node that performs a finer (more precise) control on moveit
  
- The current robot model is quite unstable. It should be adjusted so that it does not oscillate during its movments
  
- the robot needs a lot of manoeuvring space to move; There should be the need of seeking an appropriate navigation algorithm to reduce the necessary manoeuvring space


<!-- ROADMAP -->
## Roadmap

- [x] Complete the introduction of the template 
- [x] Describe the software architechture
  - [x] Component diagram (*not mandatory*)
  - [x] Temporal diagram + comments
  - [x] States diagrams, whether applicable + comments
  - [x] Create a list describing ROS messages and parameters 
- [x] Describe the installation steps and the running procedures
    - [x] Create a dedicated paragraph
    - [x] Include all the steps to display the robot's behaviour
- [x] Show in the "usage" section the running code
  - [x] Create a small video tutorial of the launch
  - [x] Create a small animated gif of the terminal while running code
- [x] Describe the Working hypothesis and environment
  - [x] Dedicated section for System's features
  - [x] Dedicated section for System's limitations
  - [x] Dedicated section for Possible technical improvements

 
    

See the [open issues](https://github.com/fedehub/ExperimentalRoboticsAssignment2/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- CONTRIBUTING -->
## Contributing

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement"

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under none License. 

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Federico Civetta - s4194543@studenti.unige.it

Project Link: [https://github.com/fedehub/ExperimentalRoboticsAssignment2](https://github.com/fedehub/ExperimentalRoboticsAssignment2)

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- ACKNOWLEDGMENTS -->
## List of resources

* [Viewing state machine](http://wiki.ros.org/smach/Tutorials/Smach%20Viewer)
* [Smach](http://wiki.ros.org/smach)
* [Armor](https://github.com/EmaroLab/armor)
* [Protègè](https://protege.stanford.edu)




<p align="right">(<a href="#top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->


[forks-shield]: 	https://img.shields.io/github/forks/fedehub/ExperimentalRoboticsAssignment2
[forks-url]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/network/members
[stars-shield]: 	https://img.shields.io/github/stars/fedehub/ExperimentalRoboticsAssignment2
[stars-url]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/stargazers
[issues-shield]: 	https://img.shields.io/github/issues/fedehub/ExperimentalRoboticsAssignment2
[issues-url]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/issues
[license-shield]: https://img.shields.io/github/license/fedehub/ExperimentalRoboticsAssignment2

<!-- general resources -->
[1]: http://wiki.ros.org/smach
[3]: http://wiki.ros.org/smach/Tutorials/Smach%20Viewer
[4]: http://wiki.ros.org
[5]: https://github.com/KCL-Planning/ROSPlan
[6]: https://moveit.ros.org/
[7]: http://wiki.ros.org/move_base
[8]: https://github.com/fedehub/robot_modelling
[9]: https://github.com/fedehub/explab_2nd
[10]: <Hint_Msg>
[11]: <BotMsg_Msg>
[12]: <param.yaml file>
[13]: <msgFolder>
[14]: <robotPose.msg>
[15]: armor_manipulation_client.py
[16]: https://ontogenesis.knowledgeblog.org/1260/
[17]: http://emarolab.github.io/armor_py_api/armor_utils_client.html
[18]: https://docs.python.org/3/library/queue.html
[19]: https://protege.stanford.edu

<!-- Nodes -->
[20]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/erl_assignment_2/scripts/cluedo_kb.py
[21]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/erl_assignment_2/scripts/go_to_point.py
[22]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/erl_assignment_2/scripts/main.py
[23]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/erl_assignment_2/scripts/test_nav.py
[24]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/erl_assignment_2/src/action_interface.cpp
[25]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/erl_assignment_2/src/manipulation.cpp
[26]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/erl2/src/my_simulation.cpp
[27]:https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/erl2/src/simulation.cpp


<!-- Launchers -->
[28]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/erl_assignment_2/launch/run_detectibot_actions.launch
[29]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/erl_assignment_2/launch/run_rosplan.launch
[30]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/erl_assignment_2/launch/run_simulated_actions.launch
[31]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/erl_assignment_2/launch/test_plan_and_sim_actions.sh

<!-- srvs -->
[32]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/erl_assignment_2_msgs/srv/GetId.srv
[33]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/erl_assignment_2_msgs/srv/MarkWrongId.srv
[34]:https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/erl2/srv/Oracle.srv


<!-- diagrams  -->
[100]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/component_diagrams/v3/component_diagram.jpg
[101]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/component_diagrams/v3/erl_assignment_2_action_interface_cpp.jpg
[102]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/component_diagrams/v1/erl2_my_simulation_cpp.jpg
[103]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/component_diagrams/v1/erl_assignment_2_cluedo_kb_py.jpg
[104]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/component_diagrams/v1/erl_assignment_2_main_py.jpg
[105]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/component_diagrams/v1/erl_assignment_2_manipulation_cpp.jpg
[106]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/component_diagrams/v1/erl_assignment_2_test_nav_py.jpg
[107]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/component_diagrams/v1/erl_assignment_go_to_point_py.jpg

<!-- rqt graphs -->
[108]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/tree/main/media/rqt/rosgraph_nodes_only.png
[109]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/tree/main/media/rqt/rosgraph_nodes_topics_active.png
[110]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/tree/main/media/rqt/rosgraph_nodes_topics_all.png

<!-- temporal diagram -->
[111]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/tree/main/media/temporal_diagrams/Diagrams_erl_img.jpg


<!-- MoveIt -->
[112]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/tree/main/erl_moveit_pkg/launch
[113]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/tree/main/erl_moveit_pkg/config

<!-- Previous project links -->
[114]: https://github.com/fedehub/expTest

<!-- pddl -->
[115]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/tree/main/erl_assignment_2/pddl
[116]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/tree/main/erl_assignment_2/pddl/detectibot_pddl
[117]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/erl_assignment_2/pddl/detectibot_pddl/solution.txt
