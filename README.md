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
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#software-architechture">Software architechture</a></li>
        <li><a href="#ros-node-description-an-overview">ROS node description: an Overview</a></li>
        <li><a href="#ros-node-description-the-go_to_point.py-node">ROS node description: the go_to_point.py node</a></li>
        <li><a href="#ros-node-description-the-main.py-node ">ROS node description:-the main.py node</a></li>
        <li><a href="#ros-node-description-the-cluedo_kb.py-node ">ROS node description: the cluedo_kb.py node</a></li>
        <li><a href="#ros-node-description-the-action_interface.cpp-node">ROS node description: he action_interface.cpp node</a></li>
        <li><a href="#ros-node-description-the-manipulation.cpp-node ">ROS node description: the manipulation.cpp node</a></li>
        <li><a href="#ros-node-description-my_simulation.cpp-node ">ROS node description:my_simulation.cpp node </a></li>
          <ul>
            <li><a href=#rossrv>rossrv</a></li>
            <li><a href=#rosmsg>rosmsg</a></li>
            <li><a href=#rostopic>rostopic</a></li>
            <li><a href=#rosparam>rosparam</a></li>
          </ul>
        <li><a href="#rqt_graph">rqt_graph</a></li>
        <li><a href="#installation">Installation</a></li>
        <li><a href="#running procedure">Running procedure</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li>
      <a href="#Working-hypothesis-and-environment">Working hypothesis and environment</a>
      <ul>
        <li><a href="#System's features">System's features</a></li>
        <li><a href="#System's limitations">System's limitations</a></li>
        <li><a href="#Possible technical Improvements">Possible technical Improvements</a></li>
      </ul>
    </li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
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

### Software architechture

<!-- how to make a ref to a specific section 
the **rqt_graph** <a href="#rqt_graph">section</a>
-->
<img src="https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/component_diagrams/v1/component_diagram.jpg" >



<!-- Software architecture, temporal diagram and states diagrams (if applicable). Each diagram should be commented with a paragraph, plus a list describing ROS messages and parameters. -->

### ROS node description: An overview 

Within the scripts folder, inside the `erl_assignment_2` pkg the following nodes are listed:  

- [cluedo_kb.py][20]              <!-- PLEASE INSERT HERE -->

- [go_to_point.py][21]            <!-- PLEASE INSERT HERE -->

- [main.py][22]                   <!-- PLEASE INSERT HERE -->

- [test_nav.py][23]               <!-- PLEASE INSERT HERE -->

In the src folder instead, there are the here listed cpp nodes:

- [action_interface.cpp][20]      <!-- PLEASE INSERT HERE -->

- [manipulation.cpp][21]          <!-- PLEASE INSERT HERE -->


Concerning the node we were provided, it belongs to the `erl2` package:

- [my_simulation.cpp][26]

> :warning: For testing purposes two nodes have been employed. The first one is the my_simulation.cpp and the second one is the test_nav.py. Be aware that the test_nav.py does not represent any relevant part of the software architechture, its purpose was just that of verifying the robustness of the navigation module. Instead, my_simulation.cpp belongs to the official SOFAR and it is needed for the project to correctly work

### ROS node description: the go_to_point.py node  🪢

Let's start with the `go_to_point.py` node

<p align="center">
<img src="https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/component_diagrams/v1/erl_assignment_go_to_point_py.jpg" width= 500 height=500>
</p>

It implements a ROS service, whose purpose is that of piloting the robot toward a specific target by following a straight line. As it is shown by the component diagram here reported, it subscribes to the `/odom` topic for retrieving the current robot position and once the robot orientation among x and y coordinates has been computed with respect to the target position (obtained by means of the ros parameter server), it publishes on the `/cmd_vel` topic

### ROS node description: the main.py node 🪢

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

### ROS node description: the cluedo_kb.py node 🪢

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


### ROS node description: the action_interface.cpp node  🪢

Concerning the `action_interface.cpp` node:

<p align="center">
<img src="https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/component_diagrams/v2/erl_assignment_2_action_interface_cpp_v2.jpg" width= 500 height=500>
</p>

action_interface.cpp implements all rosplan actions in a single ROS node, moreover:

- the same node can be run replicated for all actions specified in the pddl (please)
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

here below it is possible to see the conntent of the soultion found. If you wamt to take a look at the file itself, just [click here][117] 

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

### ROS node description: the manipulation.cpp node 🪢

Concerning the `manipulation_cpp` node:

<p align="center">
<img src="https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/component_diagrams/v1/erl_assignment_2_manipulation_cpp.jpg" width= 500 height=500>
</p>

This node is simply devoted to control the Detectibot's manipulator by directly interacting with the MoveIt! framework


### ROS node description: my_simulation.cpp node 🪢

<p align="center">
<img src="https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/component_diagrams/v1/erl2_my_simulation_cpp.jpg" width= 500 height=500>
</p>

This is the node provided by professor with some simplification in orderr to make the siumulation run faster and test wheter the detectibot would have carry out the investigation entirely. 

### rossrv 

- **/get_id**: <!-- rossrv description -->

    
  > msg type:  `erl_assignment_2_msgs/GetId`
  
- **/mark_wrong_id**: <!-- rossrv description -->

  > msg type:  `erl_assignment_2_msgs/MarkWrongId`

- **/oracle_solution**: <!-- rossrv description -->

  > msg type    `erl2/Oracle`       


### rosmsg

<!-- PLEASE INSERT HERE -->
Within the `erl2` package, a cusom message is defined. For the sake of completeness, its structure is here below mentioned

```plain txt

int32 ID
string key
string value

```

### rostopic

The `/oracle_hint` topic shows, as::
- **Publishers:**
  - [/cluedo_kb][20]
  
- **Subscribers:**
  - [/my_simulation][26]

The `/cmd_vel` topic shows, as:
- **Publishers:**
  - [/gazebo][6] 
  
- **Subscribers:**
  - [/go_to_point][21]
 
The `/odom` topic shows, as:
- **Publishers:**
  - [/go_to_point][21]

- **Subscribers:**
  - /gadzebo
 
The `/tf` topic shows, as:
- **Publishers:**
  - [/manipulation][25]
  - /rviz
  - /move_group
- **Subscribers:**
  - /robot_state_publisher
  - /gazebo
 
The `/move_group` topic shows, as:
- **Publishers:**
  - [/manipulation][7]
  - /rviz
  - /gazebo 
- **Subscribers:**
  - /manipulation
  - /rviz
  - /gazebo


### rosparameters 

`des_pos_x` and `des_pos_y` are two parameters used for keeping track  of the target goal to be assigned to the robot. 


### rqt_graph
The *rqt_graph* is a tool that shows the correlation among active nodes and messages being transmitted on the ROS network as a diagram. After executing all nodes (or after launching the roslaunch file), launch the **rqt_graph** through 

```
rosrun rqt_graph rqt_graph
```

In the figure below, circles represent nodes and squares represent topic messages. The arrow instead, indicates the transmission of the message!

<img src= "https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/rqt/rosgraph_nodes_topics_all.png" />


<!-- PLEASE INSERT HERE -->


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

## Working hypothesis and environment 

The  architecture  is designed for providing a raw simplification of the Cluedo Game. Hints are set a-priori, as well as true hypothesis that leads to the game’s end, once collected. 

All choices were made with the aim of making the system as modular and flexible as possible. Despite this, certain limitations make the system quite unrealistic but functional.

### System's features

Most of them have been already discussed in the Software architecture’s section. The most relevant one is that of having a robot capable of exploiting an ontology to solve  a problem.

### System's limitations

type here ...

### Possible technical Improvements

type here ...

<!-- ROADMAP -->
## Roadmap

- [x] Complete the introduction of the template 
- [ ] Describe the software architechture
  - [x] Component diagram (*not mandatory*)
  - [x] Temporal diagram + comments
  - [ ] States diagrams, whether applicable + comments
  - [ ] Create a list describing ROS messages and parameters 
- [ ] Describe the installation steps and the running procedures
    - [ ] Create a dedicated paragraph
    - [ ] Include all the steps to display the robot's behaviour
- [ ] Show in the "usage" section the running code
  - [ ] Create a small video tutorial of the launch
  - [ ] Create a small animated gif of the terminal while running code
- [ ] Describe the Working hypothesis and environment
  - [ ] Dedicated section for System's features
  - [ ] Dedicated section for System's limitations
  - [ ] Dedicated section for Possible technical improvements

 
    

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
[8]: <oracle>
[9]: <HypoID_Msg>
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


<!-- Component diagrams  -->
[100]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/component_diagrams/v1/component_diagram.jpg
[101]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/media/component_diagrams/v2/erl_assignment_2_action_interface_cpp_v2.jpg
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

<!-- state diagram -->
[111]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/tree/main/media/state_diagrams/Diagrams_erl_img.jpg


<!-- MoveIt -->
[112]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/tree/main/erl_moveit_pkg/launch
[113]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/tree/main/erl_moveit_pkg/config

<!-- Previous project links -->
[114]: https://github.com/fedehub/expTest

<!-- pddl -->
[115]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/tree/main/erl_assignment_2/pddl
[116]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/tree/main/erl_assignment_2/pddl/detectibot_pddl
[117]: https://github.com/fedehub/ExperimentalRoboticsAssignment2/blob/main/erl_assignment_2/pddl/detectibot_pddl/solution.txt
