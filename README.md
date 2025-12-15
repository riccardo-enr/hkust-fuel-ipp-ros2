# FUEL - ROS 2 Port

__Status:__ This is a ROS 2 port of the original FUEL project. Migration is in progress.

__Original Project:__ [HKUST-Aerial-Robotics/FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL)

__News:__

- Dec 9, 2025: ROS 2 migration started
- Feb 24, 2023: the code for **multi-UAV exploration** is released! check [this link](https://github.com/SYSU-STAR/RACER).
- Aug 24, 2021: The CPU-based simulation is released, CUDA is no longer required. Richer exploration environments are provided.

**FUEL** is a framework for **F**ast **U**AV **E**xp**L**oration.

Our method is demonstrated to complete challenging exploration tasks **3-8 times** faster than state-of-the-art approaches at the time of publication.
Central to it is a Frontier Information Structure (FIS), which maintains crucial information for exploration planning incrementally along with the online built map. Based on the FIS, a hierarchical planner plans frontier coverage paths, refine local viewpoints, and generates minimum-time trajectories in sequence to explore unknown environment agilely and safely. Try [Quick Start](#quick-start) to run a demo in a few minutes!  

<p align="center">
  <img src="files/1.gif" width = "400" height = "225"/>
  <img src="files/2.gif" width = "400" height = "225"/>
  <img src="files/3.gif" width = "400" height = "225"/>
  <img src="files/4.gif" width = "400" height = "225"/>
  <!-- <img src="files/icra20_1.gif" width = "320" height = "180"/> -->
</p>


Recently, we further develop a fully decentralized approach for exploration tasks using a fleet of quadrotors. The quadrotor team operates with asynchronous and limited communication, and does not require any central control. The coverage paths and workload allocations of the team are optimized and balanced in order to fully realize the system's potential. The associated paper has been published in IEEE TRO. Check code [here](https://github.com/SYSU-STAR/RACER).

<p align="center">
  <img src="files/racer1.gif" width = "400" height = "225"/>
  <img src="files/racer2.gif" width = "400" height = "225"/>
</p>

__Complete videos__: [video1](https://www.youtube.com/watch?v=_dGgZUrWk-8), [video2](https://www.bilibili.com/video/BV1yf4y1P7Vj).

__Authors__: [Boyu Zhou](http://sysu-star.com) from SYSU and [Shaojie Shen](http://uav.ust.hk/group/) from the [HUKST Aerial Robotics Group](http://uav.ust.hk/).

Please cite our paper if you use this project in your research:
- [__FUEL: Fast UAV Exploration using Incremental Frontier Structure and Hierarchical Planning__](https://arxiv.org/abs/2010.11561), Boyu Zhou, Yichen Zhang, Xinyi Chen, Shaojie Shen, IEEE Robotics and Automation Letters (**RA-L**) with ICRA 2021 option

```
@article{zhou2021fuel,
  title={FUEL: Fast UAV Exploration Using Incremental Frontier Structure and Hierarchical Planning},
  author={Zhou, Boyu and Zhang, Yichen and Chen, Xinyi and Shen, Shaojie},
  journal={IEEE Robotics and Automation Letters},
  volume={6},
  number={2},
  pages={779--786},
  year={2021},
  publisher={IEEE}
}
```

Please kindly star :star: this project if it helps you. We take great efforts to develope and maintain it :grin::grin:.

## Table of Contents

- [FUEL](#fuel)
  - [Table of Contents](#table-of-contents)
  - [Quick Start](#quick-start)
  - [Exploring Different Environments](#exploring-different-environments)
  - [Creating a _.pcd_ Environment](#creating-a-pcd-environment)
  - [Acknowledgements](#acknowledgements)

## Quick Start

**Note: This repository is being ported to ROS 2.** The migration is in progress. Some features may not work correctly until the migration is complete.

This project is being migrated from ROS 1 to ROS 2. The original version was tested on Ubuntu 18.04 (ROS Melodic) and 20.04 (ROS Noetic). The ROS 2 version targets Ubuntu 22.04 (ROS 2 Humble) or Ubuntu 24.04 (ROS 2 Jazzy).

### Dependencies

Firstly, you should install __nlopt v2.7.1__:
```bash
git clone -b v2.7.1 https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install
```

Next, you can run the following commands to install other required tools:
```bash
sudo apt-get install libarmadillo-dev
```

### Building with ROS 2

Then simply clone and compile our package:

```bash
cd ${YOUR_WORKSPACE_PATH}/src
git clone git@github.com:riccardo-enr/hkust-fuel-ipp-ros2.git
cd ../
colcon build
```

**Note:** The build instructions are for ROS 2. For the original ROS 1 version, please use the main FUEL repository.

### Running (Work in Progress)

After compilation you can start a sample exploration demo. The launch files are being converted to ROS 2 format.

<!-- Original ROS 1 instructions:
Firstly run ```Rviz``` for visualization: 
```
source devel/setup.bash && roslaunch exploration_manager rviz.launch
```
then run the simulation (run in a new terminals): 
```
source devel/setup.bash && roslaunch exploration_manager exploration.launch
```
-->

By default you can see an office-like environment. Trigger the quadrotor to start exploration by the ```2D Nav Goal``` tool in ```Rviz```. A sample is shown below, where unexplored structures are shown in grey and explored ones are shown in colorful voxels. The FoV and trajectories of the quadrotor are also displayed.

 <p id="demo1" align="center">
  <img src="files/office.gif" width = "600" height = "325"/>
 </p>


## Exploring Different Environments

The exploration environments in our simulator are represented by [.pcd files](https://pointclouds.org/documentation/tutorials/pcd_file_format.html).
We provide several sample environments, which can be selected in [simulator.xml](fuel_planner/exploration_manager/launch/simulator.xml):


```xml
  <!-- Change office.pcd to specify the exploration environment -->
  <!-- We provide office.pcd, office2.pcd, office3.pcd and pillar.pcd in this repo -->
  <node pkg ="map_generator" name ="map_pub" type ="map_pub" output = "screen" args="$(find map_generator)/resource/office.pcd"/>    
```

Other examples are listed below.

_office2.pcd_:
<p id="demo2" align="center">
<img src="files/office2.gif" width = "600" height = "325"/>
</p>

_office3.pcd_:
<p id="demo3" align="center">
<img src="files/office3.gif" width = "600" height = "325"/>
</p>

_pillar.pcd_:
<p id="demo4" align="center">
<img src="files/pillar.gif" width = "320" height = "325"/>
</p>

If you want to use your own environments, simply place the .pcd files in [map_generator/resource](uav_simulator/map_generator/resource), and follow the comments above to specify it.
You may also need to change the bounding box of explored space in [exploration.launch](https://github.com/HKUST-Aerial-Robotics/FUEL/blob/main/fuel_planner/exploration_manager/launch/exploration.launch):

```xml
    <arg name="box_min_x" value="-10.0"/>
    <arg name="box_min_y" value="-15.0"/>
    <arg name="box_min_z" value=" 0.0"/>
    <arg name="box_max_x" value="10.0"/>
    <arg name="box_max_y" value="15.0"/>
    <arg name="box_max_z" value=" 2.0"/>
```

To create your own .pcd environments easily, check the [next section](#creating-a-pcd-environment).

## Creating a _.pcd_ Environment

We provide a simple tool to create .pcd environments.
First, run:

```
  rosrun map_generator click_map
```

Then in ```Rviz```, use the ```2D Nav Goal``` tool (shortcut G) to create your map. Two consecutively clicked points form a wall.
An example is illustrated:

<p id="demo5" align="center">
<img src="files/create_map.gif" width = "600" height = "340"/>
</p>

After you've finished, run the following node to save the map in another terminal:

```
  rosrun map_generator map_recorder ~/
```

Normally, a file named __tmp.pcd__ will be saved at ```~/```. You may replace ```~/``` with any locations you want.
Lastly, you can use this file for exploration, as mentioned [here](#exploring-different-environments).

## Acknowledgements
  We use **NLopt** for non-linear optimization and use **LKH** for travelling salesman problem.
