# Autoexplorer
**Autoexplorer** package is a 2D mobile robot's exploration system based on frontier region detection.
This package contains C++ implementation of our IROS22 paper: [Autoexplorer: Autonomous Exploration of Unknown Environments using Fast Frontier-Region Detection and Parallel Path Planning](http://graphics.ewha.ac.kr/autoexplorer/)
Please refer to the paper if you have inquiries regarding the technical aspect of this package.

## Dependencies

**Autoexplorer** is a ROS-compatible package. Therefore, we expect you to have Ubuntu 18.04 installed along with ROS Melodic.
The package should work fine in Ubuntu 20.04 with ROS Noetic.

You need the ROS navigation stack to control an embodied agent. 
Autoexplorer runs best with our customized version of [navigation stack](https://github.com/han-kyung-min/navigation).  

If you want to run this package in a synthetic environment, such as the Gazebo simulator, we recommend you install a mapping SW such as
OctoMap. Use [Turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) packages to explore your favorite world. 
Besides, we found [TEB](https://github.com/rst-tu-dortmund/teb_local_planner) local planner runs OK with this package, so you might want to consider having this local planner.
In the case of solving real-world exploration problems with a mobile robot, you will need a SLAM SW to produce a 2D occupancy grid map. 
We recommend installing [SLAM toolbox](https://github.com/SteveMacenski/slam_toolbox) for your localization and mapping.

## To install

### Follow the steps below to install autoexplorer and its dependencies

> #### (1) Install Octomap

```
  cd ~/catkin_ws/src
  git clone https://github.com/han-kyung-min/octomap_mapping.git
  cd octomap_mapping
  git checkout explore_bench-nn_burger-fast_gridmap_pub
  sudo apt-get install ros-<ros_ver>-octomap*
  sudo apt-get install ros-<ros_ver>-octomap-server
  cd ~/catkin_ws
  catkin_make install
```
> #### (2) Install Turtlebot3 package
```
  sudo apt-get install ros-<ros_ver>-turtlebot3
```
> #### (3) Install navigation stack
```
  cd ~/catkin_ws/src
  git clone https://github.com/han-kyung-min/navigation.git
  cd navigation
  git checkout proximity_check
  cd ~/catkin_ws
  catkin_make install
```
> #### (4) Install teb_local_planner
```
  cd ~/catkin_ws/src
  git clone https://github.com/rst-tu-dortmund/teb_local_planner
  cd teb_local_planner
  git checkout <your_ros_version_branch>
  cd ~/catkin_ws
  catkin_make -DCATKIN_WHITELIST_PACKAGES="teb_local_planner"
```
> #### (5) Clone and install autoexplorer

```
cd ~/catkin_ws/src
git clone https://github.com/EwhaGlab/autoexplorer.git
cd ~/catkin_ws
catkin_make install
```
### Additionally, you need to install [SLAM toolbox](https://github.com/SteveMacenski/slam_toolbox) if your application is driving a real robot 


## Example 1: Autoexplorer in Willowgarage Gazebo world


### Start the exploration task
Don't forget to "source ~/catkin_ws/install/setup.bash" before starting the launch files below

```
roslaunch autoexplorer willowgarage.launch
roslaunch autoexplorer explorer_bench.launch
roslaunch autoexplorer autoexplorer.launch
```

## Bibtex
Good luck with your projects! Please cite our paper if you think **autoexplorer** is helpful for your research work.

```
@INPROCEEDINGS{HanKim_iros22,
  author={Han, Kyung Min and Kim, Young J.},
  booktitle={2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={Autoexplorer: Autonomous Exploration of Unknown Environments using Fast Frontier-Region Detection and Parallel Path Planning}, 
  year={2022},
  volume={},
  number={},
  pages={10536-10541},
  keywords={Buildings;Filtering algorithms;Manipulators;Path planning;Planning;Mobile robots;Detection algorithms},
  doi={10.1109/IROS47612.2022.9981263}}
```
Feel free to send us an email if you are having a trouble with compiling this package.
