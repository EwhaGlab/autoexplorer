# Autoexplorer
**Autoexplorer** package is a 2D mobile robot's exploration system based on frontier region detection.
This package contains C++ implementation of our IROS22 paper: [Autoexplorer: Autonomous Exploration of Unknown Environments using Fast Frontier-Region Detection and Parallel Path Planning](http://graphics.ewha.ac.kr/autoexplorer/)
Please refer to the paper if you have inquiries regarding the technical aspect of this package.

## Dependencies

**Autoexplorer** is a ROS-compatible package. Therefore, we expect you to have Ubuntu 18.04 installed along with ROS Melodic.
The package should work fine in Ubuntu 20.04 with ROS Noetic.

You need the ROS navigation stack to control an embodied agent. 
Autoexplorer runs best with our customized version of [navigation stack](https://github.com/han-kyung-min/navigation).  

If you want to run this package in a synthetic environment, such as the Gazebo simulator. We recommend you install a mapping SW such as
[OctoMap](https://github.com/OctoMap/octomap). Use [Turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) packages to explore your favorite world. 
Besides, we found [TEB](https://github.com/rst-tu-dortmund/teb_local_planner) local planner runs OK with this package, so you might want to consider having this local planner.
In the case of solving real-world exploration problems with a mobile robot, you will need a SLAM SW to produce a 2D occupancy grid map. 
We recommend installing [SLAM toolbox](https://github.com/SteveMacenski/slam_toolbox) for your localization and mapping.

## To install
```
cd ~/catkin_ws/src
git clone https://github.com/EwhaGlab/autoexplorer.git
cd ~/catkin_ws
catkin_make install
```


## Example 1: Autoexplorer in Willowgarage Gazebo world

```
roslaunch autoexplorer willowgarage.launch
roslaunch autoexplorer explorer_bench.launch
roslaunch autoexplorer autoexplorer.launch
```

##Citation
Good luck with your projects! Please cite our paper if you think **autoexplorer** is helpful for your research work.

```
K.M. Han and Y.J. Kim, "Autoexplorer: Autonomous Exploration of Unknown Environments using Fast Frontier-Region Detection and Parallel Path Planning," 
2022 International Conference on Intelligent Robots and Systems (IROS), 2022
```
