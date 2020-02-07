# Checkpointing and roll-forward recovery for Hierarchical Distributed Control CPS
ROS packages for numerical simulation of checkpointing and roll-forward recovery of state estimates in differential drive mobile robot (and DC motor) system under sensor attacks.

## Installation instructions
Run the following
```
mkdir ~/catkin_ws/
```
```
cd ~/catkin_ws/
```
```
git clone https://github.com/kaustubhsridhar/checkpointing_and_recovery.git
```
```
catkin_make
```
## Instructions to run simulation
Open two terminals and in each source build file

```
source ~/catkin_ws/devel/setup.bash
```
Then, in first terminal,
```
roscore
```
In second terminal,
```
roslaunch coordinator overall.launch
```


More documentation can be found at:- https://drive.google.com/open?id=1AMkvtVj9qy1y-yGgW7vhpE7YUGtaTZIl1O7uvYSZUp8
