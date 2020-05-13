# Checkpointing and roll-forward recovery for Hierarchical Control CPS
ROS packages for numerical simulation of checkpointing and roll-forward recovery of state estimates in differential drive mobile robot (and DC motor) system under sensor attacks.

Detailed documentation can be found at:- https://docs.google.com/document/d/1_nz2afkh-SWOh0j6m1g7FoTotDnMQovMn6eUlM3usv8/edit?usp=sharing

## Update Notes
* system descriptions python files for each of inner and outer loops created
* safe_auto_nonlinear_ddrive of outer loop inherits from safe_auto_nonlinear_base
* In this version of SRSC framework, state is predicted for first time step with sensor attack by rolling-forward all the way from the ckpt and for every subsequent attacked time step with saved previously predicted value
* modified main files of all three loop nodes such that EKF estimate stored in xe_o (_o stands for original), roll-forward in xe and GT in x (previous versions without this may be wrong)
* Added error_analysis (including option to turn on and off in system description files)
* Moved advancing of simulation to node files in in, in_2 and out nodes. Removed ye to reduce redundant variables

## Preliminaries
Install ROS (http://wiki.ros.org/kinetic/Installation/Ubuntu)

## Installation instructions
Python package prerequisites
```
pip install scipy
```
```
pip install matplotlib
```
```
pip install numpy
```
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
Make all python nodes executable
```
cd ~/catkin_ws/src/coordinator/src/
chmod +x coordinator_node.py
cd 
cd ~/catkin_ws/src/out_loop/src/
chmod +x out_loop_node.py
cd 
cd ~/catkin_ws/src/in_loop/src/
chmod +x in_loop_node.py
cd 
cd ~/catkin_ws/src/in_loop_2/src/
chmod +x in_loop_2_node.py
cd 
cd ~/catkin_ws/src/rf_coordinator/src/
chmod +x rf_coordinator_node.py
```
## Instructions to run simulation
Open two terminals and in each, source build file,

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

