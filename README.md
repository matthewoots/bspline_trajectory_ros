# Bspline Trajectory Server with Optimization Wrapper for ROS(bspline-trajectory-ros)

## Installation
`bspline-trajectory-ros` serves a complete trajectory server on ROS using the `bs-traj-server` and acts as a wrapper to pass data into the module.

### Dependencies
- libbspline (https://github.com/matthewoots/libbspline.git) 
- LBFGSpp (https://github.com/matthewoots/LBFGSpp.git)
- bs-traj-server (https://github.com/matthewoots/bs-traj-server.git)

### Setup
For starters who do not know how to use ROS and do not have a prior workspace, just run the commands below and all will be fine
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/matthewoots/bspline_trajectory_ros.git --recurse-submodules
cd ..
catkin build
```