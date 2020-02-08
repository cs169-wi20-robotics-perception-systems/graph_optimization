# Programming Assignment 3: Graph Optimization

Simple implementation of using g2o graph optimization with the Husarion ROSBot 2.0.

## Getting Started

This package has been tested on Ubuntu 16.04 and on the Husarion ROSBot 2.0.

Set up your catkin workspace:
```
mkdir -p catkin_ws/src
cd catkin_ws/src
```

## Dependencies

Instructions are based on the Programming Assignment 3 description.

* cython
* libqglviewer-dev
* libqglviewer2
* libsuitesparse-dev
* python-dev

```
sudo apt install cython libqglviewer-dev libqglviewer2 libsuitesparse-dev python-dev
```

* [g2opy](https://github.com/uoip/g2opy.git)

```
git clone https://github.com/uoip/g2opy.git
```

Modify 'g2opy' 'CMakeLists.txt', such that 'line 280' should be:
```
INCLUDE_DIRECTORIES(${g2o_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} /usr/include/python2.7)
```

Then in the 'g2opy' directory:
```
mkdir build
cd build
cmake -DPYBIND11_PYTHON_VERSION=2.7 -DG2O_BUILD_APPS=true -DBUILD_SHARED_LIBS=true -DG2O_BUILD_EXAMPLES=true ..
make -j1
sudo make install
cd ..
sudo ln -s <path to g2opy>/lib/g2o.so /usr/lib/python2.7/dist-packages/
```

## Install

Clone the repository. Then build and source:
```
git clone https://github.com/cs169-wi20-robotics-perception-systems/graph_optimization.git
cd ..
catkin_make
source devel/setup.bash
```

Check if the `simple_graph_optimization.py` file in `scripts` folder are executable. If not, run:
```
chmod +x simple_graph_optimization.py
```

## Run

To run the simple g2o graph optimization:
```
roslaunch graph_optimization graph_optimization.launch
```

Some parameters that can be set:
* 'scan_covariance' = \<noise from laser; Default: 4.5\>
* 'pose_covariance' = \<noise from wheel odometry; Default: 5.5\>
* 'scan_angle_left' = \<left most angle in radians; Default: pi / 12\>
* 'scan_angle_right' = \<right most angle in radians; Default: 11 pi / 12\>
* 'ground_truth_end' = \<true final distance the robot has moved; Default: 0.97\>
* 'initial_pose' = \<True/False; If true, publish initial pose via 'initialpose' ROS topic. Else, default: 0.36\>
