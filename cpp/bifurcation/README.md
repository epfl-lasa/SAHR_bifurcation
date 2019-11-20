# bifurcation

ROS node to use the bifurcation DS model.

### Installation

* Clone the repository:
```
$ git clone https://github.com/epfl-lasa/SAHR_bifurcation.git
```
* Copy the folder cpp/bifurcation in your catkin src directory:
```
$ cp -R SAHR_bifurcation/cpp/bifurcation path/to/catkin/src/
```
* wstool gets all other git repository dependencies, after the following steps you should see extra catkin 
  packages in your src directory.
```
$ wstool init
$ wstool merge bifurcation/dependencies.rosinstall 
$ wstool up 
```
* Query and installs all libraries and packages 
```
$ rosdep install --from-paths . --ignore-src --rosdistro kinetic 
```

* Install [**Gazebo**](http://gazebosim.org/), follow this [**link**](http://gazebosim.org/tutorials?tut=install_ubuntu&) for 
instructions on how to install it on ubuntu. Make sure that the ros libraries of Gazebo are also installed:
```
$ sudo apt-get install ros-indigo-gazeboX-*
```

### Simulation
Download the [**kuka-lwr-ros-examples**](https://github.com/epfl-lasa/kuka-lwr-ros-examples) into your catkin_ws and 
after compling them are ready to run the lwr_simple_example. 

Open a new terminal and run the following:
```sh
$ roslaunch lwr_simple_example sim.launch
```
This will run the simulator and the Gazebo simulator and ROS Rviz visualiser GUIs should both open. If the Gazebo 
window does not open this is because a flag is set in the sim.launch file.


### Using the DS

To test the model run the simulation (above) and run in a terminal:
```
$ rosrun bifurcation
```

**You can provide the initial value of parameters as additional command line arguments or by changing the file cfg/parametersDyn.cfg.** The function saveToCfg.m in MATLAB allows to create the configuration file from a parameters struct.

You can also change the parameters at runtime by running:
```
$ rosrun rqt_reconfigure rqt_reconfigure
```

To use the model with the mocap system, run in a terminal:
```
$ roslaunch bifurcation mocap.launch
```
and in another terminal:
```
$ rosrun bifurcationMocap
```
