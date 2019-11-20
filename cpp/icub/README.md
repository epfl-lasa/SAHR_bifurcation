# Walking
Algorithms for bifurcating DS with iCub. Original algorithm by Salman Faraji, EPFL. Modified by Ilaria Lauzana to include the DS with bifurcation from the paper "Learning Dynamical Systems with Bifurcations".

-- Get yarp from here. https://www.yarp.it/install_yarp_linux.html#install_on_linux ,
If using ubuntu 16, get the xenial version in their repository. Make sure you enable the plugins if compiling it from the source.

-- Go to gazebo-yarp-plugins folder and install it. Compared to the official package, this copy has two additional plugins that read robot and object positions.

```bash
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local/
sudo make install
```

-- Put this path in ~/.bashrc:

```bash
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/usr/local/lib
```

-- Also put the path of gazebo/objects and gazebo/robots in ~/.bashrc. For example:

```bash
if [ -z "$GAZEBO_MODEL_PATH" ]; then
    export GAZEBO_MODEL_PATH=/home/sfaraji/Dropbox/LASA/gazebo/objects:/home/sfaraji/Dropbox/LASA/gazebo/robots
else
    export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/home/sfaraji/Dropbox/LASA/gazebo/objects:/home/sfaraji/Dropbox/LASA/gazebo/robots
fi
```

-- In one command window run:

```bash
yarpserver
```

-- Open another window, go to gazebo/worlds folder, and run:

```bash
gazebo -s libgazebo_yarp_clock.so single_leg.world
```

-- Open another window, go to controllers/single_leg_bif/ folder, and compile the code:

```bash
mkdir build
cd build
cmake ..
make -j8
```

-- Run the controller with DS's parameters from params.txt by:

```bash
./main --robot icubSim5
```

-- To run the original icub controller:

```bash
./main --robot icubSim5 0
```
