This package contains MATLAB and c++ code (packaged in a ROS node for KUKA LWR and in Yarp for simulations on iCub) for the project on:

## "Learning Dynamical Systems With Bifurcations"



Please refer to the PDF in paper/DS_formulation.pdf for the formulation of the dynamical system and the optimization.

To view an example and test the MATLAB algorithm, you can use the livescript "demo.mlx" in the MATLAB folder.

To test the dynamical system with learned parameters in MATLAB, run the code "plotDynamics.m" in the MATLAB/control_parameters folder.



To use the methods yourself, please consider:

## DATA:

Data has size [M x dimensions N] and is stored in .csv files.

By default, the format in the file is:

[x_1, x_2, ..., x_N, time]

If the .csv has other formats:

[x_1,x_2,...,x_N,time] -- N (dimensions) should be provided to loadData function

[time, x_1, x_2, ..., x_N] -- use flag timeAtBeginning=1 when loading data

[time,x_1,x_2,...,x_N] -- both N and timeAtBeginning=1 should be provided



## PARAMETERS:

Refer to paper/DS_formulation.pdf for a mathematical explanation of the parameters.

rho0 -- radius; size 1

M -- mass (influences speed and how far influence of limit set is felt); size 1

R -- angular speed and direction; size 1

a -- scaling; size N (# dimensions)

x0 -- origin shift; size N (# dimensions)

Rrot -- rotation matrix (in Roll-Pitch-Yaw order for N=3); size NxN



## DEPENDENCIES:

The matlab code depends on toolboxes provided, which should be added to the path:

- drtoolbox

- GMR-dynamics-v2.0

- DMP (for evaluation comparison)

Copyright information in MATLAB/README.md.

