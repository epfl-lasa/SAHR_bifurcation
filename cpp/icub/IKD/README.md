# Project description
This project contains the model of Coman in sdfast, and the inverse kinematics algorithm for the lower body

# Pre installation
You need to have eigen and cmake:

	sudo apt-get install libeigen3-dev
	sudo apt-get install cmake

# Test the functionality
navigate to IKD folder, then:

	mkdir build
	cd build
	cmake ..
	make -j8
	./test

You should see some vectors printed out. Just read the code to see what they mean.

# Understanding the code
You're recommended to read the function "check_consistency" in codes/src/Model.cpp to understand how to work with sdfast.
In the same code, "get_massmat" and "get_frcmat" return the mass matrix and gravitational, corriolis and ... forces respectively.
Remeber that these functions only need "set_state". To get Cartesian accelerations, you need to call "set_acc" too.
The description of state vector (with size AIR_N_U+1) and its derivative (with size AIR_N_U) as well as body indices could be found in /sdfast/include/Description.hpp

