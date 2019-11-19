ROS node to use the bifurcation DS model.

To test the model, use:

	rosrun bifurcation

You can provide the initial value of parameters by changing the file cfg/parametersDyn.cfg. You can also change the parameters at runtime by running:

	rosrun rqt_reconfigure rqt_reconfigure

To use the model with the mocap system, run:

	rosrun bifurcationMocap
