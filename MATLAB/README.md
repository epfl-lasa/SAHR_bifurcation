##MATLAB code for optimization of the DS.


Structure:

control_parameters/

    plotDynamics.m : Run this GUI to test the parameters learned.


data/

	data_icub/		: Data from iCub (humanoid)
	data_kin/, data_LfD/	: Data from kinesthetic trajectories
	data_polishing/		: Data for validation on real robot
	data_simulation/	: Data from KUKA LWR simulations
	data_toy/		: Additional toy data examples
	data preparation and plotting functions


evaluation/

	DMP/		: Functions for DMP comparison. Copyright "koda" code: Andrej Gams. Copyright old code: Stefan Schaal and Auke Ijspeert
	evaluation functions (Normalized RMSE and cosine similarity)


src/

	conversions/	: Functions converting to and from polar/spherical coordinates
	DS formulation/	: Function resuming the formulation of the dynamical system
	optimization/	: Functions for the actual Least Squares optimization and dimensionality reduction
	toROS/		: Functions to save parameters to file (to be used in ROS and as .txt)


toolboxes/

	drtoolbox/	: Toolbox for dimensionality reduction. Copyright: Laurens van der Maaten, Delft University of Technology, http://homepage.tudelft.nl/19j49
	GMR-dynamics/	: Toolbox for GMM. Copyright: Sylvain Calinon and Micha Hersch, http://lasa.epfl.ch/sourcecode/index.php#GMM


demo.mlx		: Code to reproduce optimizations step.
