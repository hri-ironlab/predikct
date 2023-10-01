# PrediKCT

C++ package for converting desired end-effector motion in Cartesian space to joint velocities for redundant robotic manipulators. This package allows conversion of desired Cartesian velocity to joint velocities on any URDF-specified robotic arm with 7+ degrees of freedom. Users can also pass in a custom operator model to be used in prediction of future timesteps. Based on the paper "The Predictive Kinematic Control Tree: Enhancing Teleoperation of Redundant Robots through Probabilistic User Models".

# Usage
This code is provided as a reference example purely for academic purposes. It is not intended for direct usage on other manipulator setups. As it contains some sub-optimal implementation details, it is recommended that interested users implement the PrediKCT algorithm in their own codebase using this repository as a reference rather than directly depending on this package.

For further questions or discussions on using PrediKCT, contact Connor Brooks at connormbrooks@gmail.com.
