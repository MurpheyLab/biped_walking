# biped_walking
Application of hybrid iterative Sequential Action Control (iSAC), a receding horizon method for control of nonlinear systems, to biped walking. The biped model includes plastic impacts at the contact points. Resulting trajectories and control inputs are saved in .csv files and can be plotted using the MATLAB code provided. For more iSAC videos please visit my vimeo page https://vimeo.com/user51518074/videos

# Dependencies
This code requires the the Boost and Eigen libraries.

# Instructions
For this particular example, simply update the makefile with the Boost & Eigen paths, compile and run.

For use with other hybrid systems, all files in the user folder need to appropriately updated.
