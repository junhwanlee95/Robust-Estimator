# Robust-Estimator

'RSE_data.mat' is GPS data collected on November 4th, 2019 at University of Texas at San Antonio.

Main.m file will generate the spoofed signal and acquire the clock estimates by using Extended Kalman Filter, Luenberger Observer, and Robust Estimator.

User can select which type of TSA to be simulated by changing 'Type' variable in Main.m.

Since the Robust Estimator requires the solution for optimization problem, installation of CVX function on Matlab is required. 
Following link is available to the public for educational purpose.
http://cvxr.com/cvx/download/

