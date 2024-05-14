
# Inverted-pendulum-control-using-kalman-filter
This project focuses on the simulation and control of an inverted pendulum system mounted on a cart using Python. The main objective is to demonstrate the effectiveness of control strategies, both with and without a Kalman filter, in stabilizing the pendulum and maintaining its upright position.

Files inside Project folder:

1. InvertedPendulum2.py: This file contains the simulation code for the inverted pendulum system on a cart. 

2. Without_Kalman_Filter(Only_LQR).py: This file implements a control strategy for stabilizing the inverted pendulum without using a Kalman filter. 

3. With_Kalman_Filter.py: This file implements a control strategy for stabilizing the inverted pendulum using a Kalman filter. 

4. controllability_observability.m: This MATLAB file contains code to analyze the controllability and observability of the inverted pendulum system.


Usage:

1. Run inverted_pendulum.py to visualize the simulation of the inverted pendulum system.

2. Use the 'a' and 'd' keys to apply external forces and observe the system's response.

3. press 'q' to quit the simulation. After that it generates the plot between real and estimated values of parameters.


Dependencies:

Python 3.x

OpenCV (cv2)

NumPy

scipy

matplotlib


Note: Ensure that all dependencies are installed before running the code.
