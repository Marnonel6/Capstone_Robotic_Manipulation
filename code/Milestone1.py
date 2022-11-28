"""
Cut and paste this command in the command line to generate the csv file: 
(You must be in the same directory as the python file.)
    +  python3 -m Milestone1.py

Non self collision - Make jacobian 0 when close to self collision
page 150 in mr book fro Part 1
"""
import modern_robotics as mr
import numpy as np


def append_to_csv(trajectory_mat):
    """Appends trajectory_mat to an existing csv file."""
    with open("Milestone1.csv",'a') as csvfile:
        np.savetxt(csvfile, trajectory_mat, delimiter = ",")