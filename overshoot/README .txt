## Capstone_Robotic_Manipulation

# Description:
The python file `Milestone2.py` consistes of a function `TrajectoryGenerator` that uses either `CartesianTrajectory` or `ScrewTrajectory` (Specify `traj_select`.) to calculate the trajectory (A list of transformation matrices along the trajectory.) between two transformation matrices. The `TrajectoryGenerator` returns a matrix where the each row corresponds to a transformation matrix along the trajectory path. The `TrajectoryGenerator` is called for each of the 8 trajectory segments of the gripper. All 8 trajectory segments were then concatenated together in a `Milestone2.csv` file that is used by CoppeliaSim `Scene8` to display the motion of the gripper.

Cut and paste this command in the command line to generate the csv file:
(You must be in the same directory as the python file.)
    +  `python3 -m Milestone2.py`
