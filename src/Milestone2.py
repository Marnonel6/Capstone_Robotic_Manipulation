import modern_robotics as mr
import numpy as np

# Tse,i - End effector initial configuration
# Tsc,i - Cube initial configuration
# Tsc,f - Cube final configuration
# Tce,grasp - End effector configuration relative to cube when grasp
# Tce,standoff - Standoff before and after cube grab relative to cube
# k - 1 or greater - k = 10 then controller at 10000Hz. Easiest is to choose k = 1

# Eight segment trajectorty
# Calc Tse for any instance in time
# Calc gripper state for any instance in time - 1 close 0 open
# Gripper close/open takes 0.625sec thus stand still for that time

# N = 30*k/0.01

# mr.ScrewTrajectory()
# mr.CartesianTrajectory()

# r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper_state = 0


# Gripper geometry
d1 = 7.0/100.0 # Space between open grippers
d2 = 3.5/100.0 # Lenght of fingers
d3 = 4.3/100.0 # Lenght from gripper frame to back of grippers

# Cibe start location
Cube_i = np.array([1,0,0.025])
# Cube goal desired location
Cube_g = np.array([0,-1,0.025])

# Block initial
Tsc_initial = np.array([[1, 0, 0,   1  ],
                        [0, 1, 0,   0  ],
                        [0, 0, 1, 0.025],
                        [0, 0, 0,   1  ]])
# Block Goal
Tsc_goal = np.array([[ 0, 1, 0,   0 ],
                     [-1, 0, 0,  -1 ],
                     [ 0, 0, 1, 0.025],
                     [ 0, 0, 0,   1 ]])
# Gripper initial
Tse_initial = np.array([[ 0, 0, 1,  0 ],
                        [ 0, 1, 0,  0 ],
                        [-1, 0, 0, 0.5],
                        [ 0, 0, 0,  1 ]])
# Standoff before and after cube grab relative to cube
# Add d2/2 to have cube in centre of gripper
# Tse_standoff = np.array([[ 1, 0, 0,  Cube_i[0]+d2/2.0],
#                          [ 0, 1, 0,         Cube_i[1]],
#                          [ 0, 0,-2,            0.3   ],
#                          [ 0, 0, 0,             1    ]])
angle_grasp = 1.785398
Tce_standoff = np.array([[ np.cos(angle_grasp), 0,  np.sin(angle_grasp),  d2/2.0 ],
                         [          0,          1,            0,          0 ],
                         [-np.sin(angle_grasp), 0,  np.cos(angle_grasp), 0.3],
                         [          0,          0,             0,         1 ]])
# End effector configuration relative to cube when grasp,
# Add d2/2 to have cube in centre of gripper
# Tce_grasp = np.array([[ 0, 0, 1, Cube_i[0]+d2/2.0],
#                       [ 0, 1, 0,     Cube_i[1]   ], 
#                       [-1, 0, 0,     Cube_i[2]   ],
#                       [ 0, 0, 0,           1     ]])
# Tce_grasp = np.array([[ 0, 0, 1, 0],
#                       [ 0, 1, 0, 0],
#                       [-1, 0, 0, 0],
#                       [ 0, 0, 0, 1]])
Tce_grasp = np.array([[ np.cos(angle_grasp), 0,  np.sin(angle_grasp),  d2/2.0 ],
                      [          0,          1,            0,          0 ],
                      [-np.sin(angle_grasp), 0,  np.cos(angle_grasp),  0 ],
                      [          0,          0,             0,         1 ]])


def Traj_specs(Tf):
    """Function to calculate N and create a trajectory_mat from the Tf (time)."""
    N = int((Tf/dt)+1)
    trajectory_mat = np.zeros((N,13))
    return N, trajectory_mat


def TrajectoryGenerator(Xstart, Xend, Tf, N, method, trajectory_mat, traj_select):
    """
    Trajectory generator function
    """

    if traj_select == 0:
        traj = mr.CartesianTrajectory(Xstart, Xend, Tf, N, method)
    else:
        traj = mr.ScrewTrajectory(Xstart, Xend, Tf, N, method)

    # a = traj[-1]
    # print(f"\n a = {traj[-1]}")
    for i in range(N):
        m = 0
        for j in range(3):
            for k in range(3): #  r11, r12, r13 - r21, r22, r23, - r31, r32, r33
                l = k+m
                trajectory_mat[i,l] = traj[i][j,k]
            f = 9+j
            trajectory_mat[i,f] = traj[i][j,3] # px, py, pz
            m += 3 # Scale by one row
    return trajectory_mat



# Use for all trajectorys
dt = 0.01
method = 5



"""
Traj 1
"""
Tf = 3 # Amount of time for motion
N, trajectory_mat = Traj_specs(Tf)

Xstart = Tse_initial
Xend = Tsc_initial@Tce_standoff
traj_select = 1 # Select screw trajectory

trajectory_mat = TrajectoryGenerator(Xstart, Xend, Tf, N, method, trajectory_mat, traj_select)
# Overwrite csv file
np.savetxt("Milestone1.csv", trajectory_mat, delimiter = ",")


"""
Traj 2
"""
Tf = 3 # Amount of time for motion
N, trajectory_mat = Traj_specs(Tf)

Xstart = Tsc_initial@Tce_standoff
Xend = Tsc_initial@Tce_grasp
traj_select = 0 # Cartesian trajectory

# print(f"\n {Tsc_initial@Tce_standoff}")

trajectory_mat = TrajectoryGenerator(Xstart, Xend, Tf, N, method, trajectory_mat, traj_select)

# Append to csv file
with open("Milestone1.csv",'a') as csvfile:
    np.savetxt(csvfile, trajectory_mat, delimiter = ",")
