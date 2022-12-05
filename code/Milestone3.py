"""
Cut and paste this command in the command line to generate the csv file: 
(You must be in the same directory as the python file.)
    +  python3 -m Milestone3.py

Non self collision - Make jacobian 0 when close to self collision
page 150 in mr book fro Part 1
"""
import modern_robotics as mr
import numpy as np



"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
                                            MILESTONE 1
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

# def Milestone1():
"""
Cut and paste this command in the command line to generate the csv file: 
(You must be in the same directory as the python file.)
    +  python3 -m Milestone1.py

Non self collision - Make jacobian 0 when close to self collision
page 150 in mr book fro Part 1
"""
import modern_robotics as mr
import numpy as np


# def append_to_csv(CurrentState):
#     """Appends CurrentState to an existing csv file."""
#     with open("Milestone1.csv",'a') as csvfile:
#         np.savetxt(csvfile, CurrentState, delimiter = ",")


def NextState(CurrentState, ControlVelocities, dt, VelocityLimit):
    """
    Calculate next state using a simple first-order euler step

    Args:
        + CurrentState -
        + ControlVelocities -
        + dt -
        + VelocityLimit -

    Return:
        + next_state -
    """
    # Current chassis state
    qk = CurrentState[0:3]
    # Old and New joint angle array
    old_joint_angles = CurrentState[3:8]
    new_joint_angles = np.array([0.0,0.0,0.0,0.0,0.0])
    # Old and New wheel angle array
    old_wheel_angles = CurrentState[8:12]
    new_wheel_angles = np.array([0.0,0.0,0.0,0.0])

    # Velocities
    joint_velocity = ControlVelocities[4:]
    wheel_velocity = ControlVelocities[0:4]

    # Constrain velocities to Max velocity limit
    for x in range(len(ControlVelocities)):
        if ControlVelocities[x] > VelocityLimit:
            ControlVelocities[x] = VelocityLimit
        elif ControlVelocities[x] < -VelocityLimit:
            ControlVelocities[x] = -VelocityLimit

    # Calculate new joint angles
    for i in range(len(old_joint_angles)):
        new_joint_angles[i] = old_joint_angles[i] + joint_velocity[i]*dt

    # Calculate new wheel angles
    for i in range(len(old_wheel_angles)):
        new_wheel_angles[i] = old_wheel_angles[i] + wheel_velocity[i]*dt
        # print(f"\n wheel_velocity[i] = {wheel_velocity[i]}, new_wheel_angles[i] = {new_wheel_angles[i]}, old_wheel_angles[i] = {old_wheel_angles[i]}, dt = {dt}, wheel_velocity[i]*dt = {wheel_velocity[i]*dt}")
        # print(f"\n old_wheel_angles[i] + wheel_velocity[i]*dt = {old_wheel_angles[i] + wheel_velocity[i]*dt}")
        # print(f"\n new_wheel_angles = {new_wheel_angles}")
    # Calculate new chassis configuration
    Vb = Fo@wheel_velocity
    # print(f"\n Vb = {Vb}")
    # Tb = np.exp(Vb)
    # Calculate q in the {b} frame
    if Vb[0] == 0: # Omega_b_x = 0
        Delta_qb = np.array([0, Vb[1],Vb[2]])
    else:
        Delta_qb = np.array([Vb[0],
                            (Vb[1]*np.sin(Vb[0]) + Vb[2]*(np.cos(Vb[0])-1))/Vb[0],
                            (Vb[2]*np.sin(Vb[0]) + Vb[1]*(1-np.cos(Vb[0])))/Vb[0]])
    # Change q from the {b} frame to the {s} frame
    phi_k = CurrentState[0]
    Rx = np.array([[1,       0      ,       0      ],
                    [0, np.cos(phi_k),-np.sin(phi_k)],
                    [0, np.sin(phi_k), np.cos(phi_k)]])
    Delta_q = Rx@Delta_qb
    # print(f"Delta_qb = {Delta_qb}")
    # print(f"Delta_q = {Delta_q}")
    # print(f"qk = {qk}")
    new_chassis_cofig = qk + Delta_q*dt
    # print(f"new_chassis_cofig = {new_chassis_cofig}")

    # next_state = np.array([3 x chassis config, 5 x arm config, 4 x wheel config, 0/1 for gripper])
    # next_state = np.concatenate((new_chassis_cofig,new_joint_angles,new_wheel_angles,0), axis = None)
    next_state = np.hstack((new_chassis_cofig,new_joint_angles,new_wheel_angles,0))
    # print(f"\n next_state = {next_state}")

    return next_state



# # Joint and wheel velocity array
# joint_velocity = np.array([0,0,0,0,0])
# wheel_velocity = np.array([10,10,10,10]) # Drive forward
# # wheel_velocity = np.array([-10,10,-10,10]) # Slide sideways
# # wheel_velocity = np.array([-10,10,10,-10]) # Spin in place
# # Max velocity limit
# VelocityLimit = 100 # m/s
# # ControlVelocities
# ControlVelocities = np.hstack((wheel_velocity,joint_velocity))
# # Start State
# Start_state = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
# CurrentState = Start_state
# PrevState = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
# All_states = Start_state

# Robot dimensions
l = float(0.47/2) # Wheel to centre of body in length
w = float(0.3/2) # Wheel to centre of body in width
r = 0.0475 # Wheel radius
Fo = (r/4)*np.array([[-1.0/(l+w), 1.0/(l+w), 1.0/(l+w), -1.0/(l+w)],
                    [    1.0    ,     1.0  ,     1.0  ,     1.0   ],
                    [   -1.0    ,     1.0  ,    -1.0  ,     1.0   ]])

# Use for all trajectorys
# dt = 0.01

# Create csv file
# np.savetxt("Milestone1.csv", Start_state, delimiter = ",")

# for i in range(100):
#     CurrentState = NextState(CurrentState, ControlVelocities, dt, VelocityLimit)
#     All_states = np.vstack((All_states,CurrentState))

# append_to_csv(All_states)

    # return All_states



"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
                                            MILESTONE 2
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

# def Milestone2(X):
def Milestone2():
    """
    Cut and paste this command in the command line to generate the csv file: 
    (You must be in the same directory as the python file.)
        +  python3 -m Milestone2.py
    """
    import modern_robotics as mr
    import numpy as np

    # Total desired trajectory
    Desired_trajectory = np.array([])

    # Gripper geometry
    d1 = 7.0/100.0 # Space between open grippers
    d2 = 3.5/100.0 # Lenght of fingers
    d3 = 4.3/100.0 # Lenght from gripper frame to back of grippers

    # Use for all trajectorys
    dt = 0.01
    method = 5
    k = 1
    Tf_vec = np.array([2,1,1,1,3,1,1,1]) # Time for all trajectories
    gripper_vec = np.array([0, 0, 1, 1, 1, 1, 0, 0]) # Gripper state for each trajectory
    traj_select_vec = np.array([1, 0, 0, 0, 1, 0, 0, 0]) # Select Cartesian or ScrewTrajectory

    # Cube start location
    Cube_i = np.array([1,0,0.025])
    # Cube goal desired location
    Cube_g = np.array([0,-1,0.025])

    # Cube initial configuration
    Tsc_initial = np.array([[1, 0, 0,   1  ],
                            [0, 1, 0,   0  ],
                            [0, 0, 1, 0.025],
                            [0, 0, 0,   1  ]])
    # Cube final configuration
    Tsc_goal = np.array([[ 0, 1, 0,   0  ],
                        [-1, 0, 0,  -1  ],
                        [ 0, 0, 1, 0.025],
                        [ 0, 0, 0,   1  ]])
    # Gripper initial, End effector initial configuration
    Tse_initial = np.array([[ 0, 0, 1,  0 ],
                            [ 0, 1, 0,  0 ],
                            [-1, 0, 0, 0.5],
                            [ 0, 0, 0,  1 ]])
    # Tse_initial = X

    angle_grasp = 1.785398 # Angle for end effector

    # Standoff before and after cube grab relative to cube
    # Add d2/2 to have cube in centre of gripper
    Tce_standoff = np.array([[ np.cos(angle_grasp), 0,  np.sin(angle_grasp),   d2/2.0  ],
                            [          0,          1,            0,            0    ],
                            [-np.sin(angle_grasp), 0,  np.cos(angle_grasp),   0.3   ],
                            [          0,          0,             0,           1    ]])

    # End effector configuration relative to cube when grasp
    # Add d2/2 to have cube in centre of gripper
    Tce_grasp = np.array([[ np.cos(angle_grasp), 0,  np.sin(angle_grasp),  d2/2.0 ],
                        [          0,          1,            0,             0   ],
                        [-np.sin(angle_grasp), 0,  np.cos(angle_grasp),     0   ],
                        [          0,          0,             0,            1   ]])


    def Traj_specs(Tf):
        """Function to calculate N and create a trajectory_mat from the Tf (time)."""
        N = int((Tf*k/dt)+1)
        trajectory_mat = np.zeros((N,13))
        return N, trajectory_mat

    def append_to_csv(trajectory_mat):
        """Appends trajectory_mat to an existing csv file."""
        with open("Milestone2.csv",'a') as csvfile:
            np.savetxt(csvfile, trajectory_mat, delimiter = ",")


    def TrajectoryGenerator(Xstart, Xend, Tf, N, method, trajectory_mat, traj_select, grasp):
        """
        Trajectory generator function
        """
        if traj_select == 0:
            traj = mr.CartesianTrajectory(Xstart, Xend, Tf, N, method)
        else:
            traj = mr.ScrewTrajectory(Xstart, Xend, Tf, N, method)

        for i in range(N):
            m = 0
            for j in range(3):
                for k in range(3): # extract r11, r12, r13 - r21, r22, r23, - r31, r32, r33
                    l = k+m
                    trajectory_mat[i,l] = traj[i][j,k]
                f = 9+j
                trajectory_mat[i,f] = traj[i][j,3] # extract px, py, pz
                m += 3 # Scale by one row [3 elements]
            trajectory_mat[i,12] = grasp # Set Gripper

        return trajectory_mat


    """
    Traj 1 - Initial to standoff initial
    """
    Tf = 5 # Amount of time for motion
    N, trajectory_mat = Traj_specs(Tf)
    grasp = gripper_vec[0] # Open gripper

    Xstart = Tse_initial
    Xend = Tsc_initial@Tce_standoff
    traj_select = traj_select_vec[0] # Select screw trajectory

    trajectory_mat = TrajectoryGenerator(Xstart, Xend, Tf, N, method, trajectory_mat, traj_select,grasp)
    Desired_trajectory = trajectory_mat

    # Create csv file
    np.savetxt("Milestone2.csv", trajectory_mat, delimiter = ",")


    """
    Traj 2 - standoff initial to grasp initial
    """
    Tf = Tf_vec[1] # Amount of time for motion
    N, trajectory_mat = Traj_specs(Tf)
    grasp = gripper_vec[1] # Open gripper

    Xstart = Tsc_initial@Tce_standoff
    Xend = Tsc_initial@Tce_grasp
    traj_select = traj_select_vec[1] # Cartesian trajectory

    trajectory_mat = TrajectoryGenerator(Xstart, Xend, Tf, N, method, trajectory_mat, traj_select, grasp)
    Desired_trajectory = np.vstack((Desired_trajectory, trajectory_mat))

    # Append to csv file
    append_to_csv(trajectory_mat)


    """
    Traj 3 - Grasp initial
    """
    Tf = Tf_vec[2] # Wai for 1sec to grab block
    N, trajectory_mat = Traj_specs(Tf)
    grasp = gripper_vec[2] # Close gripper

    Xstart = Tsc_initial@Tce_grasp
    Xend = Tsc_initial@Tce_grasp
    traj_select = traj_select_vec[2] # Cartesian trajectory

    trajectory_mat = TrajectoryGenerator(Xstart, Xend, Tf, N, method, trajectory_mat, traj_select, grasp)
    Desired_trajectory = np.vstack((Desired_trajectory, trajectory_mat))

    # Append to csv file
    append_to_csv(trajectory_mat)


    """
    Traj 4 - Grasp initial to standoff initial
    """
    Tf = Tf_vec[3] # Amount of time for motion
    N, trajectory_mat = Traj_specs(Tf)
    grasp = gripper_vec[3] # Close gripper

    Xstart = Tsc_initial@Tce_grasp
    Xend = Tsc_initial@Tce_standoff
    traj_select = traj_select_vec[3] # Cartesian trajectory

    trajectory_mat = TrajectoryGenerator(Xstart, Xend, Tf, N, method, trajectory_mat, traj_select, grasp)
    Desired_trajectory = np.vstack((Desired_trajectory, trajectory_mat))

    # Append to csv file
    append_to_csv(trajectory_mat)


    """
    Traj 5 - Standoff initial to standoff goal
    """
    Tf = Tf_vec[4] # Amount of time for motion
    N, trajectory_mat = Traj_specs(Tf)
    grasp = gripper_vec[4] # Close gripper

    Xstart = Tsc_initial@Tce_standoff
    Xend = Tsc_goal@Tce_standoff
    traj_select = traj_select_vec[4] # Screw trajectory

    trajectory_mat = TrajectoryGenerator(Xstart, Xend, Tf, N, method, trajectory_mat, traj_select, grasp)
    Desired_trajectory = np.vstack((Desired_trajectory, trajectory_mat))

    # Append to csv file
    append_to_csv(trajectory_mat)


    """
    Traj 6 - Standoff goal to grasp goal
    """
    Tf = Tf_vec[5] # Amount of time for motion
    N, trajectory_mat = Traj_specs(Tf)
    grasp = gripper_vec[5] # Close gripper

    Xstart = Tsc_goal@Tce_standoff
    Xend = Tsc_goal@Tce_grasp
    traj_select = traj_select_vec[5] # Cartesian trajectory

    trajectory_mat = TrajectoryGenerator(Xstart, Xend, Tf, N, method, trajectory_mat, traj_select, grasp)
    Desired_trajectory = np.vstack((Desired_trajectory, trajectory_mat))

    # Append to csv file
    append_to_csv(trajectory_mat)



    """
    Traj 7 - Grasp goal - Release block
    """
    Tf = Tf_vec[6] # Amount of time for motion
    N, trajectory_mat = Traj_specs(Tf)
    grasp = gripper_vec[6] # Close gripper

    Xstart = Tsc_goal@Tce_grasp
    Xend = Tsc_goal@Tce_grasp
    traj_select = traj_select_vec[6] # Cartesian trajectory

    trajectory_mat = TrajectoryGenerator(Xstart, Xend, Tf, N, method, trajectory_mat, traj_select, grasp)
    Desired_trajectory = np.vstack((Desired_trajectory, trajectory_mat))

    # Append to csv file
    append_to_csv(trajectory_mat)


    """
    Traj 8 - Grasp goal to Standoff goal
    """
    Tf = Tf_vec[7] # Amount of time for motion
    N, trajectory_mat = Traj_specs(Tf)
    grasp = gripper_vec[7] # Close gripper

    Xstart = Tsc_goal@Tce_grasp
    Xend = Tsc_goal@Tce_standoff
    traj_select = traj_select_vec[7] # Cartesian trajectory

    trajectory_mat = TrajectoryGenerator(Xstart, Xend, Tf, N, method, trajectory_mat, traj_select, grasp)
    Desired_trajectory = np.vstack((Desired_trajectory, trajectory_mat))

    # Append to csv file
    append_to_csv(trajectory_mat)

    return Desired_trajectory



"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
                                            MILESTONE 3 
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
def append_to_csv(CurrentState):
    """Appends CurrentState to an existing csv file."""
    with open("Milestone3.csv",'a') as csvfile:
        np.savetxt(csvfile, CurrentState, delimiter = ",")




def FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xerr_Total, Xerr):

    Kp = Kp*np.identity(6)
    Ki = Ki*np.identity(6)

    Xerr = mr.se3ToVec(mr.MatrixLog6(mr.TransInv(X) @ Xd))

    Vd =  mr.se3ToVec((1/dt) * mr.MatrixLog6((mr.TransInv(Xd) @ Xd_next)))

    # PID with feedforward control
    # Total integral error
    # Xerr_Total += Xerr
    Ve = mr.Adjoint(mr.TransInv(X) @ Xd) @ Vd + Kp@Xerr + Ki@Xerr_Total




    # print(f"\nXerr = {Xerr}")
    # print(f"\nKp*Xerr = {Kp*Xerr}")
    # print(f"Vd = {Vd}")
    # print(f"Ve = {Ve}")

    return Ve, Xerr, Xerr_Total



""" Test values """
# Xd = np.array([[ 0, 0, 1, 0.5],
#                [ 0, 1, 0,  0 ],
#                [-1, 0, 0, 0.5],
#                [ 0, 0, 0,  1 ]])
# Xd_next = np.array([[ 0, 0, 1, 0.6],
#                     [ 0, 1, 0,  0 ],
#                     [-1, 0, 0, 0.3],
#                     [ 0, 0, 0,  1 ]])
# Xp = np.array([[ 0.170, 0, 0.985, 0.387],
#               [   0  , 1,   0  ,   0  ],
#               [-0.985, 0, 0.170, 0.570],
#               [   0  , 0,   0  ,   1  ]])

# # TODO get from other code
# curr_config = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0])
# # TODO Should come from miletone 1

""" End of Test values """


# TODO change
curr_config = np.array([0, 0, 0, 0, 0, 0, 0, 0])

# PID Gains
Kp = 10 # 1
Ki = 0  # 0 

# Time step
dt = 0.01

# Robot dimensions
l = float(0.47/2) # Wheel to centre of body in length
w = float(0.3/2) # Wheel to centre of body in width
r = 0.0475 # Wheel radius

F6 = (r/4)*np.array([[0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [-1.0/(l+w), 1.0/(l+w), 1.0/(l+w), -1.0/(l+w)],
                    [    1.0    ,     1.0  ,     1.0  ,     1.0   ],
                    [   -1.0    ,     1.0  ,    -1.0  ,     1.0   ],
                    [0, 0, 0, 0]])


# Fixed offset from chassis frame {b} to base of arm frame {0}
Tb0 = np.array([[1, 0, 0, 0.1662],
                [0, 1, 0,    0  ],
                [0, 0, 1, 0.0026],
                [0, 0, 0,    1  ]])
# Home configuration. End effector frame {e} relative to the arm base frame {0}
M0e = np.array([[1, 0, 0,  0.033],
                [0, 1, 0,    0  ],
                [0, 0, 1, 0.6546],
                [0, 0, 0,    1  ]])
# Arm at home configuration, the screw axes in {b} frame for the five joints are expressed
# in the end-effector frame {e} as
Blist = np.array([[  0  ,    0,     0,       0,   0],
                [  0  ,   -1,    -1,      -1,   0],
                [  1  ,    0,     0,       0,   1],
                [  0  ,-0.5076,-0.3526,-0.2176, 0],
                [0.033,    0,     0,       0,   0],
                [  0,      0,     0,       0,   0]])

# Extraxt joint angles
joint_angles = curr_config[3:]
# Calculate transformation matrix of end-effector {e} relative to the base of the arm {0}
T0e = mr.FKinBody(M0e,Blist,joint_angles)
# print(f"\nTe = {T0e}")

# Extraxt robot base angle and x,yz coordinates
phi = curr_config[0]
x = curr_config[1]
y = curr_config[2]
z = 0.0963
# Chassis frame {b} relative to world frame {s}
Tsb = np.array([[np.cos(phi), -np.sin(phi), 0, x],
                [np.sin(phi),  np.cos(phi), 0, y],
                [    0      ,       0     , 1, z],
                [    0      ,       0     , 0, 1]])

# Tse - Current configuration of robot end-effector {e} relative to world frame {s}
X = Tsb @ Tb0 @ T0e
# X = mr.FKinBody(X,Blist,joint_angles)
# print(f"X = {X}")


# Jacobian of arm
J_arm = mr.JacobianBody(Blist, joint_angles)
# print(f"J_arm = {J_arm}")
J_base = mr.Adjoint(np.linalg.pinv(T0e) @ np.linalg.pinv(Tb0)) @ F6
# print(f"\nJ_base = {J_base}")
Je = np.hstack((J_base, J_arm))
# print(f"Je = {Je}")


""" Milestone2 Reference Trajectory. """
# Creat desired trajectory
# Desired_trajectory = Milestone2(X) # Send Tse
Desired_trajectory = Milestone2()
# print(f"Desired_trajectory = {Desired_trajectory}")
# print(f"Shape Desired_trajectory = {Desired_trajectory.shape}")

def TransMat(R):
    return np.array([[R[0], R[1], R[2], R[9]],
                     [R[3], R[4], R[5], R[10]],
                     [R[6], R[7], R[8], R[11]],
                     [ 0  ,  0  ,   0 ,   1  ]])

# Max velocity
VelocityLimit = 100

# Gripper initial, End effector initial configuration
Tse_initial = np.array([[ 0, 0, 1,  0 ],
                        [ 0, 1, 0,  0 ],
                        [-1, 0, 0, 0.5],
                        [ 0, 0, 0,  1 ]])
# print(f"X = {X}")
# X = Tse_initial
# Start_state = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]) # TODO delete/
Start_state = np.array([0.6, -0.2, -0.2, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0, 0])
Start_state = np.array([0.6, -0.2, -0.2, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0, 0])
CurrentState = Start_state
Xtotal = Start_state

Xerr_Total = np.zeros((6))
Xerr = np.zeros((6))

for t in range(0, Desired_trajectory.shape[0]-1):
    # Milestone2 - Calculate desired trajectory and next desired trajectory
    Xd = TransMat(Desired_trajectory[t])
    Xd_next = TransMat(Desired_trajectory[t+1])


    # print(f"Next_X = {Next_X}")
    # Calculate new current state
    curr_config = CurrentState[:12]
    # print(f"curr_config = {curr_config}")
    # Extraxt joint angles
    joint_angles = curr_config[3:8]

    # print(f"joint_angles = {joint_angles}")
    # Calculate transformation matrix of end-effector {e} relative to the base of the arm {0}
    T0e = mr.FKinBody(M0e,Blist,joint_angles)

    # Extraxt robot base angle and x,yz coordinates
    phi = curr_config[0]
    x = curr_config[1]
    y = curr_config[2]
    z = 0.0963
    # Chassis frame {b} relative to world frame {s}
    Tsb = np.array([[np.cos(phi), -np.sin(phi), 0, x],
                    [np.sin(phi),  np.cos(phi), 0, y],
                    [    0      ,       0     , 1, z],
                    [    0      ,       0     , 0, 1]])


    # Tse - Current configuration of robot end-effector {e} relative to world frame {s}
    X = Tsb @ Tb0 @ T0e
    # X = mr.FKinBody(X,Blist,joint_angles)
    # CurrentState = Next_X



    # Jacobian of arm
    J_arm = mr.JacobianBody(Blist, joint_angles)
    # print(f"J_arm = {J_arm}")
    J_base = mr.Adjoint(np.linalg.pinv(T0e) @ np.linalg.pinv(Tb0)) @ F6
    # print(f"\nJ_base = {J_base}")
    Je = np.hstack((J_base, J_arm))
    # print(f"Je = {Je}")

    # Milestone 3 - PID controller 
    Ve, Xerr, Xerr_Total= FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xerr, Xerr_Total)
    Xerr_Total += Xerr*dt

    # print(f"Xerr = {Xerr}")



    # Command velocities
    u_theta_Dot = np.linalg.pinv(Je)@Ve
    # print(f"\nu_theta_Dot = {u_theta_Dot}")
    ControlVelocities = u_theta_Dot

    # Milestone 1 - Get next state
    CurrentState = NextState(CurrentState, ControlVelocities, dt, VelocityLimit)


    # print(f"Next_X = {Next_X}")
    stack = CurrentState
    stack[-1] = Desired_trajectory[t][-1]
    Xtotal = np.vstack((Xtotal, stack))




""" WITH COLLISION DETECTION - Gets stuck though """

# for t in range(0, Desired_trajectory.shape[0]-1):
#     # Milestone2 - Calculate desired trajectory and next desired trajectory
#     Xd = TransMat(Desired_trajectory[t])
#     Xd_next = TransMat(Desired_trajectory[t+1])

#     # Milestone 3 - PID controller 
#     Ve, Xerr, Xerr_Total= FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xerr, Xerr_Total)
#     Xerr_Total += Xerr*dt

#     arm1 = np.array([-1.2, 1.2])
#     arm2 = np.array([-1.116,0.3])
#     arm3 = np.array([-1.9, 0.5])
#     arm4 = np.array([-1.7, 0.1])
#     print(f"joint_angles = {joint_angles}")
#     collision = True
#     while collision == True:
#         collision = False
#         # Command velocities
#         u_theta_Dot = np.linalg.pinv(Je)@Ve
#         # print(f"\nu_theta_Dot = {u_theta_Dot}")
#         ControlVelocities = u_theta_Dot

#         # Milestone 1 - Get next state
#         Next_X = NextState(CurrentState, ControlVelocities, dt, VelocityLimit)
#         # print(f"Next_X = {Next_X}")
#         # Calculate new current state
#         curr_config = Next_X[:12]
#         # print(f"curr_config = {curr_config}")
#         # Extraxt joint angles
#         joint_angles = curr_config[3:8]

#         # print(f"joint_angles = {joint_angles}")
#         # print(f"Je = {Je[:,4:]}")

#         if joint_angles[0] < arm1[0] or joint_angles[0] > arm1[1]:
#             # print(f"1")
#             collision = True
#             for j in range(5):
#                 Je[j,4] = 0
#         if joint_angles[1] < arm2[0] or joint_angles[1] > arm2[1]:
#             # print(f"2")
#             collision = True
#             for j in range(5):
#                 Je[j,5] = 0
#         if joint_angles[2] < arm3[0] or joint_angles[2] > arm3[1]:
#             # print(f"3")
#             collision = True
#             for j in range(5):
#                 Je[j,6] = 0
#         if joint_angles[3] < arm4[0] or joint_angles[3] > arm4[1]:
#             # print(f"4")
#             collision = True
#             for j in range(5):
#                 Je[j,7] = 0

#     print(" No collision ")
#         # def testJointLimits(joint):
#         #     if joint[0] > arm1[1]:
#         #         joint
        

#         #     return 


#     # """ Test Joint Limits """
#     # testJointLimits(joint_angles)


#     # print(f"joint_angles = {joint_angles}")
#     # Calculate transformation matrix of end-effector {e} relative to the base of the arm {0}
#     T0e = mr.FKinBody(M0e,Blist,joint_angles)

#     # Extraxt robot base angle and x,yz coordinates
#     phi = curr_config[0]
#     x = curr_config[1]
#     y = curr_config[2]
#     z = 0.0963
#     # Chassis frame {b} relative to world frame {s}
#     Tsb = np.array([[np.cos(phi), -np.sin(phi), 0, x],
#                     [np.sin(phi),  np.cos(phi), 0, y],
#                     [    0      ,       0     , 1, z],
#                     [    0      ,       0     , 0, 1]])


#     # Tse - Current configuration of robot end-effector {e} relative to world frame {s}
#     X = Tsb @ Tb0 @ T0e
#     CurrentState = Next_X
#     # print(f"Next_X = {Next_X}")
#     stack = Next_X
#     stack[-1] = Desired_trajectory[t][-1]
#     Xtotal = np.vstack((Xtotal, stack))


#     # Jacobian of arm
#     J_arm = mr.JacobianBody(Blist, joint_angles)
#     # print(f"J_arm = {J_arm}")
#     J_base = mr.Adjoint(np.linalg.pinv(T0e) @ np.linalg.pinv(Tb0)) @ F6
#     # print(f"\nJ_base = {J_base}")
#     Je = np.hstack((J_base, J_arm))
#     # print(f"Je = {Je}")


# Create csv file
np.savetxt("Capstone.csv", Xtotal, delimiter = ",")

    # print(f"Xd = {Xd}")
    # print(f"[u_theta_Dot] = {u_theta_Dot}")




