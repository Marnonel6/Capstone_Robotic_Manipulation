"""
Cut and paste this command in the command line to generate the csv file: 
(You must be in the same directory as the python file.)
    +  python3 -m Milestone3.py

Non self collision - Make jacobian 0 when close to self collision
page 150 in mr book fro Part 1
"""
import modern_robotics as mr
import numpy as np
import Milestone1

def append_to_csv(CurrentState):
    """Appends CurrentState to an existing csv file."""
    with open("Milestone1.csv",'a') as csvfile:
        np.savetxt(csvfile, CurrentState, delimiter = ",")



Xerr_Total = 0

def FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt):

    Xerr = mr.se3ToVec(mr.MatrixLog6(mr.TransInv(X) @ Xd))

    Vd =  mr.se3ToVec((1/dt) * mr.MatrixLog6((mr.TransInv(Xd) @ Xd_next)))

    # PID with feedforward control
    # Total integral error
    # Xerr_Total += Xerr
    Ve = mr.Adjoint(mr.TransInv(X) @ Xd) @ Vd #+ Kp*Xerr + Ki*Xerr_Total




    # print(f"\nXerr = {Xerr}")
    # print(f"\nKp*Xerr = {Kp*Xerr}")
    # print(f"Vd = {Vd}")
    # print(f"Ve = {Ve}")

    return Ve








""" Test values """
Xd = np.array([[ 0, 0, 1, 0.5],
               [ 0, 1, 0,  0 ],
               [-1, 0, 0, 0.5],
               [ 0, 0, 0,  1 ]])
Xd_next = np.array([[ 0, 0, 1, 0.6],
                    [ 0, 1, 0,  0 ],
                    [-1, 0, 0, 0.3],
                    [ 0, 0, 0,  1 ]])
Xp = np.array([[ 0.170, 0, 0.985, 0.387],
              [   0  , 1,   0  ,   0  ],
              [-0.985, 0, 0.170, 0.570],
              [   0  , 0,   0  ,   1  ]])
Kp = 0
Ki = 0
dt = 0.01

# TODO get from other code
curr_config = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0])

""" End of Test values """














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
# print(f"X = {X}")

# Jacobian of arm
J_arm = mr.JacobianBody(Blist, joint_angles)
# print(f"J_arm = {J_arm}")
J_base = mr.Adjoint(np.linalg.pinv(T0e) @ np.linalg.pinv(Tb0)) @ F6
# print(f"\nJ_base = {J_base}")
Je = np.hstack((J_base, J_arm))
# print(f"Je = {Je}")


# next_state = Milestone1.NextState(CurrentState, ControlVelocities, dt, VelocityLimit)

Ve = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt)

u_theta_Dot = np.linalg.pinv(Je)@Ve
# print(f"[u_theta_Dot] = {u_theta_Dot}")