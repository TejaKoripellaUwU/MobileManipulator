import numpy as np

one = 2
two = 1


k = 1
save = "trajectory.csv"
dt = 0.01
max_angular_speed = 12.3

# Physical Constants
class Chassis:
    wheel_rad = 0.0475
    length = 0.235
    width = 0.15
    height = 0.0963


T_se_initial = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]])



T_b0 = np.array([[1, 0, 0, 0.1662],
                [0, 1, 0, 0],
                [0, 0, 1, 0.0026],
                [0, 0, 0, 1]])

M_0e = np.array([[1, 0, 0, 0.033],
                [0, 1, 0, 0],
                [0, 0, 1, 0.6546],
                [0, 0, 0, 1]])


T_sc_initial = np.array([[1, 0, 0, 2],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0.025],
                         [0, 0, 0, 1]])

T_sc_final = np.array([[0, 1, 0, 0],
                       [-1, 0, 0, -2],
                       [0, 0, 1, 0.025],
                       [0, 0, 0, 1]])

T_ce_grasp = np.array([[0, 0, 1, 0],
                            [0, 1, 0, 0],
                            [-1, 0, 0, 0],
                            [0, 0, 0, 1]])

T_ce_standoff = np.array([[0, 0, 1, 0],
                                [0, 1, 0, 0],
                                [-1, 0, 0, 0.25],
                                [0, 0, 0, 1]])

Blist = np.array([[ 0.    ,  0.    ,  0.    ,  0.    ,  0.    ],
       [ 0.    , -1.    , -1.    , -1.    ,  0.    ],
       [ 1.    ,  0.    ,  0.    ,  0.    ,  1.    ],
       [ 0.    , -0.5076, -0.3526, -0.2176,  0.    ],
       [ 0.033 ,  0.    ,  0.    ,  0.    ,  0.    ],
       [ 0.    ,  0.    ,  0.    ,  0.    ,  0.    ]])

# Calculations
r, l, w = Chassis.wheel_rad, Chassis.length, Chassis.width

total_time = 45
segment_division = [0.25, 0.1, 0.05, 0.1, 0.25, 0.1, 0.05, 0.1]

starting_config_ee = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]])
standoff_config_ee = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]])

H_0 = (1/r)*np.array([[-l-w, 1, -1],
                      [ l+w,  1,  1],
                      [ l+w,  1, -1],
                      [-l-w, 1, 1]])




