import numpy as np
import modern_robotics as mr
from code.constants import H_0
import code.constants as constants
import pandas as pd


def get_T_sb(phi_, x_, y_, z_):
    """
    Uses the equation defined in the wiki to get the T_sb configuration given a phi, x, y, and z
    :param phi_: The phi angle the robot base is at.
    :param x_:   The x coordinate of the body defined in the space frame
    :param y_:   The y coordinate of the body defined in the space frame
    :param z_:   The z coordinate of the body defined in the space frame
    :return:     The T_sb configuration of the robot base
    """

    T_sb = np.array([[np.cos(phi_), -1 * np.sin(phi_), 0, x_],
                        [np.sin(phi_),      np.cos(phi_), 0, y_],
                        [          0,                  0, 1, z_],
                        [          0,                  0, 0,  1]])
    return T_sb

def get_T_se(M_0e, Blist, configuration, T_b0):
    """
    Calculates T_se for a certain configuration
    :param M_0e:          The transformation matrix between the arm base and the end effector
    :param Blist:         The screw list for the arm
    :param configuration: The configuration to generate T_se for
    :param T_b0:          The constant matrix between the base and the arm base
    :return:              A SE matrix T_se
    """

    # Uses forward kinematics to determine the arm frame
    T_0e = mr.FKinBody(M_0e, Blist, configuration[3:8])

    # Uses configuration parsing to determine the base frame
    T_sb = get_T_sb(configuration[0], configuration[1], configuration[2], 0.0963)

    # Uses provided info to determine the arm hub's position relative to the space frame
    T_s0 = np.dot(T_sb, T_b0)

    # Returns the transformation matrix T_se
    return np.dot(T_s0, T_0e)


def next_step(current_config, control, dt, max_angular_speed):
    control = np.clip(control, -max_angular_speed, max_angular_speed)
    wheel_control = control[0:4]
    joint_control = control[4:]

    chassispos = current_config[0:3]
    joint_angle = current_config[3:8]
    wheel_angle = current_config[8:12]

    # Step wheel angles
    joint_angle += joint_control * dt
    wheel_angle += wheel_control * dt

    # Odometry
    F = np.linalg.pinv(H_0)
    Vb = np.dot(F, wheel_control)
    
    if Vb[0] < 1e-6:
        dqb = [0, Vb[1], Vb[2]]
    else:
        dqb = [
                Vb[0], 
               (Vb[1] * np.sin(Vb[0]) + Vb[2] * (np.cos(Vb[0]) - 1)) / Vb[0], 
               (Vb[2] * np.sin(Vb[0]) + Vb[1] * (1 - np.cos(Vb[0]))) / Vb[0]
               ]


    T_sb = np.array([
                    [1,0,0],
                    [0, np.cos(chassispos[0]), -np.sin(chassispos[0])],
                    [0, np.sin(chassispos[0]),  np.cos(chassispos[0])]
                    ])
    delta_chassis = np.dot(T_sb, dqb)

    return np.concatenate((chassispos + delta_chassis, joint_angle, wheel_angle))







if __name__ == "__main__":
    current_config: np.ndarray = np.zeros(12) #tbd
    dt = 0.01
    control = (0,0,0,0,0,-10,10,10,-10)
    past_configs = []
    for i in range(100):
        past_configs.append(current_config.tolist() + [0])
        current_config = next_step(current_config, control, dt, 10)


    df = pd.DataFrame(past_configs)
    df.to_csv('current_config.csv', index=False, header=False)


    

    


