import numpy as np
import modern_robotics as mr
from code.constants import *
import pandas as pd



def get_matrix_from_vec(vec):
    return np.array([[vec[0], vec[1], vec[2], vec[9]],
                     [vec[3], vec[4], vec[5], vec[10]],
                     [vec[6], vec[7], vec[8], vec[11]],
                     [0, 0, 0, 1]])

def get_vec_from_matrix(matrices, gripper_pos):
    all = []
    for entry in matrices:
        all.append(entry[0, :3].tolist() + entry[1, :3].tolist() + entry[2, :3].tolist() + entry[:, 3].tolist() + [gripper_pos])

    return all

def trajectory_generator(T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k, save):
    ee_history = []




    
    # Segment 1, Move from initial to standoff
    time_taken = total_time * segment_division[0]

    end_pos = np.matmul(T_sc_initial,T_ce_standoff)
    entries = mr.CartesianTrajectory(T_se_initial, end_pos, time_taken, time_taken*k/dt,3)

    ee_history.extend(get_vec_from_matrix(entries, 0))

    # Segment 2, Move from standoff to grip position
    time_taken = total_time* segment_division[1]

    start_pos = end_pos
    end_pos = np.matmul(T_sc_initial, T_ce_grasp)
    entries = mr.CartesianTrajectory(start_pos, end_pos, time_taken, time_taken*k/dt,3)

    ee_history.extend(get_vec_from_matrix(entries, 0))
    # Segment 3, Closing the gripper
    time_taken = total_time * segment_division[2]

    last_entry = ee_history[-1]
    for x in range(int(time_taken*k/0.01)):
        ee_history.append(last_entry[:-1].copy() + [1])

    # Segment 4, Move back to standoff
    start_pos = end_pos
    end_pos = np.matmul(T_sc_initial, T_ce_standoff)
    time_taken = total_time* segment_division[3]

    entries = mr.CartesianTrajectory(start_pos, end_pos, time_taken, time_taken*k/dt,3)

    ee_history.extend(get_vec_from_matrix(entries, 1))

    # Segment 5, Move to standoff above final
    start_pos = end_pos
    end_pos = np.matmul(T_sc_final, T_ce_standoff)

    time_taken = total_time* segment_division[4]

    entries = mr.CartesianTrajectory(start_pos, end_pos, time_taken, time_taken*k/dt,3)

    ee_history.extend(get_vec_from_matrix(entries, 1))

    # Segment 6, Move to final position
    start_pos = end_pos
    end_pos = np.matmul(T_sc_final, T_ce_grasp)

    time_taken = total_time* segment_division[5]

    entries = mr.CartesianTrajectory(start_pos, end_pos, time_taken, time_taken*k/dt,3)

    ee_history.extend(get_vec_from_matrix(entries, 1))
    # Segment 7, Open the gripper
    time_taken = total_time* segment_division[6]

    last_entry = ee_history[-1]
    for x in range(int(time_taken*k/0.01)):
        ee_history.append(last_entry[:-1].copy() + [0])

    # Segment 8, Move to standoff
    start_pos = end_pos
    end_pos = np.matmul(T_sc_final, T_ce_standoff)

    time_taken = total_time* segment_division[7]

    entries = mr.CartesianTrajectory(start_pos, end_pos, time_taken, time_taken*k/dt,3)

    ee_history.extend(get_vec_from_matrix(entries, 0))



    #print(ee_history)
    df = pd.DataFrame(ee_history)
    df.to_csv(save, index=False, header=False)


    return ee_history


if __name__ == "__main__":
    trajectory_generator(T_se_initial=np.matmul(T_b0, M_0e), 
                         T_sc_initial=T_sc_initial, 
                         T_sc_final=T_sc_final, 
                         T_ce_grasp=T_ce_grasp, 
                         T_ce_standoff=T_ce_standoff, 
                         k=1, 
                         save="trajectory.csv")
