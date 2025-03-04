import code.feedforward as feedforward
import code.sim as sim
import code.trajectory_generation as trajectory_generation
import code.constants as constants
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def main():
    trajectory = trajectory_generation.trajectory_generator(
        constants.T_se_initial,
        constants.T_sc_initial,
        constants.T_sc_final,
        constants.T_ce_grasp,
        constants.T_ce_standoff,
        constants.k,
        constants.save
    )

    phi    = -np.pi / 4 
    x      = -0.3 
    y      = 0.2      
    current_config = np.array([phi, x, y, 0, -0.262, -0.524, -0.524, 0, 0, 0, 0, 0])

    feedback_controller = feedforward.FeedbackController(np.eye(6)*1.3, np.eye(6)*0)
    # Run Trajectory
    steps = []
    errors = []
    for index in range(len(trajectory)-1):
        cur_ee_config = sim.get_T_se(constants.M_0e, constants.Blist, current_config, constants.T_b0)
        cur_ee_ref = trajectory_generation.get_matrix_from_vec(trajectory[index])
        next_ee_ref = trajectory_generation.get_matrix_from_vec(trajectory[index+1])

        X_err, commanded_vb = feedback_controller.feedback_control(cur_ee_config, cur_ee_ref, next_ee_ref, constants.dt)
        control = feedback_controller.follow_command(commanded_vb, current_config)

        current_config = sim.next_step(current_config, control, constants.dt, constants.max_angular_speed)

        steps.append(current_config.tolist() + [trajectory[index][-1]])
        errors.append(X_err)


    plt.plot(list(range(len(errors))), errors)
    plt.xlabel("Time")
    plt.ylabel("X_err")
    plt.savefig(f"err.png")
    plt.close()

    error_df = pd.DataFrame(errors)
    error_df.to_csv("error.csv", index=False, header=False)


    df = pd.DataFrame(steps)
    df.to_csv("output.csv", index=False, header=False)

if __name__ == "__main__":
    print("Running main")
    main()




    

