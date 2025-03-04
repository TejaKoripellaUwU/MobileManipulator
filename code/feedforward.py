import code.constants as constants
import modern_robotics as mr
import numpy as np

class FeedbackController:
    def __init__(self, Kp, Ki):
        self.Kp = Kp
        self.Ki = Ki
        self.integral = np.zeros(6)





    def feedback_control(self, cur_ee_config, cur_ee_ref, next_ee_ref, dt):
        # Get commanded Vb

        ad = mr.Adjoint(np.linalg.inv(cur_ee_config) @ cur_ee_ref)
        Vd = mr.se3ToVec((1/dt)*(mr.MatrixLog6(np.matmul(np.linalg.inv(cur_ee_ref), next_ee_ref))))
        # print("Vd", Vd)
        feedforward_term = np.matmul(ad, Vd)
        # print("feedforward_term", feedforward_term)

        Xerr = mr.se3ToVec((mr.MatrixLog6(np.matmul(np.linalg.inv(cur_ee_config), cur_ee_ref))))
        self.integral += Xerr*dt
        # print("Xerr", Xerr)

        p_term = np.matmul(self.Kp, Xerr)
        i_term = np.matmul(self.Ki, self.integral)


        commanded_vb = feedforward_term + p_term + i_term

        # print("commanded_vb", commanded_vb)

        return Xerr, commanded_vb


    def get_je(self, cur_config):# cur_config = [phi, x, y, theta1, theta2, theta3, theta4, theta5]
        # Create Jacobian

        # Wheel Base
        F = np.linalg.pinv(constants.H_0)
        F6 = np.array([[0, 0, 0, 0], [0, 0, 0, 0], F[0], F[1], F[2], [0, 0, 0, 0]])

        cur_end_effector = mr.FKinBody(constants.M_0e, constants.Blist, cur_config[3:8])
        ad = mr.Adjoint(np.matmul(np.linalg.inv(cur_end_effector), np.linalg.inv(constants.T_b0)))

        Jw = np.matmul(ad, F6)

        # Arms
        Jb = mr.JacobianBody(constants.Blist, cur_config[3:8])

        return np.concatenate((Jw, Jb), axis=1)
    
    
    def follow_command(self, commanded_vb, cur_config): 
        je = self.get_je(cur_config)

        # print("je", je)



        return np.matmul(np.linalg.pinv(je), commanded_vb)



if __name__ == "__main__":
    controller = FeedbackController(np.zeros((6,6)), np.zeros((6,6)))

    Xd = [[0, 0, 1, 0.5], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]]
    Xd_next = [[0, 0, 1, 0.6], [0, 1, 0, 0], [-1, 0, 0, 0.3], [0, 0, 0, 1]]

    X = [[0.170, 0, 0.985, 0.387], [0, 1, 0, 0], [-0.985, 0, 0.170, 0.570], [0, 0, 0, 1]]

    dt = 0.01

    _, controlled_vb = controller.feedback_control(X, Xd, Xd_next, dt)
    print(controlled_vb)

    current_config = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0])

    control = controller.follow_command(controlled_vb, current_config)
    print(control)




