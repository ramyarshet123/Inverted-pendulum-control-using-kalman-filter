import numpy as np
import cv2

from InvertedPendulum2 import InvertedPendulum

from scipy.integrate import solve_ivp
import control

class MyLinearizedSystem:
    def __init__(self):
        g = 9.8
        L = 1.5
        m = 1.0
        M = 5.0
        d1 = 1.0
        d2 = 0.5

        _q = (m+M) * g / (M*L)
        self.A = np.array([
            [0, 1, 0, 0],
            [0, -d1, -g*m/M, 0],
            [0, 0, 0, 1.],
            [0, d1/L, _q, -d2]
        ])

        self.B = np.expand_dims(np.array([0, 1.0/M, 0., -1/(M*L)]), 1)

    def compute_K(self, desired_eigs=[-0.1, -0.2, -0.3, -0.4]):
        print('[compute_K] desired_eigs=', desired_eigs)
        self.K = control.place(self.A, self.B, desired_eigs)

    def get_K(self):
        return self.K

ss = MyLinearizedSystem()

Q = np.diag([1, 1, 1, 1.])
R = np.diag([1.])
K, S, E = control.lqr(ss.A, ss.B, Q, R)
ss.compute_K(desired_eigs=E)

control_input = 0.0

def u(t, y):
    global control_input
    print('u()', 't=', t, 'control_input=', control_input)
    return -np.matmul(ss.K, y - np.array([0, 0, np.pi/2., 0]))[0] + control_input

def y_dot(t, y):
    g = 9.8
    L = 1.5
    m = 1.0
    M = 5.0
    d1 = 1.0
    d2 = 0.5

    global control_input

    x_ddot = u(t, y) - m*L*y[3]*y[3] * np.cos(y[2]) + m*g*np.cos(y[2]) * np.sin(y[2])
    x_ddot = x_ddot / (M+m-m* np.sin(y[2])* np.sin(y[2]))

    theta_ddot = -g/L * np.cos(y[2]) -  np.sin(y[2]) / L * x_ddot

    damping_x = -d1*y[1]
    damping_theta = -d2*y[3]

    return [y[1], x_ddot + damping_x, y[3], theta_ddot + damping_theta]

if __name__ == "__main__":
    syst = InvertedPendulum()

    # Initial state
    y0 = [0.0, 0.0, np.pi/2 + 0.01, 0.0]

    for t in np.linspace(0, 20, 1000):
        rendered = syst.step(y0, t)
        cv2.imshow('im', rendered)
        cv2.moveWindow('im', 100, 100)

        key = cv2.waitKey(50) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('a'):  # "a" key
            control_input += 1.0  # Angle change in degrees
        elif key == ord('d'):  # "d" key
            control_input -= 1.0  # Angle change in degrees
        else:
            control_input = 0.0  # No angle change

        # Update the state based on the control input
        sol = solve_ivp(y_dot, [t, t+0.5], y0, t_eval=[t+0.5])
        y0 = sol.y[:, -1]

cv2.destroyAllWindows()