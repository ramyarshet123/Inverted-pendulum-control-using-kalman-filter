import numpy as np
import cv2
from scipy.integrate import solve_ivp
import control
from InvertedPendulum2 import InvertedPendulum 
import matplotlib.pyplot as plt

# Linearized System Class
class MyLinearizedSystem:
    def __init__(self):
        g = 9.8
        L = 1.5
        m = 1.0
        M = 5.0
        d1 = 1.0
        d2 = 0.5

        _q = (m + M) * g / (M * L)
        self.A = np.array([
            [0, 1, 0, 0],
            [0, -d1, -g * m / M, 0],
            [0, 0, 0, 1.],
            [0, d1 / L, _q, -d2]
        ])

        self.B = np.expand_dims(np.array([0, 1.0 / M, 0., -1 / (M * L)]), 1)

        self.K = None

    def compute_K(self, desired_eigs=[-0.4, -0.5, -0.6, -0.7]):
        self.K = control.place(self.A, self.B, desired_eigs)

    def get_K(self):
        return self.K


def kalman_filter(A, B, H, x, P, z, desired_poles, Vd,Vn):
    # Compute Kalman gain using control.lqr
    #K, _, E = control.lqr(A.T, H.T, Vd, Vn) #Calculation of K with disturbances
    #K = K.T
    K = control.place(A.T, H.T, desired_poles).T #Calculation of k for the desired poles
    
    # Predict
    x_pred = A @ x
    P_pred = A @ P @ A.T + Vd

    # Update
    innovation = z - H @ x_pred
    S = H @ P_pred @ H.T + Vn
    K = P_pred @ H.T @ np.linalg.inv(S)
    x = x_pred + K @ innovation
    P = (np.eye(len(P)) - K @ H) @ P_pred

    return x, P

def u(t, y, K):
    return -np.matmul(K, y - np.array([0, 0, np.pi / 2., 0]))[0] + control_input

def y_dot(t, y):
    g = 9.8
    L = 1.5
    m = 1.0
    M = 5.0
    d1 = 1.0
    d2 = 0.5

    x_ddot = u(t, y, ss.K) - m * L * y[3] * y[3] * np.cos(y[2]) + m * g * np.cos(y[2]) * np.sin(y[2])
    x_ddot = x_ddot / (M + m - m * np.sin(y[2]) * np.sin(y[2]))

    theta_ddot = -g / L * np.cos(y[2]) - np.sin(y[2]) / L * x_ddot

    damping_x = -d1 * y[1]
    damping_theta = -d2 * y[3]

    return [y[1], x_ddot + damping_x, y[3], theta_ddot + damping_theta]

# System parameters and matrices
ss = MyLinearizedSystem()
Q = np.diag([1, 1, 1, 1])
R = np.diag([1])
#K, _, _ = control.lqr(ss.A, ss.B, Q, R)
#ss.compute_K(desired_eigs=[-1, -2, -3, -4])
K, _, E = control.lqr(ss.A, ss.B, Q, R)
ss.compute_K(desired_eigs= E)

# Kalman filter parameters
Vd = np.eye(4) * 2   # System noise covariance matrix
Vn = np.eye(1) * 0.01      # Measurement noise covariance matrix
print(Vn)
desired_poles = np.array([-2, -3, -4, -5])

# Initial estimates for state and covariance
x_est = np.array([0, 0, np.pi / 2, 0])
P_est = np.eye(4) * 1000

# Data storage for plotting
t_data = []
y_data = []
x_est_data = []

# Main simulation loop
if __name__ == "__main__":
    syst = InvertedPendulum()
    
    control_input = 0.0
    y0 = [0.0, 0.0, np.pi /4 + 0.01, 0.0]

    for t in np.linspace(0, 20, 1000):
        rendered = syst.step(y0, t)
        cv2.imshow('im', rendered)
        cv2.moveWindow('im', 100, 100)

        t_data.append(t)
        y_data.append(y0.copy())
        x_est_data.append(x_est.copy())

        key = cv2.waitKey(50) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('a'):  # "a" key
            control_input += 1.0  # Angle change in degrees
        elif key == ord('d'):  # "d" key
            control_input -= 1.0  # Angle change in degrees
        else:
            control_input = 0.0  # No angle change

        sol = solve_ivp(y_dot, [t, t + 0.1], y0, t_eval=[t + 0.1])
        y0 = sol.y[:, -1]

        # Extract cart position for measurement
        z = np.array([y0[0]]) + np.random.normal(0, 0.1)  # Simulate noisy measurement

        # Kalman filter update
        x_est, P_est = kalman_filter(ss.A, ss.B, np.array([[1, 0, 0, 0]]), x_est, P_est, z, desired_poles, Vd, Vn)

        cv2.waitKey(1)  # Update the display

    cv2.destroyAllWindows()

    # Convert lists to arrays for plotting
    t_data = np.array(t_data)
    y_data = np.array(y_data)
    x_est_data = np.array(x_est_data)

    # Plotting
    plt.figure(figsize=(14, 8))

    plt.subplot(2, 2, 1)
    plt.plot(t_data, y_data[:, 0], label='Real Cart Position')
    plt.plot(t_data, x_est_data[:, 0], label='Estimated Cart Position')
    plt.title('Cart Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.legend()

    plt.subplot(2, 2, 2)
    plt.plot(t_data, y_data[:, 1], label='Real Cart Velocity')
    plt.plot(t_data, x_est_data[:, 1], label='Estimated Cart Velocity')
    plt.title('Cart Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.legend()

    plt.subplot(2, 2, 3)
    plt.plot(t_data, y_data[:, 2], label='Real Pendulum Angle')
    plt.plot(t_data, x_est_data[:, 2], label='Estimated Pendulum Angle')
    plt.title('Pendulum Angle')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')
    plt.legend()

    plt.subplot(2, 2, 4)
    plt.plot(t_data, y_data[:, 3], label='Real Pendulum Angular Velocity')
    plt.plot(t_data, x_est_data[:, 3], label='Estimated Pendulum Angular Velocity')
    plt.title('Pendulum Angular Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.legend()

    plt.tight_layout()
    plt.show()
