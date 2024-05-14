import numpy as np
import cv2

class InvertedPendulum:
    def __init__(self):
        pass

    def step(self, state_vec, t=None):
        """ state vector :
                x0 : position of the cart
                x1 : velocity of the cart
                x2 : angle of pendulum. In ref frame with x as forward of the cart and y as up. Angle with respect to ground plane
                x3 : angular velocity of the pendulum
        """
        CART_POS = state_vec[0]
        CART_VEL = state_vec[1]
        BOB_ANG = state_vec[2] * 180. / np.pi  # degrees
        BOB_ANG_VEL = state_vec[3]
        LENGTH_OF_PENDULUM = 110.

        IM = np.ones((512, 512, 3), dtype='uint8')*255

        # Ground line
        cv2.line(IM, (0, 450), (IM.shape[1], 450), (0, 60, 120), 90)  

        # Draw Wheels of the cart
        wheel_1_pos = int((CART_POS - 1.2 + 5) / 10 * IM.shape[0])
        wheel_2_pos = int((CART_POS + 1.2 + 5) / 10 * IM.shape[0])
        cv2.circle(IM, (wheel_1_pos, 415), 25, (50, 50, 50), 7)  
        cv2.circle(IM, (wheel_2_pos, 415), 25, (50, 50, 50), 7) 
        cv2.circle(IM, (wheel_1_pos, 415), 2, (0, 0, 0), -1)  
        cv2.circle(IM, (wheel_2_pos, 415), 2, (0, 0, 0), -1)  

        # Cart base
        cart_base_start = int((CART_POS - 2.5 + 5) / 10 * IM.shape[0])
        cart_base_end = int((CART_POS + 2.5 + 5) / 10 * IM.shape[0])
        cv2.line(IM, (cart_base_start, 380), (cart_base_end, 380), (160, 40, 40), 6)  # 

        # Pendulum hinge
        pendulum_hinge_x = int((CART_POS + 5) / 10 * IM.shape[0])
        pendulum_hinge_y = 380
        cv2.circle(IM, (pendulum_hinge_x, pendulum_hinge_y), 10, (120, 120, 120), -1)

        # Pendulum
        pendulum_bob_x = int(LENGTH_OF_PENDULUM * np.cos(BOB_ANG / 180. * np.pi))
        pendulum_bob_y = int(LENGTH_OF_PENDULUM * np.sin(BOB_ANG / 180. * np.pi))
        cv2.circle(IM, (pendulum_hinge_x + pendulum_bob_x, pendulum_hinge_y - pendulum_bob_y), 10, (0, 0, 192), -1)  # changed color to light grey
        cv2.line(IM, (pendulum_hinge_x, pendulum_hinge_y),
                 (pendulum_hinge_x + pendulum_bob_x, pendulum_hinge_y - pendulum_bob_y), (0, 0, 0), 3)  # changed color to light grey

        # Mark the current angle
        angle_display = BOB_ANG % 360
        if angle_display > 180:
            angle_display = -360 + angle_display
        cv2.putText(IM, "theta=" + str(np.round(angle_display, 4)) + " deg",
                    (pendulum_hinge_x - 15, pendulum_hinge_y - 15), cv2.FONT_HERSHEY_TRIPLEX, 0.4,
                    (0, 0, 0), 1)

        # Display on top
        if t is not None:
            cv2.putText(IM, "t=" + str(np.round(t, 4)) + " sec", (15, 15), cv2.FONT_HERSHEY_TRIPLEX, 0.5,
                        (0, 0, 0), 1)
            cv2.putText(IM, "theta=" + str(np.round(BOB_ANG, 4)) + " degrees", (350, 15),
                        cv2.FONT_HERSHEY_TRIPLEX, 0.4, (0, 0, 0), 1)
            cv2.putText(IM, "x=" + str(np.round(CART_POS, 4)) + " m", (350, 35), cv2.FONT_HERSHEY_TRIPLEX, 0.4,
                        (0, 0, 0), 1)
            cv2.putText(IM, "x_dot=" + str(np.round(CART_VEL, 4)) + " m/s", (350, 55), cv2.FONT_HERSHEY_TRIPLEX, 0.4,
                        (0, 0, 0), 1)
            cv2.putText(IM, "theta_dot=" + str(np.round(BOB_ANG_VEL, 4)) + " deg/s", (350, 75), cv2.FONT_HERSHEY_TRIPLEX, 0.4,
                        (0, 0, 0), 1)

        return IM


if __name__ == "__main__":
    syst = InvertedPendulum()

    x = 0.
    sx = 1.
    theta = np.pi / 3
    stheta = np.pi / 3
    t = 0.
    while True:
        rendered = syst.step([x, 0, theta, 0], t)
        cv2.imshow('im', rendered)

        if cv2.waitKey(30) == ord('q'):
            break

        t += 30. / 1000.
