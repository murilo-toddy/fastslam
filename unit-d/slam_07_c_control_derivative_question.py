# This adds the derivative of g, this time with respect to the control
# (left and right motor movement).
#
# slam_07_c_control_derivative
# Claus Brenner, 11.12.2012
from lego_robot import *
from math import sin, cos, pi
from numpy import *

class ExtendedKalmanFilter:

    @staticmethod
    def g(state, control, w):
        x, y, theta = state
        l, r = control
        if r != l:
            alpha = (r - l) / w
            rad = l/alpha
            g1 = x + (rad + w/2.)*(sin(theta+alpha) - sin(theta))
            g2 = y + (rad + w/2.)*(-cos(theta+alpha) + cos(theta))
            g3 = (theta + alpha + pi) % (2*pi) - pi
        else:
            g1 = x + l * cos(theta)
            g2 = y + l * sin(theta)
            g3 = theta

        return array([g1, g2, g3])

    @staticmethod
    def dg_dcontrol(state, control, w):
        theta = state[2]
        l, r = tuple(control)
        if r != l:
            alpha = (r - l) / w
            thetap = theta + alpha
            dg1_dl = w * r / ((r - l) ** 2) * (sin(thetap) - sin(theta)) - (r + l) / (2 * (r - l)) * cos(thetap)
            dg2_dl = w * r / ((r - l) ** 2) * (-cos(thetap) + cos(theta)) - (r + l) / (2 * (r - l)) * sin(thetap)
            dg3_dl = - 1 / w

            dg1_dr = -w * l / ((r - l) ** 2) * (sin(thetap) - sin(theta)) + (r + l) / (2 * (r - l)) * cos(thetap)
            dg2_dr = -w * l / ((r - l) ** 2) * (-cos(thetap) + cos(theta)) + (r + l) / (2 * (r - l)) * sin(thetap)
            dg3_dr = 1 / w
        else:
            dg1_dl = 1/2 * (cos(theta) + l/w * sin(theta))
            dg2_dl = 1/2 * (sin(theta) - l/w * cos(theta))
            dg3_dl = - 1 / w

            dg1_dr = 1/2 * (-l/w * sin(theta) + cos(theta))
            dg2_dr = 1/2 * (l/w * cos(theta) + sin(theta))
            dg3_dr = 1 / w

        m = array([[dg1_dl, dg1_dr], [dg2_dl, dg2_dr], [dg3_dl, dg3_dr]])  # Remove this.
            
        return m


if __name__ == '__main__':
    # If the partial derivative with respect to l and r (the control)
    # are correct, then the numerical derivative and the analytical
    # derivative should be the same.

    # Set some variables. Try other variables as well.
    # In particular, you should check cases with l == r and l != r.
    x = 10.0
    y = 20.0
    theta = 35. / 180. * pi
    state = array([x, y, theta])
    l = 50.0
    r = 54.32
    control = array([l, r])
    w = 150.0

    # Compute derivative numerically.
    print("Numeric differentiation dl, dr")
    delta = 1e-7
    control_l = array([l + delta, r])
    control_r = array([l, r + delta])
    dg_dl = (ExtendedKalmanFilter.g(state, control_l, w) -\
             ExtendedKalmanFilter.g(state, control, w)) / delta
    dg_dr = (ExtendedKalmanFilter.g(state, control_r, w) -\
             ExtendedKalmanFilter.g(state, control, w)) / delta
    dg_dcontrol_numeric = column_stack([dg_dl, dg_dr])
    print (dg_dcontrol_numeric)

    # Use the above code to compute the derivative analytically.
    print ("Analytic differentiation dl, dr:")
    dg_dcontrol_analytic = ExtendedKalmanFilter.dg_dcontrol(state, control, w)
    print (dg_dcontrol_analytic)

    # The difference should be close to zero (depending on the setting of
    # delta, above).
    print ("Difference:")
    print (dg_dcontrol_numeric - dg_dcontrol_analytic)
    print ("Seems correct:", allclose(dg_dcontrol_numeric, dg_dcontrol_analytic))
