# Implement the first move model for the Lego robot.
# 02_a_filter_motor
# Claus Brenner, 31 OCT 2012
from math import sin, cos, pi
from pylab import *
from lego_robot import *

# This function takes the old (x, y, heading) pose and the motor ticks
# (ticks_left, ticks_right) and returns the new (x, y, heading).
def filter_step(old_pose, motor_ticks, ticks_to_mm, robot_width):
    old_x = old_pose[0]
    old_y = old_pose[1]
    old_theta = old_pose[2]

    w = robot_width

    # Find out if there is a turn at all.
    if motor_ticks[0] == motor_ticks[1]:
        # No turn. Just drive straight.
        x = old_x + motor_ticks[0] * ticks_to_mm * cos(old_theta)
        y = old_y + motor_ticks[1] * ticks_to_mm * sin(old_theta)
        theta = old_theta
        # --->>> Implement your code to compute x, y, theta here.
        return x, y, theta

    else:
        # Turn. Compute alpha, R, etc.
        l = motor_ticks[0] * ticks_to_mm
        r = motor_ticks[1] * ticks_to_mm

        alpha = (r - l) / w
        radius = l / alpha

        c_x = old_x - (radius + w/2) * sin(old_theta)
        c_y = old_y + (radius + w/2) * cos(old_theta)
        # --->>> Implement your code to compute x, y, theta here.
        theta = (old_theta + alpha) % (2*pi)
        x = c_x + (radius + w/2) * sin(theta)
        y = c_y - (radius + w/2) * cos(theta)
        return x, y, theta


if __name__ == '__main__':
    # Empirically derived conversion from ticks to mm.
    ticks_to_mm = 0.349

    # Measured width of the robot (wheel gauge), in mm.
    robot_width = 150.0

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Start at origin (0,0), looking along x axis (alpha = 0).
    pose = (0.0, 0.0, 0.0)

    # Loop over all motor tick records generate filtered position list.
    filtered = []
    for ticks in logfile.motor_ticks:
        pose = filter_step(pose, ticks, ticks_to_mm, robot_width)
        filtered.append(pose)

    # Draw result.
    for pose in filtered:
        print(pose)
        plot([p[0] for p in filtered], [p[1] for p in filtered], 'bo')
    show()
