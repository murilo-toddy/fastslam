# Implement the second move model for the Lego robot.
# The difference to the first implementation is:
# - added a scanner displacement
# - added a different start pose (measured in the real world)
# - result is now output to a file, as "F" ("filtered") records.
#
# 02_b_filter_motor_file
# Claus Brenner, 09 NOV 2012
from math import sin, cos, pi
from lego_robot import *


# This function takes the old (x, y, heading) pose and the motor ticks
# (ticks_left, ticks_right) and returns the new (x, y, heading).
def filter_step(old_pose, motor_ticks, ticks_to_mm, robot_width,
                scanner_displacement):
    old_lidar_x = old_pose[0]
    old_lidar_y = old_pose[1]
    old_theta = old_pose[2]

    w = robot_width

    # Find out if there is a turn at all.
    if motor_ticks[0] == motor_ticks[1]:
        # No turn. Just drive straight.
        # Think about if you need to modify your old code due to the
        # scanner displacement?
        x = old_lidar_x + motor_ticks[0] * ticks_to_mm * cos(old_theta)
        y = old_lidar_y + motor_ticks[1] * ticks_to_mm * sin(old_theta)
        theta = old_theta
        # --->>> Implement your code to compute x, y, theta here.
        return x, y, theta

    else:
        # Turn. Compute alpha, R, etc.
        # --->>> Modify your previous implementation.
        # First modify the the old pose to get the center (because the
        #   old pose is the LiDAR's pose, not the robot's center pose).
        # Second, execute your old code, which implements the motion model
        #   for the center of the robot.
        # Third, modify the result to get back the LiDAR pose from
        #   your computed center. This is the value you have to return.

        l = motor_ticks[0] * ticks_to_mm
        r = motor_ticks[1] * ticks_to_mm

        alpha = (r - l) / w
        radius = l / alpha

        old_x = old_lidar_x - scanner_displacement * cos(old_theta)
        old_y = old_lidar_y - scanner_displacement * sin(old_theta)

        c_x = old_x - (radius + w / 2) * sin(old_theta)
        c_y = old_y + (radius + w / 2) * cos(old_theta)
        # --->>> Implement your code to compute x, y, theta here.
        theta = (old_theta + alpha) % (2 * pi)
        x = c_x + (radius + w / 2) * sin(theta) + scanner_displacement * cos(theta)
        y = c_y - (radius + w / 2) * cos(theta) + scanner_displacement * sin(theta)
        return x, y, theta


if __name__ == '__main__':
    # Empirically derived distance between scanner and assumed
    # center of robot.
    scanner_displacement = 30.0

    # Empirically derived conversion from ticks to mm.
    ticks_to_mm = 0.349

    # Measured width of the robot (wheel gauge), in mm.
    robot_width = 172.0

    # Measured start position.
    pose = (1850.0, 1897.0, 213.0 / 180.0 * pi)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Loop over all motor tick records generate filtered position list.
    filtered = []
    for ticks in logfile.motor_ticks:
        pose = filter_step(pose, ticks, ticks_to_mm, robot_width,
                           scanner_displacement)
        filtered.append(pose)

    # Write all filtered positions to file.
    f = open("poses_from_ticks.txt", "w")
    for pose in filtered:
        f.write("F %f %f %f\n" % pose)
    f.close()
