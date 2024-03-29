# For each cylinder in the scan, find its ray and depth.
# 03_c_find_cylinders
# Claus Brenner, 09 NOV 2012
from pylab import *
from lego_robot import *

# Find the derivative in scan data, ignoring invalid measurements.
def compute_derivative(scan, min_dist):
    jumps = [0]
    for i in range(1, len(scan) - 1):
        l = scan[i-1]
        r = scan[i+1]
        if l > min_dist and r > min_dist:
            derivative = (r - l) / 2.0
            jumps.append(derivative)
        else:
            jumps.append(0)
    jumps.append(0)
    return jumps


# For each area between a left falling edge and a right rising edge,
# determine the average ray number and the average depth.
def find_cylinders(scan, scan_derivative, jump, min_dist):
    cylinder_list = []
    for i in range(len(scan_derivative)):
        # --->>> Insert your cylinder code here.
        # Whenever you find a cylinder, add a tuple
        # (average_ray, average_depth) to the cylinder_list.

        # Just for fun, I'll output some cylinders.
        # Replace this by your code.
        sum_ray, sum_depth, rays = 0.0, 0.0, 0
        if scan_derivative[i] < -jump:
            # Found left edge
            while scan_derivative[i] < jump:
                if scan[i] < min_dist:
                    i += 1
                    continue

                sum_ray += i
                sum_depth += scan[i]
                i += 1
                rays += 1

                if i >= len(scan_derivative):
                    return cylinder_list

                if scan_derivative[i] < -jump:
                    sum_ray, sum_depth, rays = 0.0, 0.0, 0

            avg_point = sum_ray / rays
            avg_depth = sum_depth / rays
            cylinder_list.append((avg_point, avg_depth))

    return cylinder_list


if __name__ == '__main__':
    minimum_valid_distance = 20.0
    depth_jump = 100.0

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_scan.txt")

    # Pick one scan.
    scan = logfile.scan_data[8]

    # Find cylinders.
    der = compute_derivative(scan, minimum_valid_distance)
    cylinders = find_cylinders(scan, der, depth_jump,
                               minimum_valid_distance)

    print(cylinders)

    # Plot results.
    plot(scan)
    scatter([c[0] for c in cylinders], [c[1] for c in cylinders],
        c='r', s=200)
    show()
