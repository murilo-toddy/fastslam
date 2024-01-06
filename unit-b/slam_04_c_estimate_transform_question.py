# For each cylinder in the scan, find its cartesian coordinates,
# in the world coordinate system.
# Find the closest pairs of cylinders from the scanner and cylinders
# from the reference, and the optimal transformation which aligns them.
# Then, output the scanned cylinders, using this transform.
# 04_c_estimate_transform
# Claus Brenner, 14 NOV 2012
from lego_robot import *
from slam_b_library import filter_step
from slam_04_a_project_landmarks import\
     compute_scanner_cylinders, write_cylinders
from math import sqrt
from numpy import Infinity as inf


def calculate_distance(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return sqrt((float(x2) - float(x1)) ** 2 + (float(y2) - float(y1)) ** 2)


# Given a list of cylinders (points) and reference_cylinders:
# For every cylinder, find the closest reference_cylinder and add
# the index pair (i, j), where i is the index of the cylinder, and
# j is the index of the reference_cylinder, to the result list.
def find_cylinder_pairs(cylinders, reference_cylinders, max_radius):
    cylinder_pairs = []
    for i in range(len(cylinders)):
        min_dist = inf
        closest_ref = None
        for j in range(len(reference_cylinders)):
            dist = calculate_distance(cylinders[i], reference_cylinders[j])
            if dist < max_radius and dist < min_dist:
                min_dist = dist
                closest_ref = j
        if closest_ref is not None:
            cylinder_pairs.append((i, closest_ref))

    return cylinder_pairs


# Given a point list, return the center of mass.
def compute_center(point_list):
    # Safeguard against empty list.
    if not point_list:
        return (0.0, 0.0)
    # If not empty, sum up and divide.
    sx = sum([float(p[0]) for p in point_list])
    sy = sum([float(p[1]) for p in point_list])
    return (float(sx) / len(point_list), float(sy) / len(point_list))

# Given a left_list of points and a right_list of points, compute
# the parameters of a similarity transform: scale, rotation, translation.
# If fix_scale is True, use the fixed scale of 1.0.
# The returned value is a tuple of:
# (scale, cos(angle), sin(angle), x_translation, y_translation)
# i.e., the rotation angle is not given in radians, but rather in terms
# of the cosine and sine.
def estimate_transform(left_list, right_list, fix_scale = False):
    # Compute left and right center.
    lc = compute_center(left_list)
    rc = compute_center(right_list)
    lcx, lcy = lc
    rcx, rcy = rc

    l_prime = []
    r_prime = []

    if len(left_list) < 3:
        return None

    for l in left_list:
        l_prime.append((float(l[0]) - float(lc[0]), float(l[1]) - float(lc[1])))
    for r in right_list:
        r_prime.append((float(r[0]) - float(rc[0]), float(r[1]) - float(rc[1])))

    cs = ss = rr = ll = 0
    for i in range(len(left_list)):
        rx = r_prime[i][0]
        ry = r_prime[i][1]
        lx = l_prime[i][0]
        ly = l_prime[i][1]

        cs += rx * lx + ry * ly
        ss += -rx * ly + ry * lx
        rr += rx * rx + ry * ry
        ll += lx * lx + ly * ly

    la = sqrt(rr / ll)
    c = cs / sqrt(cs ** 2 + ss ** 2)
    s = ss / sqrt(cs ** 2 + ss ** 2)
    tx = rcx - la * (c*lcx - s*lcy)
    ty = rcy - la * (s*lcx + c*lcy)
    # --->>> Insert here your code to compute lambda, c, s and tx, ty.

    return la, c, s, tx, ty

# Given a similarity transformation:
# trafo = (scale, cos(angle), sin(angle), x_translation, y_translation)
# and a point p = (x, y), return the transformed point.
def apply_transform(trafo, p):
    la, c, s, tx, ty = trafo
    lac = la * c
    las = la * s
    x = lac * p[0] - las * p[1] + tx
    y = las * p[0] + lac * p[1] + ty
    return (x, y)


if __name__ == '__main__':
    # The constants we used for the filter_step.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 150.0

    # The constants we used for the cylinder detection in our scan.    
    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0

    # The maximum distance allowed for cylinder assignment.
    max_cylinder_distance = 300.0

    # The start pose we obtained miraculously.
    pose = (1850.0, 1897.0, 3.717551306747922)

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")

    # Also read the reference cylinders (this is our map).
    logfile.read("robot_arena_landmarks.txt")
    reference_cylinders = [l[1:3] for l in logfile.landmarks]

    out_file = open("estimate_transform.txt", "w")
    for i in range(len(logfile.scan_data)):
        # Compute the new pose.
        pose = filter_step(pose, logfile.motor_ticks[i],
                           ticks_to_mm, robot_width,
                           scanner_displacement)

        # Extract cylinders, also convert them to world coordinates.
        cartesian_cylinders = compute_scanner_cylinders(
            logfile.scan_data[i],
            depth_jump, minimum_valid_distance, cylinder_offset)
        world_cylinders = [LegoLogfile.scanner_to_world(pose, c)
                           for c in cartesian_cylinders]

        # For every cylinder, find the closest reference cylinder.
        cylinder_pairs = find_cylinder_pairs(
            world_cylinders, reference_cylinders, max_cylinder_distance)

        # Estimate a transformation using the cylinder pairs.
        trafo = estimate_transform(
            [world_cylinders[pair[0]] for pair in cylinder_pairs],
            [reference_cylinders[pair[1]] for pair in cylinder_pairs],
            fix_scale = True)

        # Transform the cylinders using the estimated transform.
        transformed_world_cylinders = []
        if trafo:
            transformed_world_cylinders =\
                [apply_transform(trafo, c) for c in
                 [world_cylinders[pair[0]] for pair in cylinder_pairs]]            

        # Write to file.
        # The pose.
        out_file.write("F %f %f %f\n" % pose)
        # The detected cylinders in the scanner's coordinate system.
        write_cylinders(out_file, "D C", cartesian_cylinders)
        # The detected cylinders, transformed using the estimated trafo.
        write_cylinders(out_file, "W C", transformed_world_cylinders)

    out_file.close()
