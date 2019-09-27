import sys
import math
from robot_config import RobotConfig, make_robot_config_from_ee1
from problem_spec import ProblemSpec
from obstacle import Obstacle
from angle import Angle

"""
Tester script.

Use this script to test whether your output files are valid solutions. This script takes 2 arguments - an input file
and your solution file.

You should avoid modifying this file directly. You may use code from functions in this file (e.g. collision checking)
either directly or by copying into your own file. Note that the implementations in this file are not necessarily
the most efficient way possible.

COMP3702 2019 Assignment 2 Support Code

Last updated by njc 24/08/19
"""


def load_output(filename):
    # return a list of RobotConfig objects
    robot_configs = []
    f = open(filename, 'r')
    for line in f:
        ee1_xy_str, ee1_angles_str, lengths_str = line.strip().split(';')
        ee1x, ee1y = tuple([float(i) for i in ee1_xy_str.split(' ')])
        ee1_angles = [Angle(degrees=float(i)) for i in ee1_angles_str.strip().split(' ')]
        lengths = [float(i) for i in lengths_str.strip().split(' ')]
        robot_configs.append(make_robot_config_from_ee1(ee1x, ee1y, ee1_angles, lengths))
    return robot_configs


def test_bounding_box(p1, q1, p2, q2):
    # return true for collision possible, false otherwise
    p1x, p1y = p1
    q1x, q1y = q1
    p2x, p2y = p2
    q2x, q2y = q2
    x1_min = min(p1x, q1x)
    x1_max = max(p1x, q1x)
    x2_min = min(p2x, q2x)
    x2_max = max(p2x, q2x)
    if x1_max < x2_min or x2_max < x1_min:
        return False

    y1_min = min(p1y, q1y)
    y1_max = max(p1y, q1y)
    y2_min = min(p2y, q2y)
    y2_max = max(p2y, q2y)
    if y1_max < y2_min or y2_max < y1_min:
        return False

    return True


def determinant(a, b, c, d):
    return (a * d) - (b * c)


def triangle_orientation(a, b, c):
    ax, ay = a
    bx, by = b
    cx, cy = c
    area = determinant(bx, by, cx, cy) - determinant(ax, ay, cx, cy) + determinant(ax, ay, bx, by)
    if area > 0:
        return 1
    elif area < 0:
        return -1
    else:
        return 0


def test_orientation(p1, q1, p2, q2):
    # return true for collision, false otherwise
    if triangle_orientation(p1, q1, p2) == triangle_orientation(p1, q1, q2):
        return False
    if triangle_orientation(p2, q2, p1) == triangle_orientation(p2, q2, q1):
        return False
    return True


def test_line_collision(line1, line2):
    # return true for collision, false otherwise
    p1, q1 = line1
    p2, q2 = line2
    if not test_bounding_box(p1, q1, p2, q2):
        return False
    return test_orientation(p1, q1, p2, q2)


def test_environment_bounds(config):
    # return true for pass, false for fail
    for x, y in config.points:
        if not 0.0 <= x <= 1.0:
            return False
        if not 0.0 <= y <= 1.0:
            return False
    return True


def test_angle_constraints(config, spec):
    # return true for pass, false for fail
    for i in range(1, spec.num_segments):
        a = config.ee1_angles[i]
        if not ((-11 * math.pi / 12) - spec.TOLERANCE < a < (11 * math.pi / 12) + spec.TOLERANCE):
            # internal angle tighter than 15 degrees
            return False
    return True


def test_length_constraints(config, spec):
    # return true for pass, false for fail
    for i in range(spec.num_segments):
        if config.lengths[i] < spec.min_lengths[i] - spec.TOLERANCE or \
                config.lengths[i] > spec.max_lengths[i] + spec.TOLERANCE:
            return False
    return True


def point_is_close(x1, y1, x2, y2, tolerance):
    return abs(x2 - x1) + abs(y2 - y1) < tolerance


def test_grapple_point_constraint(config, spec):
    # return true for pass, false for fail
    ee1x, ee1y = config.points[0]
    ee2x, ee2y = config.points[-1]
    for gpx, gpy in spec.grapple_points:
        if (point_is_close(ee1x, ee1y, gpx, gpy, spec.TOLERANCE) or
            point_is_close(ee2x, ee2y, gpx, gpy, spec.TOLERANCE)) and \
                not point_is_close(ee1x, ee1y, ee2x, ee2y, spec.TOLERANCE):
            return True
    return False


def test_self_collision(config, spec):
    # return true for pass, false for fail
    if spec.num_segments < 3:
        # collision impossible with less than 3 segments
        return True
    # do full check
    for i in range(spec.num_segments - 1):
        p1 = config.points[i]
        q1 = config.points[i+1]

        for j in range(i + 2, spec.num_segments):
            p2 = config.points[j]
            q2 = config.points[j+1]

            if test_line_collision((p1, q1), (p2, q2)):
                return False

    return True


def __get_lenient_obstacle_bounds(obstacle, spec):
    """
    This method should only be used by tester. To avoid unexpected errors in your solution caused by floating point
    noise, you should not use this method in your solver.
    """
    # shrink obstacle by TOLERANCE in each direction
    return Obstacle(obstacle.x1 + spec.TOLERANCE, obstacle.y1 + spec.TOLERANCE,
                    obstacle.x2 - spec.TOLERANCE, obstacle.y2 - spec.TOLERANCE)


def __get_lenient_obstacles(spec):
    """
    This method should only be used by tester. To avoid unexpected errors in your solution caused by floating point
    noise, you should not use this method in your solver.
    """
    # shrink all obstacles by TOLERANCE
    obstacles = []
    for o in spec.obstacles:
        obstacles.append(__get_lenient_obstacle_bounds(o, spec))
    return obstacles


def test_obstacle_collision(config, spec, obstacles):
    # return true for pass, false for fail
    for i in range(spec.num_segments):
        p = config.points[i]
        q = config.points[i+1]
        for o in obstacles:
            # bounding box check
            if not test_bounding_box(p, q, (o.x1, o.y1), (o.x2, o.y2)):
                continue

            # full edge check
            for e in o.edges:
                if test_line_collision((p, q), e):
                    # collision between robot segment and obstacle edge
                    return False
    return True


def test_config_equality(c1, c2, spec):
    """
    Check for equality between robot config objects.
    :param other: object for comparison
    :return: True if equal (i.e. all points match), false otherwise
    """
    if not isinstance(c1, RobotConfig) or not isinstance(c2, RobotConfig):
        return False
    for i in range(spec.num_segments + 1):
        if not point_is_close(c1.points[i][0], c1.points[i][1], c2.points[i][0], c2.points[i][1], spec.TOLERANCE):
            return False
    for i in range(spec.num_segments):
        if abs(c2.lengths[i] - c1.lengths[i]) > spec.TOLERANCE:
            return False
    return True


def test_config_distance(c1, c2, spec):
    # return maximum distance between 2 configurations
    max_delta = 0

    max_ee1_delta = 0
    max_ee2_delta = 0
    for i in range(spec.num_segments):
        if abs((c2.ee1_angles[i] - c1.ee1_angles[i]).in_radians()) > max_ee1_delta:
            max_ee1_delta = abs((c2.ee1_angles[i] - c1.ee1_angles[i]).in_radians())

        if abs((c2.ee2_angles[i] - c1.ee2_angles[i]).in_radians()) > max_ee2_delta:
            max_ee2_delta = abs((c2.ee2_angles[i] - c1.ee2_angles[i]).in_radians())

    # measure leniently - allow compliance from EE1 or EE2
    max_delta = min(max_ee1_delta, max_ee2_delta)

    for i in range(spec.num_segments):
        if abs(c2.lengths[i] - c1.lengths[i]) > max_delta:
            max_delta = abs(c2.lengths[i] - c1.lengths[i])

    if max_delta > spec.PRIMITIVE_STEP + spec.TOLERANCE:
        return False
    return True


def main(arglist):
    input_file = arglist[0]
    soln_file = arglist[1]

    spec = ProblemSpec(input_file)
    robot_configs = load_output(soln_file)
    lenient_obstacles = __get_lenient_obstacles(spec)
    violations = 0

    # run validity tests for each config
    if not test_config_equality(robot_configs[0], spec.initial, spec):
        violations += 1
        if violations <= 10:
            print("!!! The first robot configuration does not match the initial position from the problem spec !!!")

    for i in range(len(robot_configs)):
        if not test_environment_bounds(robot_configs[i]):
            violations += 1
            if violations <= 10:
                print("!!! Robot goes outside the bounds of the environment at step number " +
                      str(i) + " !!!")

        if not test_angle_constraints(robot_configs[i], spec):
            violations += 1
            if violations <= 10:
                print("!!! One or more of the angles between robot segments is tighter than allowed at step number " +
                      str(i) + " !!!")

        if not test_length_constraints(robot_configs[i], spec):
            violations += 1
            if violations <= 10:
                print("!!! One or more of the robot segments is shorter or longer than allowed at step number " +
                      str(i) + " !!!")

        if not test_grapple_point_constraint(robot_configs[i], spec):
            violations += 1
            if violations <= 10:
                print("!!! Robot is not connected to at least 1 grapple point (or a grapple point is occupied by " +
                      "more than one end effector) at step number " + str(i) + " !!!")

        if not test_self_collision(robot_configs[i], spec):
            violations += 1
            if violations <= 10:
                print("!!! Robot is in collision with itself at step number " + str(i) + " !!!")

        if not test_obstacle_collision(robot_configs[i], spec, lenient_obstacles):
            violations += 1
            if violations <= 10:
                print("!!! Robot is in collision with an obstacle at step number " + str(i) + " !!!")

        if i + 1 < len(robot_configs) and not test_config_distance(robot_configs[i], robot_configs[i+1], spec):
            violations += 1
            if violations <= 10:
                print("!!! Step size is greater than primitive step limit between step number " + str(i) + " and " +
                      str(i + 1) + " !!!")

    if not test_config_equality(robot_configs[-1], spec.goal, spec):
        violations += 1
        if violations <= 10:
            print("!!! The last robot configuration does not match the goal position from the problem spec !!!")

    # print summary
    if violations > 10:
        print("!!! " + str(violations - 10) + " other rule violation(s) !!!")

    if violations == 0:
        print("Testcase solved successfully!")
        return 0
    else:
        print("Invalid solution file - " + str(violations) + " violations encountered.")
        return 1


if __name__ == '__main__':
    main(sys.argv[1:])


