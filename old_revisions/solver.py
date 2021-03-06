"""
****************************************************************************
* Canadarm - robot arm motion planner                                      *
* Author: Nick Morris                                                      *
*                                                                          *
* Useful link on PRM                                                       *
* http://www.cs.columbia.edu/~allen/F15/NOTES/Probabilisticpath.pdf        *
****************************************************************************
"""
from copy import deepcopy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import time
from tester import test_config_equality, test_bounding_box, test_line_collision
import copy
import sys
import robot_config
from robot_config import make_robot_config_from_ee1, make_robot_config_from_ee2
from obstacle import Obstacle
from angle import Angle
import random

# Change to anything to force a solution
# TODO Change to command line argument
FORCE = None


class Obstacle:
    """
    Class representing a rectangular obstacle.
    """

    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        assert x1 < x2, "For a valid obstacle, mush have x1 < x2"
        assert y1 < y2, "For a valid obstacle, mush have y1 < y2"

        self.corners = [(x1, y1), (x1, y2), (x2, y2), (x2, y1)]
        self.edges = [(self.corners[i], self.corners[(i + 1) % 4]) for i in range(4)]
        self.x_length = abs(x1 - x2)
        self.y_length = abs(y1 - y2)
        self.area = self.x_length * self.y_length


class ProblemSpec:
    """
    Class representing a planning problem. You may add to this class if you wish, but you should not modify the existing
    functions or variable names.

    COMP3702 2019 Assignment 2 Support Code

    Last updated by njc 24/08/19
    """

    # max allowable error for floating point comparisons
    TOLERANCE = 1e-5

    # max primitive step size
    PRIMITIVE_STEP = 1e-3

    def __init__(self, input_file):
        # parse input file
        f = open(input_file, 'r')

        # parse arm constraints
        try:
            self.num_segments = int(next_valid_line(f))
        except Exception:
            print("Invalid value for number of segments")
            sys.exit(1)
        self.min_lengths = [float(i) for i in next_valid_line(f).split(' ')]
        assert len(self.min_lengths) == self.num_segments, \
            "Number of minimum lengths does not match number of segments"
        self.max_lengths = [float(i) for i in next_valid_line(f).split(' ')]
        assert len(self.max_lengths) == self.num_segments, \
            "Number of maximum lengths does not match number of segments"

        # parse initial configuration
        initial_grappled = int(next_valid_line(f))
        assert initial_grappled == 1 or initial_grappled == 2, "Initial end effector number is not 1 or 2"
        try:
            initial_eex, initial_eey = [float(i) for i in next_valid_line(f).split(' ')]
        except Exception:
            print("Invalid value(s) for initial end effector position")
            sys.exit(1)
        initial_angles = [Angle(degrees=float(i)) for i in next_valid_line(f).split(' ')]
        assert len(initial_angles) == self.num_segments, \
            "Number of initial angles does not match number of segments"
        initial_lengths = [float(i) for i in next_valid_line(f).split(' ')]
        assert len(initial_lengths) == self.num_segments, \
            "Number of initial lengths does not match number of segments"
        if initial_grappled == 1:
            self.initial = make_robot_config_from_ee1(initial_eex, initial_eey, initial_angles, initial_lengths,
                                                      ee1_grappled=True)
        else:
            self.initial = make_robot_config_from_ee2(initial_eex, initial_eey, initial_angles, initial_lengths,
                                                      ee2_grappled=True)
        # parse goal configuration
        goal_grappled = int(next_valid_line(f))
        assert goal_grappled == 1 or goal_grappled == 2, "Goal end effector number is not 1 or 2"
        try:
            goal_eex, goal_eey = [float(i) for i in next_valid_line(f).split(' ')]
        except Exception:
            print("Invalid value(s) for goal end effector 1 position")
            sys.exit(1)
        goal_angles = [Angle(degrees=float(i)) for i in next_valid_line(f).split(' ')]
        assert len(goal_angles) == self.num_segments, \
            "Number of goal ee1 angles does not match number of segments"
        goal_lengths = [float(i) for i in next_valid_line(f).split(' ')]
        assert len(goal_lengths) == self.num_segments, \
            "Number of goal lengths does not match number of segments"
        if goal_grappled == 1:
            self.goal = make_robot_config_from_ee1(goal_eex, goal_eey, goal_angles, goal_lengths, ee1_grappled=True)
        else:
            self.goal = make_robot_config_from_ee2(goal_eex, goal_eey, goal_angles, goal_lengths, ee2_grappled=True)

        # parse grapple points
        try:
            self.num_grapple_points = int(next_valid_line(f))
        except Exception:
            print("Invalid value for number of grapple points")
            sys.exit(1)
        grapple_points = []
        for i in range(self.num_grapple_points):
            try:
                grapple_points.append(tuple([float(i) for i in next_valid_line(f).split(' ')]))
            except Exception:
                print("Invalid value(s) for grapple point " + str(i) + " position")
                sys.exit(1)
        self.grapple_points = grapple_points

        # parse obstacles
        try:
            self.num_obstacles = int(next_valid_line(f))
        except Exception:
            print("Invalid value for number of obstacles")
            sys.exit(1)
        obstacles = []
        for i in range(self.num_obstacles):
            try:
                x1, y1, x2, y2 = [float(i) for i in next_valid_line(f).split(' ')]
                obstacles.append(Obstacle(x1, y1, x2, y2))
            except Exception:
                print("Invalid value(s) for obstacle " + str(i))
                sys.exit(1)
        self.obstacles = obstacles
        self.goal_x = goal_eex
        self.goal_y = goal_eey
        self.goal_angles = []
        for i in range(len(goal_angles)):
            self.goal_angles.append(in_degrees(goal_angles[i].radians))
        self.initial_angles = []
        for i in range(len(initial_angles)):
            self.initial_angles.append(in_degrees(initial_angles[i].radians))


def next_valid_line(f):
    # skip comments and empty lines, return None on EOF
    while True:
        line = f.readline()
        if len(line) == 0:
            return None
        if len(line) > 1 and line[0] != '#':
            return line.strip()


def in_obstacle(obstacles, node):
    for obstacle in obstacles:

        tolerance = 0.01
        x1 = obstacle.x1 - tolerance
        x2 = obstacle.x2 + tolerance
        y1 = obstacle.y1 - tolerance
        y2 = obstacle.y2 + tolerance
        x = node.point[0]
        y = node.point[1]
        b = (x1 <= x <= x2) and (y1 <= y <= y2)
        if b:
            return True

    return False


def get_distance(point1, point2):
    """ Get the Euclidean distance between two points """
    point1 = list(point1)
    point2 = list(point2)
    distance = math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
    return distance


def line_segment_collision(node1, node2, obstacle_corner1, obstacle_corner2):
    """ Check whether there is a intersection between two line segments
    :param node1: A node class object
    :param node2: A node class object
    :param obstacle_corner1: A tuple containing x and y coordinates for obstacle corner
    :param obstacle_corner2: A tuple containing x and y coordinates for obstacle corner
    """
    obstacle_corner1 = list(obstacle_corner1)
    obstacle_corner2 = list(obstacle_corner2)
    a = [node1.x, node1.y]
    b = [node2.x, node2.y]
    c = obstacle_corner1
    d = obstacle_corner2
    area_abc = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])
    area_abd = (b[0] - a[0]) * (d[1] - a[1]) - (d[0] - a[0]) * (b[1] - a[1])
    area_cda = (d[0] - c[0]) * (a[1] - c[1]) - (a[0] - c[0]) * (d[1] - c[1])
    area_cdb = (d[0] - c[0]) * (b[1] - c[1]) - (b[0] - c[0]) * (d[1] - c[1])
    # If abc and abd have different orientations AND cda and cbd have different orientations, there is a line collision between ab and cd
    if (np.sign(area_abc) != np.sign(area_abd)) and (np.sign(area_cda) != np.sign(area_cdb)):
        return True
    # There is no line collision
    return False


def test_obstacle_collision(config, spec, obstacles):
    # return true for pass, false for fail
    for i in range(spec.num_segments):
        p = config.points[i]
        q = config.points[i + 1]
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


class Node:

    def __init__(self, x, y, node_id, angle, x2=None, y2=None, length=None):
        self.point = (x, y)
        self.x = x
        self.y = y
        self.length = length
        self.node_id = node_id
        self.neighbours = []
        self.angles = angle
        self.configuration = [x, y]
        try:
            self.configuration.extend(angle)
        except TypeError:
            pass
        try:
            self.configuration.extend(length)
        except TypeError:
            pass
        try:
            # print(x2, y2)
            if x2 and y2 is not None:
                self.grappled_point = [x2, y2]
                self.configuration.extend(self.grappled_point)
                # print('yes')
        except TypeError:
            pass

        # self.parent = parent
        # self.configuration.append(self.parent)

    def get_successors(self):
        """ Returns a nodes neighbours
            :param node: A Node class object
        """
        return self.neighbours


class PRM:

    def __init__(self, x_max, x_min, y_max, y_min, N, obstacle, grapple_points, goal_x, goal_y, initial_x, initial_y,
                 min_lengths, max_lengths, initial_ee1, goal_angles, initial_angles, spec):
        self.x_max = x_max

        self.y_max = y_max

        self.x_min = x_min

        self.y_min = y_min

        self.num_nodes = N

        self.nodes = []

        self.obstacle = obstacle

        self.current_grapple = list(grapple_points[0])

        self.goal_grapple = [spec.goal_x, spec.goal_y]

        self.goal_length = spec.goal.lengths

        self.goal_angles = goal_angles

        self.initial_angles = initial_angles

        self.initial_ee1_x = initial_ee1[0]

        self.initial_ee1_y = initial_ee1[1]

        self.initial_length = spec.initial.lengths

        self.grapple = grapple_points

        self.min_lengths = min_lengths

        self.max_lengths = max_lengths

        self.num_links = len(min_lengths)

        self.initial_x = initial_x

        self.initial_y = initial_y

        self.goal = [goal_x, goal_y]

        self.spec = spec

        self.num_grapples = len(self.grapple)

        self.iteration = 0

        """ Add the goal as a node - Always has ID of 0 """
        if self.num_grapples % 2 == 0:
            goal = list(spec.goal.get_ee1())
            self.nodes.append(
                Node(goal[0], goal[1], 0, self.goal_angles, x2=self.goal_grapple[0], y2=self.goal_grapple[1],
                     length=self.goal_length))
        else:
            self.nodes.append(
                Node(goal_x, goal_y, 0, self.goal_angles, x2=self.goal_grapple[0], y2=self.goal_grapple[1],
                     length=self.goal_length))

        # print(goal_x, goal_y, self.goal_angles)
        print('goal node', self.nodes[0].configuration)

        """ Add initial EE2 as node - Always had ID of 1 """
        self.nodes.append(Node(initial_x, initial_y, 1, self.initial_angles, self.initial_ee1_x, self.initial_ee1_y,
                               self.initial_length))

        ''' Add grapple points as nodes - ID always 2 - num. grapple pts '''
        if self.num_grapples > 1:
            grapple_configuration = self.bridge_me_bb(self.grapple[0], self.grapple[1])
            self.nodes.append(
                Node(grapple_configuration[0], grapple_configuration[1], 2, grapple_configuration[2:2 + self.num_links],
                     grapple_configuration[-2], grapple_configuration[-1],
                     grapple_configuration[2 + self.num_links:-2]))
        '''
        for i in range(len(grapple_points) - 1):
            print(grapple_points[i + 1])
            self.nodes.append(Node(grapple_points[i + 1][0], grapple_points[i + 1][1], i + 2, 1))
        '''
        # print(self.nodes[0])

        # print(goal_x, goal_y)

        # self.nodes.append(Node(1,1,1))
        self.num_grapples = len(grapple_points)

    def test_self_collision(self, config, spec):
        # return true for pass, false for fail
        if spec.num_segments < 3:
            # collision impossible with less than 3 segments
            return True
        # do full check
        for i in range(spec.num_segments - 1):
            p1 = config.points[i]
            q1 = config.points[i + 1]

            for j in range(i + 2, spec.num_segments):
                p2 = config.points[j]
                q2 = config.points[j + 1]

                if test_line_collision((p1, q1), (p2, q2)):
                    return False

        return True

    def generate_random_points(self):
        """ Generate N random sample points"""
        print('grapple', self.grapple)
        for i in range(self.num_grapples):
            total = 0
            grapple = list(self.grapple[i])
            print('n_gple', grapple)
            while total < round(self.num_nodes / self.num_grapples):
                angles = [random.randrange(180)]
                # print('annnngles', angles, type(angles))
                angles.extend(list(np.random.uniform(-165, 165, self.num_links - 1)))
                radian_angles = deepcopy(angles)
                for i in range(len(angles)):
                    radian_angles[i] = in_radians(radian_angles[i])

                # print(angles)
                lengths = np.random.uniform(self.min_lengths[0], self.max_lengths[0], self.num_links)

                # print('angles',angles)
                # print(self.initial_x)
                if i % 2 == 0:
                    config = robot_config.make_robot_config_from_ee1(lengths=lengths, angles=radian_angles,
                                                                     x=grapple[0],
                                                                     y=grapple[1], ee1_grappled=True)
                else:
                    config = robot_config.make_robot_config_from_ee1(lengths=lengths, angles=radian_angles,
                                                                     x=grapple[0],
                                                                     y=grapple[1], ee2_grappled=True)
                # print(self.initial_ee1_x, self.initial_ee1_y)
                # print(self.initial_ee1_x, self.initial_ee1_y)
                point = list(config.get_ee2())
                # print(point)
                # print(point)
                p = Node(point[0], point[1], total + self.num_grapples + 2, angles, self.current_grapple[0],
                         self.current_grapple[1], lengths)
                '''
                p = Node(np.random.uniform(self.x_min, self.x_max, 1)[0],
    
                         np.random.uniform(self.y_min, self.y_max, 1)[0],
    
                         total + self.num_grapples + 2, angles)
                '''
                # print(p.point)
                if not in_obstacle(self.obstacle, p) and self.inside_world(p) and test_obstacle_collision(config,
                                                                                                          self.spec,
                                                                                                          self.obstacle) and self.test_self_collision(
                    config, self.spec):
                    self.nodes.append(p)
                    # print(point)
                    # print(angles)
                    # print(p.configuration)

                    # Indent line below to ensure N points generated
                    total += 1
        # print(total)
        print('succesful no. nodes: ', len(self.nodes))

    def inside_world(self, node):
        """ Check if node point is inside the boundary conditions
            :param node: A node class object
        """
        # print('x max =', self.x_max, 'x_min = ', self.x_min)
        if (self.x_min <= node.x <= self.x_max) and (self.y_min <= node.y <= self.y_max):
            return True
        else:
            return False

    def inside_world_point(self, point):
        """ Check if node point is inside the boundary conditions
            :param point: a list of x and y coordinate
        """
        # print('x max =', self.x_max, 'x_min = ', self.x_min)
        if (self.x_min <= point[0] <= self.x_max) and (self.y_min <= point[1] <= self.y_max):
            return True
        else:
            return False

    def get_k_neighbours(self):
        """ Pair q (every sample point) to k nearest neighbours (q') """
        # Define number of neighbours k here
        k_max = 12
        # For every node point get the edge distances that don't intersect obstacles
        for i in self.nodes:
            distances = []
            for j in self.nodes:
                # Check not comparing the same point to itself and edge does not intersect obstacle
                if (i.node_id != j.node_id) and not (self.obstacle_collision(i, j, self.obstacle)):
                    # Add distance between points if no collision between interpolation
                    # ('attempt')
                    if not self.interpolate_collision_check(i, j, self.num_links):
                        # print('success')
                        distances.append((get_distance(i.point, j.point), j))
            # Sort from smallest to largest distance
            distances = sorted(distances, key=lambda d: d[0])
            # for each set of points add the k nearest neighbours
            k = 0
            for points in distances:
                if k >= k_max:
                    break
                # print('POINTS', points[1])
                i.neighbours.append(points[1])
                # NB! This if statement forces the neighbours of the goal node to include the goal node as a neighbour
                if FORCE is True:
                    if i == self.nodes[0] and not (self.obstacle_collision(i, j, self.obstacle)):
                        j.neighbours.append(i)
                else:
                    pass
                k += 1
        # print('NEIGHBOURS', i.neighbours[0].point)

    def interpolate_collision_check(self, node1, node2, num_links):
        """

        :param node1: first node
        :param node2: second node
        :param num_links: number of links of robot arm
        :param spec: problem spec - used for obstacle detection
        :return: True for collision between interpolations, False for no collision
        """
        ee1 = node1.configuration[-2:]
        node1_configuration = node1.configuration
        node2_configuration = node2.configuration
        configs = [node1_configuration, node2_configuration]
        robot_configuration = []

        for i in range(len(configs)):
            # Remove coord points for now
            # print(type(robot_config))
            robot_configuration.append(configs[i][2:-2])
            # Convert the angles to radians
            for j in range(num_links):
                robot_configuration[i][j] = in_radians(robot_configuration[i][j])

        # Get the configurations
        c1 = np.array(robot_configuration[0])
        c2 = np.array(robot_configuration[1])
        # Get the diff
        diff = c2 - c1
        # Get the max diff
        max_diff = max(abs(diff))
        # Get the number steps required
        n_steps = math.ceil(max_diff / 0.05)
        # Find delta for each step
        delta = diff / n_steps
        # Replace nans with 0
        delta[np.isnan(delta)] = 0
        # Deep copy first config
        ci = copy.deepcopy(c1)
        for step_number in range(n_steps):
            # Iterate over every angle and length
            ci = c1 + (step_number * delta)
            # Create robot config for new ci
            lengths = ci[- num_links:]
            angles = ci[:num_links]
            config = robot_config.make_robot_config_from_ee1(lengths=lengths, angles=angles,
                                                             x=ee1[0],
                                                             y=ee1[1], ee1_grappled=True)
            ee2 = list(config.get_ee2())
            # If there is a collision return True
            if not test_obstacle_collision(config, self.spec, self.obstacle) or not self.test_self_collision(config,
                                                                                                             self.spec) or not self.inside_world_point(
                ee2):
                return True
        # There is no collision between the interpolation
        return False

    def obstacle_collision(self, node1, node2, obstacles):
        """ Check the line segment between two nodes and check if it collides with any four segments that make up a rectangle obstacle
            :param node1: A node point containing x and y coordinate
            :param node2: A node point containing x and y coordinate
            :param obstacles: A list of obstacle class objects
            e.g obstacles[0].edges [((0.2, 0.0), (0.2, 0.095)), ((0.2, 0.095), (1.0, 0.095)), ((1.0, 0.095), (1.0, 0.0)), ((1.0, 0.0), (0.2, 0.0))]
        """
        # Check line collisions
        for i in range(len(obstacles)):
            for j in range(len(obstacles[i].edges)):
                if line_segment_collision(node1, node2, obstacles[i].edges[j][0], obstacles[i].edges[j][1]):
                    return True

        return False

    def bridge_me_bb(self, grapple1, grapple2):
        """
        Finds and returns the configuration to create a bridge between two grapple points
        :param grapple1: list of grapple point coords [x,y]
        :param grapple2: list of grapple point coords [x,y]
        :param joint_N1: coords of N-1 arm joint [x,y]
        :param joint_N2: coords of N-2 arm joint [x,y]
        :param num_links: number of links
        :param length_n: length of N-1 arm link
        :param ee1: coords of ee1 [x,y]
        :return: a configuration list [ee2x, ee2y, a1, a2 ... an, l1, l2 ... ln, ee1x, ee2y]
        """
        while True:
            configuration = [grapple2[0], grapple2[1]]
            # Create random angles and lengths for N - 1 links
            # Angles are in radians
            angles = list(np.random.uniform(0, np.pi, 1))
            # print(angles)
            angles.extend(list(np.random.uniform(-2.87979, 2.87979, self.num_links - 2)))
            # print(self.min_lengths, self.max_lengths, self.num_links - 1)
            lengths = list(np.random.uniform(self.min_lengths[0], self.max_lengths[0], self.num_links - 1))

            config = make_robot_config_from_ee1(grapple1[0], grapple1[1], angles, lengths, ee1_grappled=True)
            joint_n1 = config.points[-1]
            joint_n2 = config.points[-2]
            length_n = lengths[-1]
            # Calculate parameters
            b = length_n
            a = np.sqrt((grapple2[0] - joint_n1[0]) ** 2 + (grapple2[1] - joint_n1[1]) ** 2)
            c = np.sqrt((grapple2[0] - joint_n2[0]) ** 2 + (grapple2[1] - joint_n2[1]) ** 2)
            c_angle = np.arccos((a ** 2 + b ** 2 - c ** 2) / (2 * a * b))
            a_nangle = -(np.pi - c_angle)
            # If parameters satisfy conditions create a configuration from them and return it
            angles.append(a_nangle)
            lengths.append(a)
            config = make_robot_config_from_ee1(grapple1[0], grapple1[1], angles, lengths, ee1_grappled=True,
                                                ee2_grappled=True)

            if self.min_lengths[0] <= a <= self.max_lengths[
                0] and -2.87979 <= a_nangle <= 2.87979 and test_obstacle_collision(config, self.spec,
                                                                                   self.obstacle) and self.test_self_collision(
                config, self.spec):
                for i in range(len(angles)):
                    angles[i] = in_degrees(angles[i])
                print(config.get_ee1(), config.get_ee2())
                configuration.extend(angles)
                configuration.extend(lengths)
                configuration.append(grapple1[0])
                configuration.append(grapple1[1])
                print('config of bridge!!!!!', configuration)
                return configuration

    def solve(self):
        """
        Solve method containing code to perform a breadth first search of the state graph and return a list of
        configs which form a path through the state graph between the initial and the goal.

        :return: List of configs forming a path through the graph from initial to goal
        """
        # Set initial and goal nodes
        if self.current_grapple == self.goal_grapple:
            init_node = self.nodes[1]
            goal_node = self.nodes[0]
        else:
            init_node = self.nodes[1 + self.iteration]
            # Set the goal node to be another grapple node
            goal_node = self.nodes[2 + self.iteration]
        # print("Initial goal node", goal_node.configuration)

        # search the graph
        init_container = [init_node]

        # here, each key is a graph node, each value is the list of configs visited on the path to the graph node
        # init_visited = [init_node.configuration]
        init_visited = {init_node: [init_node.configuration]}
        while len(init_container) > 0:

            current = init_container.pop(0)
            # current_copy = deepcopy(current)
            if current.configuration == goal_node.configuration:  # and goal_node == self.nodes[0]:
                # found path to goal
                # print('found a path to: ', init_visited[-1])

                print('current: ', current.configuration)
                print('init', init_node.configuration)
                print('goal: ', goal_node.configuration)
                # print('parent', current.parent.configuration)
                # list = self.done(current, init_node)
                return init_visited[current]
            elif current.configuration == goal_node.configuration:
                goal_node = self.nodes[0]

            successors = current.get_successors()
            # print('current', current.configuration)
            for suc in successors:

                # print('a', suc.configuration, suc not in init_visited, suc.configuration == goal_node.configuration)
                if suc not in init_visited:
                    # suc.parent = current
                    # suc.configuration[-1] = current
                    init_container.append(suc)
                    init_visited[suc] = init_visited[current] + [suc.configuration]
        print('failed to find a path from solve function', init_container)


def interpolate(robot_configuration, num_links):
    new_robot_config_list = []

    robot_config = []
    copy_robot_config = copy.deepcopy(robot_configuration)
    for i in range(len(copy_robot_config)):
        # Remove coord points for now
        # print(type(robot_config))
        robot_config.append(copy_robot_config[i][2:-2])
        # Convert the angles to radians
        for j in range(num_links):
            robot_config[i][j] = in_radians(robot_config[i][j])
            # print('should be radians', robot_config[i])
    print('robot_config', robot_config)
    # For each node
    for i in range(len(robot_config) - 1):
        # print('len', len(robot_config))

        c1 = np.array(robot_config[i])
        c2 = np.array(robot_config[i + 1])
        # print("c1c2", c1, c2)
        diff = c2 - c1
        # print('c1', c1)
        # print('diff', diff)
        max_diff = max(abs(diff))
        n_steps = math.ceil(max_diff / 0.001)
        # print('steps', n_steps)
        delta = diff / n_steps
        # print('delta', delta)
        # Remove nans and replace for 0
        delta[np.isnan(delta)] = 0
        # print('new delta', delta)
        # n_steps = n_steps.astype(int)
        # print('rounded', n_steps)
        ci = copy.deepcopy(c1)
        # print('c1 here',c1)

        # print("links", num_links*2)
        # Iterate over every step
        for step_number in range(n_steps):
            # Iterate over every angle and length
            # print("NUM STEPS!!!!", n_steps[j])
            ci = c1 + (step_number * delta)
            # print('c1__', c1)
            # print('c1[j], [k],  delta[j], k*delta[j])', j, c1[j], [k], delta[j], k * delta[j])
            '''
            print('j=', j, 'k=', k)
            print('ci[j]',ci[j])
            print('delta', delta[j])
            print('c1', c1[j], 'c2', c2[j])
            print(c1, c2)
            print(robot_config[-2], robot_config[-1])
            '''
            # Convert back to degrees
            # for angle in range(num_links):
            # ci[angle] = in_degrees(ci[angle])
            # ci.insert(0, robot_configuration[i][1])
            # ci.insert(0, robot_configuration[i][0])
            ci = list(ci)
            ci.append(robot_configuration[i][-2])
            ci.append(robot_configuration[i][-1])
            # print('new_ci', ci)

            new_robot_config_list.append(ci)
    c2 = list(c2)
    c2.append(robot_configuration[i][-2])
    c2.append(robot_configuration[i][-1])
    new_robot_config_list.append(c2)
    print(new_robot_config_list[-2])
    print(new_robot_config_list[-1])
    # print(new_robot_config_list[-5])  # <- DO NOT Uncomment           Unless you want a BAD TIME... IT'S HUGE
    print(len(new_robot_config_list))

    return new_robot_config_list


def in_degrees(radians):
    return radians * 180 / math.pi


def in_radians(degrees):
    return degrees * math.pi / 180


def write_robot_config_list_to_file(filename, robot_config_list, num_links):
    """
    Write an output file for the given list of RobotConfigs. We recommend using this method to generate your output
    file.
    :param filename: name of output file (e.g. 2nd argument given to program)
    :param robot_config_list: list of RobotConfig objects forming a path
    :param num_links: The number of links for the robot arm
    """

    f = open(filename, 'w')
    j = 0
    for rc in range(len(robot_config_list)):
        #  print(robot_config_list)
        lengths = []
        angles = []
        # Create robot config to get EE1 positions
        for i in range(num_links):
            lengths.append(robot_config_list[rc][i + num_links])
            angles.append(in_degrees(robot_config_list[rc][i]))
        '''
        config = robot_config.make_robot_config_from_ee2(lengths=lengths, angles=angles,
                                                         x=robot_config_list[rc][0],
                                                         y=robot_config_list[rc][0])
        ee1 = list(config.get_ee1())
        '''
        ee1 = [robot_config_list[rc][-2], robot_config_list[rc][-1]]
        # print(ee1)
        ee1 = str(ee1).replace(",", "")
        angles = str(angles).replace(",", "")
        lengths = str(lengths).replace(",", "")
        # print('lengths', lengths)
        # print('angles', angles)

        f.write("{0}; {1}; {2}\n".format(str(ee1)[1:-1], str(angles)[1:-1], str(lengths)[1:-1]))

    f.close()


def main(arglist):
    start = time.time()
    # print(np.random.uniform(0, 100, size=10))
    print(arglist)
    testcase = arglist[0]
    problem_spec = ProblemSpec(testcase)

    # print(problem_spec.obstacles[0].corners)
    bottom_left_corner = []
    x_length = []
    y_length = []
    for j in range(len(problem_spec.obstacles)):
        bottom_left_corner.append(problem_spec.obstacles[j].corners[0])
    # print(bottom_left_corner)
    for j in range(len(problem_spec.obstacles)):
        x_length.append(problem_spec.obstacles[j].x_length)
        y_length.append(problem_spec.obstacles[j].y_length)
    # print(problem_spec.obstacles[0].corners[0][0])
    # print('bc', bottom_left_corner[0][0])

    # Create a PRM with X min, X max, Y min, X max, N samples, Obstacles, Grapple points, EE2 goal_x, EE2_goal_y, EE2_initial_x, EE2_initial_y, min_lengths, max_lengths, initial_ee1
    prm = PRM(1, 0, 1, 0, 100, problem_spec.obstacles, problem_spec.grapple_points, problem_spec.goal.get_ee2()[0],
              problem_spec.goal.get_ee2()[1], problem_spec.initial.get_ee2()[0], problem_spec.initial.get_ee2()[1],
              problem_spec.min_lengths, problem_spec.max_lengths, problem_spec.initial.get_ee1(),
              problem_spec.goal_angles, problem_spec.initial_angles, problem_spec)
    # print(problem_spec.obstacles)

    # print('goal', problem_spec.goal_x)
    # Generate random points for the prm
    prm.generate_random_points()
    # Get k neighbours for the random points
    prm.get_k_neighbours()
    # print('obs', in_obstacle(problem_spec.obstacles[0],(0.3,0.2)))
    path = prm.solve()
    print('configs to goal', path)
    new_path = interpolate(path, problem_spec.num_segments)
    # We need to write a list of primitive steps, extrapolating between the configurations
    # EE1x, EE1y; angle1, angle2 ... anglen; length1, length 2 ... length 3
    write_robot_config_list_to_file(arglist[1], new_path, problem_spec.num_segments)
    print('Time required = ', -start + time.time())
    '''
    for i in range(len(list)):
        print(list[i].configuration)
    '''
    k = 0
    x = []
    y = []
    for i in range(len(prm.nodes)):
        # print(prm.nodes[i].point)
        x.append(prm.nodes[i].point[0])
        y.append(prm.nodes[i].point[1])

    '''
    For testing purposes
    '''
    print('grapple points', problem_spec.grapple_points)
    print('Goal point', problem_spec.goal)
    print('Goal EE1', problem_spec.goal.get_ee1())
    print('Goal EE2', problem_spec.goal.get_ee2())
    print('Obstacles', problem_spec.obstacles[0].edges[0][0])
    print('lengths', problem_spec.min_lengths, problem_spec.max_lengths)
    test = robot_config.make_robot_config_from_ee1(lengths=[0.2, 0.2, 0.2], angles=[30, 30, 30],
                                                   x=problem_spec.initial.get_ee1()[0],
                                                   y=problem_spec.initial.get_ee1()[0])
    # print('Configuration: ', prm.nodes[9].configuration)
    print('Configuration goal: ', prm.nodes[0].configuration)
    # print('goal angles prm', problem_spec.goal_angles)
    # p1 = [0, 0]
    # p2 = [2, 2]
    # dist = math.sqrt( (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 )
    # Take away from the tests below is that key=lambda lets you control which part is used for sorting
    # t = ((5,99), (6,20), (4,55), (3,200000))
    # test = sorted(t, key=lambda x:x[1])
    # print(test)
    # print(dist)
    ''''''

    fig = plt.figure(testcase)
    ax = fig.add_subplot(1, 1, 1)
    ''' Plot the obstacles'''
    for i in range(len(problem_spec.obstacles)):
        ax.add_patch(patches.Rectangle(bottom_left_corner[i], x_length[i], y_length[i], zorder=-1))
    ''' Plot the sampled points '''
    plt.scatter(x, y, color='red', s=30)

    ''' Plot the grapple points'''
    for i in range(len(problem_spec.grapple_points)):
        plt.scatter(problem_spec.grapple_points[i][0], problem_spec.grapple_points[i][1], color='blue', s=30)

    ''' Plot the goal point'''
    if prm.num_grapples % 2 == 0:
        print(prm.num_grapples)
        print('went through if')
        plt.scatter(problem_spec.goal.get_ee1()[0], problem_spec.goal.get_ee1()[1], color='green', s=100)
    else:
        print('else')
        plt.scatter(problem_spec.goal.get_ee2()[0], problem_spec.goal.get_ee2()[1], color='green', s=100)
    ''' Plot the start EE point'''
    plt.scatter(problem_spec.initial.get_ee2()[0], problem_spec.initial.get_ee2()[1], color='black', s=100)
    # plt.scatter(bottom_left_corner[0], bottom_left_corner[1], color='red', s=3000)
    # print(x_length)
    ''' Plot the potential paths between nodes'''
    for i in range(len(prm.nodes)):
        for j in range(len(prm.nodes[i].neighbours)):
            # print('SCATTER', prm.nodes[i].neighbours[j])
            plt.plot([prm.nodes[i].x, prm.nodes[i].neighbours[j].x], [prm.nodes[i].y, prm.nodes[i].neighbours[j].y],
                     'y-')
    # Plot the correct path
    try:
        for i in range(len(path) - 1):
            # print(path[i])
            plt.plot([path[i][0], path[i + 1][0]], [path[i][1], path[i + 1][1]],
                     'r-', zorder=3)
            counter = i
    except:
        pass
    # print(i)
    # plt.plot([x1, x2], [y1, y2], 'k-')
    plt.title(testcase)
    plt.xlabel('X coordinates')
    plt.ylabel('Y coordinates')

    plt.show()


if __name__ == '__main__':
    main(sys.argv[1:])
