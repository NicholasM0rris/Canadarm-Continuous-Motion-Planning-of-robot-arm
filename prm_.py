from copy import deepcopy

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import time
from tester import test_config_equality, test_bounding_box, test_line_collision

import sys
import robot_config
from robot_config import make_robot_config_from_ee1, make_robot_config_from_ee2
from obstacle import Obstacle
from angle import Angle

'''
Useful link on PRM
http://www.cs.columbia.edu/~allen/F15/NOTES/Probabilisticpath.pdf
'''

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

    def __init__(self, x, y, node_id, angle, length=None):
        self.point = (x, y)
        self.x = x
        self.y = y
        self.length = length
        self.node_id = node_id
        self.neighbours = []
        self.angles = angle
        self.configuration = [x, y]
        self.configuration.extend(angle)
        try:
            self.configuration.extend(length)
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

        """ Add the goal as a node - Always has ID of 0 """
        self.nodes.append(Node(goal_x, goal_y, 0, self.goal_angles, self.goal_length))
        # print(goal_x, goal_y, self.goal_angles)
        print('goal node', self.nodes[0].configuration)

        """ Add initial EE2 as node - Always had ID of 1 """
        self.nodes.append(Node(initial_x, initial_y, 1, self.initial_angles, self.initial_length))

        ''' Add grapple points as nodes - ID always 1 - num. grapple pts '''
        for i in range(len(grapple_points) - 1):
            self.nodes.append(Node(grapple_points[i + 1][0], grapple_points[i + 1][1], i + 2, 1))

        # print(self.nodes[0])

        # print(goal_x, goal_y)

        # self.nodes.append(Node(1,1,1))
        self.num_grapples = len(grapple_points)

    def generate_random_points(self):
        """ Generate N random sample points"""
        total = 0

        while total < self.num_nodes:

            angles = np.random.uniform(0, 180, self.num_links)
            lengths = np.random.uniform(self.min_lengths, self.max_lengths, self.num_links)

            # print(angles)
            # print(self.initial_x)
            config = robot_config.make_robot_config_from_ee1(lengths=lengths, angles=angles,
                                                             x=self.initial_ee1_x,
                                                             y=self.initial_ee1_y)
            # print(self.initial_ee1_x, self.initial_ee1_y)
            point = list(config.get_ee2())
            # print(point)
            p = Node(point[0], point[1], total + self.num_grapples + 2, angles, lengths)
            '''
            p = Node(np.random.uniform(self.x_min, self.x_max, 1)[0],

                     np.random.uniform(self.y_min, self.y_max, 1)[0],

                     total + self.num_grapples + 2, angles)
            '''
            # print(p.point)
            if not in_obstacle(self.obstacle, p) and self.inside_world(p) and test_obstacle_collision(config, self.spec,
                                                                                                      self.obstacle):
                self.nodes.append(p)
                print(p.configuration)

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

    def get_k_neighbours(self):
        """ Pair q (every sample point) to k nearest neighbours (q') """
        # Define number of neighbours k here
        k_max = 20
        # For every node point get the edge distances that don't intersect obstacles
        for i in self.nodes:
            distances = []
            for j in self.nodes:
                # Check not comparing the same point to itself and edge does not intersect obstacle
                if (i.node_id != j.node_id) and not (self.obstacle_collision(i, j, self.obstacle)):
                    # Add distance between points
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

    def done(self, current_node, initial_node):

        """ The purpose of this function  is: Trace back this node to the founding granpa.

        Print out the states through out

        """

        founding_father = current_node

        visited_nodes = []  # the retraced states will be stored here.

        counter = 0

        limit = 50  # if the trace is longer than 50, don't print anything, it will be a mess.
        # print('doen init', initial_node.configuration)
        while founding_father:  # != initial_node:
            visited_nodes.append(founding_father)

            founding_father = founding_father.parent

            counter += 1
            if counter > 100:
                break
            # Keep doing this until you reach the founding father that has a parent None (see default of init method)

        print('Number of nodes traveled to the goal = ', counter - 1)

        return visited_nodes

    def solve(self):
        """
        Solve method containing code to perform a breadth first search of the state graph and return a list of
        configs which form a path through the state graph between the initial and the goal. Note that this path will not
        satisfy the primitive step requirement - you will need to interpolate between the configs in the returned list.


        :param spec: ProblemSpec object
        :return: List of configs forming a path through the graph from initial to goal
        """
        # Set initial and goal nodes
        init_node = self.nodes[1]
        goal_node = self.nodes[0]
        # print("Initial goal node", goal_node.configuration)

        # search the graph
        init_container = [init_node]

        # here, each key is a graph node, each value is the list of configs visited on the path to the graph node
        # init_visited = [init_node.configuration]
        init_visited = {init_node: [init_node.configuration]}
        while len(init_container) > 0:

            current = init_container.pop(0)
            # current_copy = deepcopy(current)
            if current.configuration == goal_node.configuration:
                # found path to goal
                # print('found a path to: ', init_visited[-1])
                print('current: ', current.configuration)
                print('init', init_node.configuration)
                print('goal: ', goal_node.configuration)
                # print('parent', current.parent.configuration)
                # list = self.done(current, init_node)
                return init_visited[current]

            successors = current.get_successors()
            print('current', current.configuration)
            for suc in successors:

                # print('a', suc.configuration, suc not in init_visited, suc.configuration == goal_node.configuration)
                if suc not in init_visited:
                    # suc.parent = current
                    # suc.configuration[-1] = current
                    init_container.append(suc)
                    init_visited[suc] = init_visited[current] + [suc.configuration]
        print('failed to find a path from solve function', init_container)


def in_degrees(radians):
    return radians * 180 / math.pi


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

        lengths = []
        angles = []
        # Create robot config to get EE1 positions
        for i in range(num_links):
            lengths.append(robot_config_list[rc][i + 2 + num_links])
            angles.append(robot_config_list[rc][i + 2])
        config = robot_config.make_robot_config_from_ee2(lengths=lengths, angles=angles,
                                                         x=robot_config_list[rc][0],
                                                         y=robot_config_list[rc][0])
        ee1 = list(config.get_ee1())
        f.write("{0}; {1}; {2}\n".format(str(ee1)[1:-1], str(angles)[1:-1], str(lengths)[1:-1]))
    f.close()


def main():
    start = time.time()
    # print(np.random.uniform(0, 100, size=10))
    testcase = 'testcases/4g1_m1.txt'
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
    prm = PRM(1, 0, 1, 0, 200, problem_spec.obstacles, problem_spec.grapple_points, problem_spec.goal.get_ee2()[0],
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
    list = prm.solve()
    print('configs to goal', list)
    # We need to write a list of primitive steps, extrapolating between the configurations
    # EE1x, EE1y; angle1, angle2 ... anglen; length1, length 2 ... length 3
    write_robot_config_list_to_file('output.txt', list, problem_spec.num_segments)
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
        for i in range(len(list) - 1):
            plt.plot([list[i][0], list[i + 1][0]], [list[i][1], list[i + 1][1]],
                     'r-', zorder=3)
            counter = i
    except:
        pass
    # print(i)
    # plt.plot([x1, x2], [y1, y2], 'k-')
    plt.title(testcase)
    plt.xlabel('X coordinates')
    plt.ylabel('Y coordinates')
    print('Time required = ', -start + time.time())
    plt.show()

# TODO Add arguments "input_file" "option for force" "Num_nodes" "K neighbours input" "output_file"
if __name__ == '__main__':
    main()