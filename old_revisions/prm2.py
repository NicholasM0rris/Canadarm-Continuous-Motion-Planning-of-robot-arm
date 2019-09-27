import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import time
from tester import test_config_equality

import sys
from robot_config import make_robot_config_from_ee1, make_robot_config_from_ee2
from obstacle import Obstacle
from angle import Angle


class Obstacle:
    """
    Class representing a rectangular obstacle. You may add to this class if you wish, but you should not modify the
    existing functions or variable names.

    COMP3702 2019 Assignment 2 Support Code

    Last updated by njc 24/08/19
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


class Node:

    def __init__(self, x, y, node_id):
        self.point = (x, y)
        self.x = x
        self.y = y
        self.node_id = node_id
        self.neighbours = []


class PRM:

    def __init__(self, x_max, x_min, y_max, y_min, N, obstacle, grapple_points, goal_x, goal_y, initial_x, initial_y):
        self.x_max = x_max

        self.y_max = y_max

        self.x_min = x_min

        self.y_min = y_min

        self.num_nodes = N

        self.nodes = []

        self.obstacle = obstacle

        self.grapple = grapple_points

        self.goal = [goal_x, goal_y]
        # Add grapple points as nodes - ID always 1 - num. grapple pts
        for i in range(len(grapple_points)):
            self.nodes.append(Node(grapple_points[i][0], grapple_points[i][1], i + 2))

        # Add the goal as a node - Always has ID of 0
        self.nodes.append(Node(goal_x, goal_y, 0))
        # Add initial EE2 as node - Always had ID of 1
        self.nodes.append(Node(initial_x, initial_y, 1))
        # print(goal_x, goal_y)

        # self.nodes.append(Node(1,1,1))
        self.num_grapples = len(grapple_points)

    def generate_random_points(self):
        """ Generate N random sample points"""
        total = 0

        while total < self.num_nodes:
            p = Node(np.random.uniform(self.x_min, self.x_max, 1)[0],

                     np.random.uniform(self.y_min, self.y_max, 1)[0],

                     total + self.num_grapples + 2)

            # if (not self.intersectsObs(p.point, p.point, obsVec) and self.isWithinWorld(p.point)):
            # print(p.point)
            if not in_obstacle(self.obstacle, p):
                self.nodes.append(p)

            # Indent line below to ensure N points generated
            total += 1
        # print(total)
        # print(len(self.nodes))

    def get_k_neighbours(self):
        """ Pair q (every sample point) to k nearest neighbours (q') """
        # Define number of neighbours k here
        k_max = 10
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
                k += 1
        # print('NEIGHBOURS', i.neighbours[0].point)

    def obstacle_collision(self, node1, node2, obstacles):
        """ Check the line segment between two nodes and check if it collides with any four segments that make up a rectangle obstacl
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


class GraphNode:
    """
    Class representing a node in the state graph. You should create an instance of this class each time you generate
    a sample.
    """

    def __init__(self, spec, config):
        """
        Create a new graph node object for the given config.

        Neighbors should be added by appending to self.neighbors after creating each new GraphNode.

        :param spec: ProblemSpec object
        :param config: the RobotConfig object to be stored in this node
        """
        self.spec = spec
        self.config = config
        self.neighbors = []

    def __eq__(self, other):
        return test_config_equality(self.config, other.config, self.spec)

    def __hash__(self):
        return hash(tuple(self.config.points))

    def get_successors(self):
        return self.neighbors


def solve(spec):
    """
    An example solve method containing code to perform a breadth first search of the state graph and return a list of
    configs which form a path through the state graph between the initial and the goal. Note that this path will not
    satisfy the primitive step requirement - you will need to interpolate between the configs in the returned list.

    If you wish to use this code, you may either copy this code into your own file or add your existing code to this
    file.

    :param spec: ProblemSpec object
    :return: List of configs forming a path through the graph from initial to goal
    """

    init_node = GraphNode(spec, spec.initial)
    goal_node = GraphNode(spec, spec.goal)

    # TODO: Insert your code to build the state graph here
    # *** example for adding neighbors ***
    # if path between n1 and n2 is collision free:
    #   n1.neighbors.append(n2)
    #   n2.neighbors.append(n1)

    # search the graph
    init_container = [init_node]

    # here, each key is a graph node, each value is the list of configs visited on the path to the graph node
    init_visited = {init_node: [init_node.config]}

    while len(init_container) > 0:
        current = init_container.pop(0)

        if test_config_equality(current.config, spec.goal, spec):
            # found path to goal
            return init_visited[current]

        successors = current.get_successors()
        for suc in successors:
            if suc not in init_visited:
                init_container.append(suc)
                init_visited[suc] = init_visited[current] + [suc.config]


def main():
    start = time.time()
    # print(np.random.uniform(0, 100, size=10))
    testcase = 'testcases/3g1_m0.txt'
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

    ''' For plotting the obstacles'''
    '''
    for j in range(len(problem_spec.obstacles)):
        # print('j', j)
        for i in range(len(problem_spec.obstacles[0].corners)):
            # print('i', i)
            if problem_spec.obstacles[j].corners[i][0] <= bottom_left_corner[j][0]:
                if problem_spec.obstacles[j].corners[i][1] <= bottom_left_corner[j][1]:
                    bottom_left_corner[j] = (problem_spec.obstacles[0].corners[i][0], problem_spec.obstacles[0].corners[i][1])
        #print('bottom left corner', bottom_left_corner)
    #print('bottom left corner', bottom_left_corner)
   
    x_ob = []
    y_ob = []
    for i in range(len(problem_spec.obstacles[0].corners)):
        x_ob.append(problem_spec.obstacles[0].corners[i][0])
        y_ob.append(problem_spec.obstacles[0].corners[i][1])
    print(x_ob)
    print(y_ob)
    '''
    # Create a PRM with X min, X max, Y min, X max, N samples, Obstacles, Grapple points, EE2 goal_x, EE2_goal_y, EE2_initial_x, EE2_initial_y
    prm = PRM(0, 1, 0, 1, 100, problem_spec.obstacles, problem_spec.grapple_points, problem_spec.goal.get_ee2()[0],
              problem_spec.goal.get_ee2()[1], problem_spec.initial.get_ee2()[0], problem_spec.initial.get_ee2()[1])
    # print(problem_spec.obstacles)
    # print('goal', problem_spec.goal_x)
    # Generate random points for the prm
    prm.generate_random_points()
    # Get k neighbours for the random points
    prm.get_k_neighbours()
    # print('obs', in_obstacle(problem_spec.obstacles[0],(0.3,0.2)))
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
    for i in range(len(prm.nodes)):
        for j in range(len(prm.nodes[i].neighbours)):
            # print('SCATTER', prm.nodes[i].neighbours[j])
            plt.plot([prm.nodes[i].x, prm.nodes[i].neighbours[j].x], [prm.nodes[i].y, prm.nodes[i].neighbours[j].y],
                     'y-')
    # plt.plot([x1, x2], [y1, y2], 'k-')
    plt.title(testcase)
    plt.xlabel('X coordinates')
    plt.ylabel('Y coordinates')
    print('Time required = ', -start + time.time())
    plt.show()


if __name__ == '__main__':
    main()
