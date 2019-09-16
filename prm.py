import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math

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


class Node:

    def __init__(self, x, y, node_id):
        self.point = (x, y)
        self.x = x
        self.y = y
        self.node_id = node_id
        self.neighbours = []


class PRM:

    def __init__(self, x_max, x_min, y_max, y_min, N, obstacle):
        self.x_max = x_max

        self.y_max = y_max

        self.x_min = x_min

        self.y_min = y_min

        self.num_nodes = N

        self.nodes = []

        self.obstacle = obstacle

    def generate_random_points(self):
        """ Generate N random sample points"""
        total = 0

        while total < self.num_nodes:
            p = Node(np.random.uniform(self.x_min, self.x_max, 1)[0],

                     np.random.uniform(self.y_min, self.y_max, 1)[0],

                     total + 2)

            # if (not self.intersectsObs(p.point, p.point, obsVec) and self.isWithinWorld(p.point)):
            # print(p.point)
            if not in_obstacle(self.obstacle, p):
                self.nodes.append(p)
            # Indent line below to ensure N points generated
            total += 1
        print(total)
        print(len(self.nodes))

    def get_k_neighbours(self):
        """ Pair q (every sample point) to k nearest neighbours (q') """
        # Define number of neighbours k here
        k_max = 10
        # For every node point get the edge distances that don't intersect obstacles
        for i in self.nodes:
            distances = []
            for j in self.nodes:
                # Check not comparing the same point to itself and edge does not intersect obstacle
                if i.node_id != j.node_id:
                    # Add distance between points
                    distances.append((self.get_distance(i.point, j.point), j))
            distances = sorted(distances)
            # for each set of points add the k nearest neighbours
            k = 0
            for points in distances:
                if k >= k_max:
                    break
                # print('POINTS', points[1])
                i.neighbours.append(points[1])
                k += 1
        #print('NEIGHBOURS', i.neighbours[0].point)

    def get_distance(self, point1, point2):
        """ Get the Euclidean distance between two points """
        point1 = list(point1)
        point2 = list(point2)
        distance = math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
        return distance


def main():
    # print(np.random.uniform(0, 100, size=10))

    problem_spec = ProblemSpec('testcases/4g3_m2.txt')
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

    prm = PRM(0, 1, 0, 1, 100, problem_spec.obstacles)
    print(problem_spec.obstacles)
    prm.generate_random_points()
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
    # p1 = [0, 0]
    # p2 = [2, 2]
    # dist = math.sqrt( (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 )
    # Take away from the tests below is that key=lambda lets you control which part is used for sorting
    # t = ((5,99), (6,20), (4,55), (3,200000))
    # test = sorted(t, key=lambda x:x[1])
    # print(test)

    # print(dist)
    ''''''

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ''' Plot the obstacles'''
    for i in range(len(problem_spec.obstacles)):
        ax.add_patch(patches.Rectangle(bottom_left_corner[i], x_length[i], y_length[i], zorder=-1))
    ''' Plot the sampled points '''
    plt.scatter(x, y, color='red', s=3)
    ''' Plot the grapple points'''
    for i in range(len(problem_spec.grapple_points)):
        plt.scatter(problem_spec.grapple_points[i][0], problem_spec.grapple_points[i][1], color='blue', s=30)
    ''' Plot the goal point'''
    plt.scatter(problem_spec.goal.get_ee2()[0], problem_spec.goal.get_ee2()[1], color='green', s=100)
    # plt.scatter(bottom_left_corner[0], bottom_left_corner[1], color='red', s=3000)
    # print(x_length)
    for i in range(len(prm.nodes)):
        for j in range(len(prm.nodes[i].neighbours)):
            # print('SCATTER', prm.nodes[i].neighbours[j])
            plt.plot([prm.nodes[i].x, prm.nodes[i].neighbours[j].x], [prm.nodes[i].y, prm.nodes[i].neighbours[j].y], 'r-')
    # plt.plot([x1, x2], [y1, y2], 'k-')
    plt.show()


if __name__ == '__main__':
    main()
