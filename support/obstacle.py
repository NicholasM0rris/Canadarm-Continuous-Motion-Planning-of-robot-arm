

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

