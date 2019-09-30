import math

class Angle:
    """
    Class representing an angle. Behaves like a normal floating point number, supporting addition, subtraction,
    multiplication by scalar, division by scalar, negation, equality and comparison. Constructor accepts degrees or
    radians, and value can be accessed as degrees or radians. Automatically keeps value in the range of -pi and pi.

    Provides static methods for sin, cos, tan, asin, acos, atan - call these from the class itself rather than an
    instance, e.g. Angle.sin(x) not x.sin()

    COMP3702 2019 Assignment 2 Support Code

    Last updated by njc 01/09/19
    """

    def __init__(self, radians=None, degrees=None):
        if radians is None:
            radians = degrees * math.pi / 180
        self.radians = (radians + math.pi) % (2 * math.pi) - math.pi  # get in range (-pi, pi)

    def in_radians(self):
        return self.radians

    def in_degrees(self):
        return self.radians * 180 / math.pi

    def __add__(self, other):
        if isinstance(other, Angle):
            radians = self.radians + other.radians
        elif isinstance(other, (int, float)):
            radians = self.radians + other
        else:
            raise Exception("+ operation between Angle and " + str(type) + " not supported.")
        # get in range (-pi, pi)
        if radians > 0:
            while radians > math.pi:
                radians -= 2 * math.pi
        else:
            while radians < -math.pi:
                radians += 2 * math.pi
        return Angle(radians=radians)

    def __sub__(self, other):
        if isinstance(other, Angle):
            radians = self.radians - other.radians
        elif isinstance(other, (int, float)):
            radians = self.radians - other
        else:
            raise Exception("- operation between Angle and " + str(type) + " not supported.")
        # get in range (-pi, pi)
        if radians > 0:
            while radians > math.pi:
                radians -= 2 * math.pi
        else:
            while radians < -math.pi:
                radians += 2 * math.pi
        return Angle(radians=radians)

    def __mul__(self, other):
        if isinstance(other, (int, float)):
            radians = self.radians * other
        else:
            raise Exception("* operation between Angle and " + str(type) + " not supported.")
        # get in range (-pi, pi)
        if radians > 0:
            while radians > math.pi:
                radians -= 2 * math.pi
        else:
            while radians < -math.pi:
                radians += 2 * math.pi
        return Angle(radians=radians)

    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            radians = self.radians / other
        else:
            raise Exception("/ operation between Angle and " + str(type) + " not supported.")
        # get in range (-pi, pi)
        if radians > 0:
            while radians > math.pi:
                radians -= 2 * math.pi
        else:
            while radians < -math.pi:
                radians += 2 * math.pi
        return Angle(radians=radians)

    def __floordiv__(self, other):
        if isinstance(other, (int, float)):
            radians = self.radians // other
        else:
            raise Exception("// operation between Angle and " + str(type) + " not supported.")
        # get in range (-pi, pi)
        if radians > 0:
            while radians > math.pi:
                radians -= 2 * math.pi
        else:
            while radians < -math.pi:
                radians += 2 * math.pi
        return Angle(radians=radians)

    def __radd__(self, other):
        if isinstance(other, Angle):
            radians = other.radians + self.radians
        elif isinstance(other, (int, float)):
            radians = other + self.radians
        else:
            raise Exception("+ operation between " + str(type) + " and Angle not supported.")
        # get in range (-pi, pi)
        if radians > 0:
            while radians > math.pi:
                radians -= 2 * math.pi
        else:
            while radians < -math.pi:
                radians += 2 * math.pi
        return Angle(radians=radians)

    def __rsub__(self, other):
        if isinstance(other, Angle):
            radians = other.radians - self.radians
        elif isinstance(other, (int, float)):
            radians = other - self.radians
        else:
            raise Exception("- operation between " + str(type) + " and Angle not supported.")
        # get in range (-pi, pi)
        if radians > 0:
            while radians > math.pi:
                radians -= 2 * math.pi
        else:
            while radians < -math.pi:
                radians += 2 * math.pi
        return Angle(radians=radians)

    def __rmul__(self, other):
        if isinstance(other, (int, float)):
            radians = other * self.radians
        else:
            raise Exception("* operation between Angle and " + str(type) + " not supported.")
        # get in range (-pi, pi)
        if radians > 0:
            while radians > math.pi:
                radians -= 2 * math.pi
        else:
            while radians < -math.pi:
                radians += 2 * math.pi
        return Angle(radians=radians)

    def __neg__(self):
        return Angle(radians=-self.radians)

    def __eq__(self, other):
        if isinstance(other, Angle):
            return abs(self.radians - other.radians) < 1e-8
        elif isinstance(other, (int, float)):
            return abs(self.radians - other) < 1e-8
        else:
            raise Exception("== operation between Angle and " + str(type) + " not supported.")

    def __ne__(self, other):
        if isinstance(other, Angle):
            return abs(self.radians - other.radians) > 1e-8
        elif isinstance(other, (int, float)):
            return abs(self.radians - other) > 1e-8
        else:
            raise Exception("!= operation between Angle and " + str(type) + " not supported.")

    def __lt__(self, other):
        if isinstance(other, Angle):
            return self.radians < other.radians
        elif isinstance(other, (int, float)):
            return self.radians < other
        else:
            raise Exception("< operation between Angle and " + str(type) + " not supported.")

    def __le__(self, other):
        if isinstance(other, Angle):
            return self.radians <= other.radians
        elif isinstance(other, (int, float)):
            return self.radians <= other
        else:
            raise Exception("<= operation between Angle and " + str(type) + " not supported.")

    def __gt__(self, other):
        if isinstance(other, Angle):
            return self.radians > other.radians
        elif isinstance(other, (int, float)):
            return self.radians > other
        else:
            raise Exception("> operation between Angle and " + str(type) + " not supported.")

    def __ge__(self, other):
        if isinstance(other, Angle):
            return self.radians >= other.radians
        elif isinstance(other, (int, float)):
            return self.radians >= other
        else:
            raise Exception(">= operation between Angle and " + str(type) + " not supported.")

    def __str__(self):
        return str(round(self.radians * 180 / math.pi, 8))

    def __hash__(self):
        return hash(self.radians)

    @staticmethod
    def sin(a):
        if isinstance(a, Angle):
            return math.sin(a.in_radians())
        elif isinstance(a, (int, float)):
            return math.sin(a)
        else:
            raise Exception("sin function for " + str(type) + " not supported.")

    @staticmethod
    def cos(a):
        if isinstance(a, Angle):
            return math.cos(a.in_radians())
        elif isinstance(a, (int, float)):
            return math.cos(a)
        else:
            raise Exception("cos function for " + str(type) + " not supported.")

    @staticmethod
    def tan(a):
        if isinstance(a, Angle):
            return math.tan(a.in_radians())
        elif isinstance(a, (int, float)):
            return math.tan(a)
        else:
            raise Exception("tan function for " + str(type) + " not supported.")

    @staticmethod
    def asin(x):
        if isinstance(x, (int, float)):
            return Angle(radians=math.asin(x))
        else:
            raise Exception("asin function for " + str(type) + " not supported.")

    @staticmethod
    def acos(x):
        if isinstance(x, (int, float)):
            return Angle(radians=math.acos(x))
        else:
            raise Exception("acos function for " + str(type) + " not supported.")

    @staticmethod
    def atan(x):
        if isinstance(x, (int, float)):
            return Angle(radians=math.atan(x))
        else:
            raise Exception("atan function for " + str(type) + " not supported.")

    @staticmethod
    def atan2(y, x):
        if isinstance(x, (int, float)) and isinstance(y, (int, float)):
            return Angle(radians=math.atan2(y, x))
        else:
            raise Exception("atan2 function for " + str(type) + " not supported.")


