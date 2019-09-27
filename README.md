# Assignment 2 Support Code

This is the support code for COMP3702 2019 Assignment 2.

The following files are provided:

**support/problem_spec.py**

This file contains the `ProblemSpec` class. This class serves the same purpose as the `SokobanMap` class from the first assignment - it can be used to represent the environment your agent operates in. The constructor for this class takes the filename of the input file as an argument, and handles parsing of the input file. The properties of the environment are stored as class variables within the `ProblemSpec` instance which can be accessed by your code.

Refer to the documentation at the start of this file for more details.

**support/robot_config.py**

This file contains the `RobotConfig` class. This class represents a particular configuration (state) the robot can be in. There are 2 helper methods which can be used to generate sample configurations using the reference frame of either EE1 or EE2. Additionally, this file provides a method for writing a solution (i.e. a list of `RobotConfig` objects forming a path) to an output file.

Refer to the documentation at the start of this file for more details.

**support/obstacle.py**

This file contains the `Obstacle` class, a simple class representing a rectangular obstacle.

Refer to the documentation at the start of this file for more details.

**support/angle.py**

This file contains the `Angle` class, representing an angle. This class behaves like a normal floating point number, supporting addition, subtraction, multiplication by scalar, division by scalar, negation, equality and comparison. Constructor accepts degrees or radians, and value can be accessed as degrees or radians. Automatically keeps value in the range of -pi and pi.

We suggest using this class when working with angles to avoid having to manually keep the angle within the correct range.

This class also contains static methods for performing trigonometric operations on `Angle` objects.

Refer to the documentation at the start of this file for more details.

**tester.py**

This file is a script which takes an input file and a solution file as input, and determines whether the solution file is valid. This script will be used to verify your solutions during your demonstration. Your should ensure that your solution files match the standard required by the `tester.py` script. For valid solution files, the script will output "Testcase solved successfully!".

Refer to the documentation at the start of this file for more details.

**visualiser.py**

This file is a GUI program which takes an input file and (optionally) a solution file as input, and provides a graphical representation of the input file, and animates the path provided in the solution file (if a solution file was given).

Refer to the documentation at the start of this file for more details.

**testcases**

Example input files for you to test your solver on. You should make sure that your solver is able to produce valid solution files (verified by `tester.py`) for these inputs.





