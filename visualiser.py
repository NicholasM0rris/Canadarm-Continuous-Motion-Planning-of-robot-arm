import sys
import math
import tkinter as tk
from problem_spec import ProblemSpec
from tester import load_output

"""
Graphical visualiser program.

Use this program to visualise the path of your solution files. This program takes 2 arguments - an input file and
(optionally) a solution file.

You may add to the update(), render_environment() or render_robot() functions of this class (e.g. to help visualise
your sampling strategy) if you wish. You should avoid modifying all other functions.

Taking screenshots of this program may be useful for your report.

COMP3702 2019 Assignment 2 Support Code

Last updated by njc 01/09/19
"""


class Visualiser:
    """
    GUI main class for visualiser program.
    """

    # internal states
    PAUSED = 0
    PLAYING = 1

    # canvas size
    CANV_SIZE = 600

    LINE_WIDTH = 3

    EE_LABEL_OFFSET = 0.025

    # render colours
    OBSTACLE_COLOUR = "white"
    GRAPPLE_COLOUR = "red"
    ROBOT_COLOUR = "blue"
    GOAL_COLOUR = "green"

    # radius of grapple point
    GP_RADIUS = 0.013

    def __init__(self, spec, soln):
        self.problem_spec = spec
        self.soln = soln

        window = tk.Tk()
        window.title("Visualiser")
        window.geometry("600x750")

        canvas = tk.Canvas(window, bg="white", height=600, width=600)
        canvas.pack()

        lower_control_frame = tk.Frame(window, pady=10)
        lower_control_frame.pack(side=tk.BOTTOM)

        step_label = tk.Label(lower_control_frame, text="Playback Position")
        step_label.pack(side=tk.BOTTOM)
        step_slider = tk.Scale(lower_control_frame, from_=0, to=len(soln)-1, length=500, orient=tk.HORIZONTAL,
                               command=self.handle_step_slider)
        step_slider.pack()

        upper_control_frame = tk.Frame(window)
        upper_control_frame.pack(side=tk.BOTTOM)

        play_frame = tk.Frame(upper_control_frame, padx=10)
        play_frame.pack(side=tk.LEFT)
        play_btn = tk.Button(play_frame, text="Play", width=10, height=2,
                             command=self.handle_play)
        play_btn.pack(side=tk.LEFT)

        reset_frame = tk.Frame(upper_control_frame, padx=10)
        reset_frame.pack(side=tk.LEFT)
        reset_btn = tk.Button(reset_frame, text="Reset", width=10, height=2,
                              command=self.handle_reset)
        reset_btn.pack(side=tk.LEFT)

        speed_frame = tk.Frame(upper_control_frame, width=300, height=50, padx=30)
        speed_frame.pack(side=tk.RIGHT)
        speed_label = tk.Label(speed_frame, text="Playback Speed")
        speed_label.pack(side=tk.BOTTOM)
        speed_slider = tk.Scale(speed_frame, from_=1, to=400, length=300, orient=tk.HORIZONTAL,
                                command=self.handle_speed_slider)
        speed_slider.pack()
        speed_slider.set(20)

        self.window = window
        self.canvas = canvas
        self.step_slider = step_slider
        self.play_btn = play_btn
        self.reset_btn = reset_btn
        self.speed_slider = speed_slider

        self.state = self.PAUSED
        self.idx = 0

        self.update_interval = 1000 // int(self.speed_slider.get())

        self.drawn_objects = []

        try:
            self.bg_img = tk.PhotoImage(file="assets/background_1.png")
        except Exception:
            self.bg_img = None

        self.render_environment()
        self.drawn_objects += self.render_robot(self.problem_spec.initial, self.ROBOT_COLOUR)  # initial
        self.drawn_objects += self.render_robot(self.problem_spec.goal, self.GOAL_COLOUR)  # goal

        # queue initial update
        self.last_update_job = window.after(1, self.update)

        try:
            window.mainloop()
        except:
            sys.exit(0)

    def update(self):
        # compute adjusted update interval - should never be shorter than 0.05 sec
        inc = 1
        adjusted_interval = self.update_interval
        while adjusted_interval < 100:
            adjusted_interval *= 2
            inc += 1

        # if playing, update display; otherwise do nothing
        if self.state == self.PLAYING:
            # reset if play pressed while at end
            if self.idx >= len(self.soln):
                self.idx = 0

            # update canvas and slider
            self.unrender_objects(self.drawn_objects)
            self.drawn_objects += self.render_robot(self.soln[self.idx], self.ROBOT_COLOUR)
            self.step_slider.set(self.idx)

            # move to next index
            self.idx += inc

            # stop if at end of solution
            if self.idx >= len(self.soln):
                self.idx = len(self.soln) - 1
                # TODO: draw all robot positions?
                self.unrender_objects(self.drawn_objects)
                self.drawn_objects += self.render_robot(self.problem_spec.initial, self.ROBOT_COLOUR)   # initial
                self.drawn_objects += self.render_robot(self.problem_spec.goal, self.GOAL_COLOUR)   # goal

                self.step_slider.set(self.idx)
                self.state = self.PAUSED
                self.play_btn.config(text="Play")

        # queue next update after interval
        self.last_update_job = self.window.after(self.update_interval, self.update)

    def render_environment(self):
        # render background
        if self.bg_img is not None:
            self.canvas.create_image(0, 0, anchor=tk.NW, image=self.bg_img)
        else:
            self.canvas.create_rectangle(0, 0, 600, 600, fill='black')

        # draw obstacles
        for o in self.problem_spec.obstacles:
            self.canvas.create_rectangle(o.x1 * self.CANV_SIZE,
                                         (1 - o.y1) * self.CANV_SIZE,
                                         o.x2 * self.CANV_SIZE,
                                         (1 - o.y2) * self.CANV_SIZE,
                                         fill=self.OBSTACLE_COLOUR,
                                         outline=self.OBSTACLE_COLOUR)

        # draw grapple points
        for gpx, gpy in self.problem_spec.grapple_points:
            self.canvas.create_oval((gpx - self.GP_RADIUS) * self.CANV_SIZE,
                                    ((1 - gpy) - self.GP_RADIUS) * self.CANV_SIZE,
                                    (gpx + self.GP_RADIUS) * self.CANV_SIZE,
                                    ((1 - gpy) + self.GP_RADIUS) * self.CANV_SIZE,
                                    fill=self.GRAPPLE_COLOUR,
                                    outline=self.GRAPPLE_COLOUR)

    def render_robot(self, robot_config, colour):
        drawn_objects = []

        # draw arm segments
        for i in range(len(robot_config.points) - 1):
            x1, y1 = robot_config.points[i]
            x2, y2 = robot_config.points[i + 1]
            line = self.canvas.create_line(x1 * self.CANV_SIZE,
                                           (1 - y1) * self.CANV_SIZE,
                                           x2 * self.CANV_SIZE,
                                           (1 - y2) * self.CANV_SIZE,
                                           fill=colour,
                                           width=self.LINE_WIDTH)
            drawn_objects.append(line)

        # draw label on ee1
        ee1x, ee1y = robot_config.points[0]
        drawn_objects.append(self.canvas.create_text(
            (ee1x + (math.cos(robot_config.ee1_angles[0].in_radians() + math.pi) * self.EE_LABEL_OFFSET))
            * self.CANV_SIZE,
            (1 - (ee1y + (math.sin(robot_config.ee1_angles[0].in_radians() + math.pi) * self.EE_LABEL_OFFSET)))
            * self.CANV_SIZE,
            text='EE1',
            fill=colour))

        # draw label on ee2
        ee2x, ee2y = robot_config.points[-1]
        drawn_objects.append(self.canvas.create_text(
            (ee2x + (math.cos(robot_config.ee2_angles[0].in_radians() + math.pi) * self.EE_LABEL_OFFSET))
            * self.CANV_SIZE,
            (1 - (ee2y + (math.sin(robot_config.ee2_angles[0].in_radians() + math.pi) * self.EE_LABEL_OFFSET)))
            * self.CANV_SIZE,
            text='EE2',
            fill=colour))

        return drawn_objects

    def unrender_objects(self, objects):
        for o in objects:
            self.canvas.delete(o)

    def handle_play(self):
        if self.state == self.PAUSED:
            self.state = self.PLAYING
            self.play_btn.config(text="Pause")
        else:
            self.state = self.PAUSED
            self.play_btn.config(text="Play")
        # queue next update immediately
        self.window.after_cancel(self.last_update_job)
        self.last_update_job = self.window.after(1, self.update)

    def handle_reset(self):
        if self.state == self.PLAYING:
            self.state = self.PAUSED
            self.play_btn.config(text="Play")
        self.idx = 0

        # update canvas and slider
        self.unrender_objects(self.drawn_objects)
        self.drawn_objects += self.render_robot(self.problem_spec.initial, self.ROBOT_COLOUR)  # initial
        self.drawn_objects += self.render_robot(self.problem_spec.goal, self.GOAL_COLOUR)      # goal
        self.step_slider.set(self.idx)

        # queue next update immediately
        self.window.after_cancel(self.last_update_job)
        self.last_update_job = self.window.after(1, self.update)

    def handle_step_slider(self, value):
        self.idx = int(value)

        # update canvas and slider
        if self.state == self.PAUSED and 0 < self.idx < len(self.soln) - 1:
            self.unrender_objects(self.drawn_objects)
            self.drawn_objects += self.render_robot(self.soln[self.idx], self.ROBOT_COLOUR)   # current
        self.step_slider.set(self.idx)

        # queue next update immediately
        self.window.after_cancel(self.last_update_job)
        self.last_update_job = self.window.after(1, self.update)

    def handle_speed_slider(self, value):
        self.update_interval = 1000 // int(value)

        # queue next update immediately
        self.window.after_cancel(self.last_update_job)
        self.last_update_job = self.window.after(1, self.update)


def main(arglist):
    if len(arglist) == 0 or len(arglist) > 2:
        print("Running this file launches a graphical program for visualising maps and solutions.")
        print("Usage: visualiser.py [input_file] [solution_file(optional)]")
        return
    spec = ProblemSpec(arglist[0])
    if len(arglist) == 2:
        soln = load_output(arglist[1])
    else:
        soln = []
    vis = Visualiser(spec, soln)


if __name__ == '__main__':
    main(sys.argv[1:])


