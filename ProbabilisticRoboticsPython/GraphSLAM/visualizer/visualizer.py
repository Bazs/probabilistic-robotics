from graph_slam import GraphSlamState, generate_ground_truth_map, generate_ground_truth_path
from slam_parameters import *

from PyQt5.QtWidgets import QDialog, QApplication, QPushButton, QVBoxLayout

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import matplotlib.pyplot as plt

import random as rnd
import sys


class MainWindow(QDialog):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)

        # a figure instance to plot on
        self.figure = plt.figure()

        # Attribute for holding the plot of the current path
        self.path_plot = None

        # this is the Canvas Widget that displays the `figure`
        # it takes the `figure` instance as a parameter to __init__
        self.canvas = FigureCanvas(self.figure)

        # this is the Navigation widget
        # it takes the Canvas widget and a parent
        self.toolbar = NavigationToolbar(self.canvas, self)

        # Button to trigger new path generation
        self.button = QPushButton('Regenerate path')
        self.button.clicked.connect(self.generate_path)

        # set the layout
        layout = QVBoxLayout()
        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas)
        layout.addWidget(self.button)
        self.setLayout(layout)

        self.slam_state = GraphSlamState()
        self._init_world()
        self._plot_world()

        # Set up truly random number generation for creating the ground truth path (if the system supports it)
        rnd.seed(self.slam_state.true_random_gen.random())

    def _init_world(self):
        self.slam_state.ground_truth_map, self.slam_state.landmarks = \
            generate_ground_truth_map(MAP_HEIGHT, MAP_WIDTH, LANDMARK_COUNT)

    def _plot_world(self):
        plt.title("Ground truth map")
        plt.imshow(self.slam_state.ground_truth_map, origin='lower')

    def generate_path(self):
        self.slam_state.ground_truth_states = generate_ground_truth_path(
            self.slam_state.ground_truth_map, max_velocity=MAX_VELOCITY, velocity_variance=VELOCITY_VARIANCE,
            max_turn_rate=MAX_TURN_RATE, turn_rate_variance=TURN_RATE_VARIANCE, step_count=STEP_COUNT)

        path_x = []
        path_y = []
        for state in self.slam_state.ground_truth_states:
            path_x.append(state[0])
            path_y.append(state[1])

        if self.path_plot is not None:
            self.path_plot.remove()

        self.path_plot, = plt.plot(path_x, path_y, marker='o')

        # # refresh canvas
        self.canvas.draw()


if __name__ == '__main__':
    app = QApplication(sys.argv)

    main = MainWindow()
    main.show()

    sys.exit(app.exec_())
