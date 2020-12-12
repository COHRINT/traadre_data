#!/usr/bin/env python2.7

# Imports
import rospy
import math
from PyQt5 import QtWidgets
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as Canvas
import matplotlib
import matplotlib.pyplot as plt
from traadre_msgs.srv import *
from traadre_msgs.msg import *


# Matplotlib widget
class MplWidget(QtWidgets.QWidget):
	def __init__(self):
		QtWidgets.QWidget.__init__(self)   # Inherit from QWidget
		self.canvas = MplCanvas()                  # Create canvas object
		self.vbl = QtWidgets.QVBoxLayout()         # Set box for plotting
		self.vbl.addWidget(self.canvas)
		self.setLayout(self.vbl)


class MplCanvas(Canvas):
		def __init__(self):
			plt.rcParams.update({'font.size': 15})

			self.fig = Figure()
			self.ax = self.fig.add_subplot(111)


			Canvas.__init__(self, self.fig)
			# Matplotlib canvas class to create figure

			Canvas.setSizePolicy(self, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
			Canvas.updateGeometry(self)



			#cid = self.fig.canvas.mpl_connect('button_press_event', onclick)



