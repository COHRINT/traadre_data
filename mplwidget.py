#!/usr/bin/env python2.7

# Imports
from PyQt5 import QtWidgets
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as Canvas
import matplotlib
import matplotlib.pyplot as plt




class MplCanvas(Canvas):
	def __init__(self):
		def onclick(event):
			'''print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
				('double' if event.dblclick else 'single', event.button,
				 event.x, event.y, event.xdata, event.ydata))'''
			try:
				self.span.remove()
			except:
				pass
			self.span = event.canvas.ax.axvspan(event.x, event.x+10, color='red', alpha=0.5)
			event.canvas.draw()


		# Matplotlib canvas class to create figure



		self.fig = Figure()
		self.ax = self.fig.add_subplot(111)
		Canvas.__init__(self, self.fig)
		Canvas.setSizePolicy(self, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
		Canvas.updateGeometry(self)

		cid = self.fig.canvas.mpl_connect('button_press_event', onclick)

# Matplotlib widget
class MplWidget(QtWidgets.QWidget):
	def __init__(self, parent=None):
		QtWidgets.QWidget.__init__(self, parent)   # Inherit from QWidget
		self.canvas = MplCanvas()                  # Create canvas object
		self.vbl = QtWidgets.QVBoxLayout()         # Set box for plotting
		self.vbl.addWidget(self.canvas)
		self.setLayout(self.vbl)


