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
	def __init__(self, parent=None):
		QtWidgets.QWidget.__init__(self, parent)   # Inherit from QWidget
		self.canvas = MplCanvas()                  # Create canvas object
		self.vbl = QtWidgets.QVBoxLayout()         # Set box for plotting
		self.vbl.addWidget(self.canvas)
		self.setLayout(self.vbl)




class MplCanvas(Canvas):
		def __init__(self):

			def onclick(event):
				msg = OptionSelect()
				'''print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
					('double' if event.dblclick else 'single', event.button,
					 event.x, event.y, event.xdata, event.ydata))'''
				
				self.rewards = self.getRewards_client((self._goal))
				max_reward = max(self.rewards)
				width = max_reward*0.16
				location = 0
				if event.xdata:
					ax_min, ax_max = event.canvas.ax.get_xlim()
					location = math.floor(event.xdata/width) +1 

				try:
					self.span.remove()
				except:
					pass
				if location < 6:
					self.span = event.canvas.ax.axvspan(0, location*width, color='red', alpha=0.5)
				event.canvas.draw()

				msg.option = int(location-1)
				self.option_pub.publish(msg)

			self.fig = Figure()
			self.ax = self.fig.add_subplot(111)
			Canvas.__init__(self, self.fig)
			# Matplotlib canvas class to create figure

			Canvas.setSizePolicy(self, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
			Canvas.updateGeometry(self)


			cid = self.fig.canvas.mpl_connect('button_press_event', onclick)
			self.goal_sub = rospy.Subscriber('current_goal', NamedGoal, self.goal_cb)
			self.option_pub = rospy.Publisher('option', OptionSelect, queue_size=10)

		def goal_cb(self, msg):
			 #Resolve the odometry to a screen coordinate for display

			worldX = msg.pose.x
			worldY = msg.pose.y

			print 'Got Goal at: ' + str(worldX) + ',' + str(worldY)

			self._goal = msg.id
				#self.goals_changed.emit()





		def getRewards_client(self, id):
			try:
				goal = rospy.ServiceProxy('/policy/policy_server/GetMCSims', GetMCSims)
				response = goal(id)

				return response.rewards
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e





