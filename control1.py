#!/usr/bin/env python2.7

from PyQt5 import QtGui, QtCore, QtWidgets
from PyQt5.QtWidgets import *; 
from PyQt5.QtGui import *;
from PyQt5.QtCore import *;
import sys,os, os.path

import rospy
import signal
from code.QCircle import *
import math
from code.RobotIcon import *
from traadre_msgs.msg import *
from traadre_msgs.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from code.OA import *
from code.plane_functions import *

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np

import cv2
import copy
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError

from code.mplwidget import MplWidget
from code.surveywidget import SurveyWidget
from code.historywidget import HistoryWidget


class SimulationWindow(QWidget):
	sketch = pyqtSignal()
	defogSignal = pyqtSignal(int, int, int, int, list, int)
	cameraSketch = pyqtSignal()
	dem_changed = pyqtSignal()
	robot_odom_changed = pyqtSignal()
	goals_changed = pyqtSignal()
	hazmap_changed = pyqtSignal(int)
	rightClick = pyqtSignal(int,int)
	option_changed = pyqtSignal(int, bool)
	state = pyqtSignal(int,int)
	buttonClicked = pyqtSignal(int)

	def __init__(self):

		super(SimulationWindow,self).__init__()

		self.layout = QGridLayout(); 
		self.setLayout(self.layout); 


		#self.setGeometry(300, 300, 250, 150)
		self.setWindowTitle("DLA BluePrint");
		self.setStyleSheet("background-color:slategray;")
		self.populateInterface(); 
		self.populated = True

		self.showMaximized();

		#Stop interfacing from stretching to accomodate widgets!
		for i in range(1,self.layout.rowCount()):
			self.layout.setRowStretch(i,1)
		for i in range(1,self.layout.columnCount()):
			self.layout.setColumnStretch(i,1)


		self.setWindowState(Qt.WindowMaximized);
		self.dem_sub = rospy.Subscriber('dem', Image, self.dem_cb)
		self.steer_sub = rospy.Subscriber('current_steer', Steering, self.state_callback)
		self.goal_sub = rospy.Subscriber('current_goal', NamedGoal, self.goal_cb)
		self.option_sub = rospy.Subscriber('option', OptionSelect, self.option_cb)

		self.option_pub = rospy.Publisher('dist_bin', OptionSelect, queue_size=10)
		self.current_state_pub = rospy.Publisher('state', RobotState, queue_size=10)
		self.hazard_pub = rospy.Publisher('hazard', Hazard, queue_size=10)
		self.decision_pub = rospy.Publisher('user_decision', TraverseDecision, queue_size=10)
		self.mouse_pub = rospy.Publisher('mouse', Mouse, queue_size=10)
		self.history_pub = rospy.Publisher('history', PrevTraverse, queue_size=10)



		# self.populated = False

		self.a = 255*.3

		self._colors = [(125, 0, 125), (68, 134, 252), (236, 228, 46), (102, 224, 18), (242, 156, 6), (240, 64, 10), (196, 30, 250)]
		self._goalLocations = [(0,0)]

		rospy.init_node('interface')
		self.msg = RobotState()
		rate = rospy.Rate(10) # 10hz
		# self.sketchPub = rospy.Publisher('/Sketch', sketch, queue_size=10)


	def populateInterface(self):
		self.goals_changed.connect(self._updateGoal)
		self.dem_changed.connect(self._update)
		self.hazmap_changed.connect(self._updateHazmap)
		self.robot_odom_changed.connect(self._updateRobot)
		self.option_changed.connect(self._updateOption)
		self.buttonClicked.connect(self.operator_toast)
		self._dem_item = None
		self._goalIcon = None
		self.gworld = [0,0]
		self.hazmapItem = None
		self.current_score = 0
		self._goalID = 'Start'
		self._robotIcon = None
		self.demDownsample = 4
		self.count = 0
		self.num_options = 6
		self.paths = None
		self.span = None


		#Minimap ---------------------------
		self.minimapView = QGraphicsView(self); 
		self.minimapScene = QGraphicsScene(self);


		self.minimapView.setScene(self.minimapScene);
		self.minimapView.setStyleSheet("background-color: beige; border: 4px inset grey;")
		self.layout.addWidget(self.minimapView,1,1,8,9);


		#Add circle ---------------------------------------------
		self.thisRobot = QArrow(color=QColor(0,250,0,255))
		self.minimapScene.addItem(self.thisRobot);
		self.thisRobot.setZValue(2)


		#Hazard slider --------------------------------
		sliderLayout = QGridLayout(); 
		self.beliefOpacitySlider = QSlider(Qt.Horizontal); 
		self.beliefOpacitySlider.setSliderPosition(30)
		self.beliefOpacitySlider.setTickPosition(QSlider.TicksBelow)
		self.beliefOpacitySlider.setTickInterval(10); 

		sliderLayout.addWidget(self.beliefOpacitySlider,0,0);

		self.layout.addLayout(sliderLayout,9,1,1,9) 

		#----------------------------------------------
		self.stateLayout = QGridLayout();

		stateGroup = QGroupBox()
		stateGroup.setLayout(self.stateLayout)

		stateGroup.setStyleSheet("background-color: beige; border: 4px inset grey; font: 15pt Lato")
		self.time_remaining = 120

		self.fuel = QLabel()
		self.fuel.setStyleSheet("background-color: white")
		self.timer = QLabel()
		self.timer.setStyleSheet("background-color: white")
		self.goal = QLabel()

		self.goal.setStyleSheet("background-color: white")
		self.score = QLabel()
		self.score.setToolTip("Current score")
		self.score.setStyleSheet("background-color: white")
		self.shotClock = QTimer()

		self.shotClock.timeout.connect(self.updateTime)


		self.fuel.setText('')
		self.goal.setText('Current Goal: ' + self._goalID)
		self.timer.setText('Time Remaining: ' + str(self.time_remaining) + ' seconds')
		self.score.setText('Current Score: ' + str(self.current_score))

		self.stateLayout.addWidget(self.fuel,12,1,1,1); 
		self.stateLayout.addWidget(self.goal,11,1);
		self.stateLayout.addWidget(self.timer,11,2); 
		self.stateLayout.addWidget(self.score,12,2); 
		self.layout.addWidget(stateGroup,10,1,4,9)

		#self.timer.setToolTip("Time remaining to make decision, at 0 there will be a penalty")
		#self.timer.setStyleSheet("""QToolTip { background-color: black; color: white; border: black solid 1px }""")
		#self.goal.setToolTip("Current navigation goal from [A,F]")

		#------------------------------------------
		self.pushLayout = QGridLayout();

		#Stop interfacing from stretching to accomodate widgets!
		for i in range(1,self.pushLayout.rowCount()):
			self.pushLayout.setRowStretch(i,1)
		for i in range(1,self.pushLayout.columnCount()):
			self.pushLayout.setColumnStretch(i,1)

		goGroup = QGroupBox()
		goGroup.setLayout(self.pushLayout)

		goGroup.setStyleSheet("QGroupBox {background-color: beige; border: 4px inset grey;}")

		self.go_btn = QPushButton('Go',self)
		#self.go_btn.setEnabled(False)
		self.pushLayout.addWidget(self.go_btn,3,10,1,4); 
		self.go_btn.setFixedSize(QSize(300,50))
		self.go_btn.setStyleSheet("background-color: green; color: white")
		self.go_btn.setFont(QtGui.QFont('Lato', 12))

		self.no_btn = QPushButton('No Go',self)
		#self.no_btn.setEnabled(False)
		self.pushLayout.addWidget(self.no_btn,6,10,1,4); 
		self.no_btn.setFixedSize(QSize(300,50))
		self.no_btn.setStyleSheet("background-color: red; color: white")
		self.no_btn.setFont(QtGui.QFont('Lato', 12))

		self.prev_btn = QPushButton('Previous Traverse',self)
		self.prev_btn.setVisible(False)
		self.prev_btn.setEnabled(False)
		self.pushLayout.addWidget(self.prev_btn,9,10,1,4); 
		self.prev_btn.setFixedSize(QSize(300,50))
		self.prev_btn.setStyleSheet("background-color: grey; color: white")
		self.prev_btn.setFont(QtGui.QFont('Lato', 12))
		
		self.sq_label = QLabel()
		self.sq = None
		self.sq_label.setText('Solver Quality: ' + str(self.sq))
		self.sq_label.setVisible(False)
		self.sq_label.setEnabled(False)
		self.pushLayout.addWidget(self.sq_label,9,3,1,4)
		self.sq_label.setStyleSheet("background-color: white; border: 4px inset grey;")
		self.sq_label.setFont(QtGui.QFont('Lato', 15))

		self.go_btn.clicked.connect(self.go_button)
		self.no_btn.clicked.connect(self.no_button)

		self.prev_btn.clicked.connect(self.traverse_history)

		#--------------------------------------------------------------------
		self.table = QTableWidget(self.num_options,6,self)

		self.table.setFont(QtGui.QFont('Lato', 12))
		self.table.horizontalHeader().setFont(QtGui.QFont('Lato', 12))
		self.table.verticalHeader().setFont(QtGui.QFont('Lato', 12))
		self.table.setHorizontalHeaderLabels(('Span', 'Outcome Assessment','Avg. Steps','% of Samples','Min. Reward', '% Goal Reached'))
		self.table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		self.table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		self.table.setStyleSheet("background-color: rgb(192,192,192)")
		self.table.setMaximumWidth(self.table.horizontalHeader().length()+30)
		self.table.setMinimumHeight(self.table.verticalHeader().length()+28)
		self.table.setMaximumHeight(self.table.verticalHeader().length()+31)
		#self.table.setSizePolicy(QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding))
		self.table.resizeColumnsToContents()
		self.table.setVisible(False)
		self.table.setEnabled(False)
		#self.table.resizeRowsToContents()

		self.pushLayout.addWidget(self.table,1,1,10,8,Qt.AlignTop); 



		self.layout.addWidget(goGroup,10,11,4,12)


		#-----------------------------------------------------
		self.histLayout = QGridLayout();

		histGroup = QGroupBox()
		histGroup.setLayout(self.histLayout)

		histGroup.setStyleSheet("QGroupBox {background-color: beige; border: 4px inset grey;}")
		histGroup.setToolTip('Choose one of reward distribution bins')

		histGroup.setVisible(False)
		histGroup.setEnabled(False)

		self.layout.addWidget(histGroup, 1,11,8,12)

	def go_button(self):
		self.buttonClicked.emit(1)

	def no_button(self):
		self.buttonClicked.emit(0)

	def sliderChanged(self):
		msg = Hazard()
		self.a = 255*self.beliefOpacitySlider.sliderPosition()/100
		self.hazmap_changed.emit(self.a)

		msg.goal = self._goalID
		msg.value = self.beliefOpacitySlider.sliderPosition()
		self.hazard_pub.publish(msg)

	def updateTime(self):
		if self.time_remaining != 0:
			self.time_remaining = self.time_remaining - 1
			self.timer.setText('Time Remaining: ' + str(self.time_remaining) + ' seconds')

		if self.time_remaining > 60:
			self.timer.setStyleSheet("background-color: rgba(192,192,192,255)")

		if self.time_remaining > 10 and self.time_remaining < 60:
			self.timer.setStyleSheet("background-color: rgba(255,255,0,150)")

		if self.time_remaining < 10:
			self.timer.setStyleSheet("background-color: rgba(255,0,0,150)")
		if self.time_remaining == 0:
			self.buttonClicked.emit(2)

	def option_cb(self, data):
		self.option_changed.emit(data.option, data.boundary)

	def _updateOption(self, option, boundary):
		tile_x = (float(self._dem.width())/20.0)/2
		tile_y = (float(self._dem.height())/20.0)/2

		if boundary == False:
			self.table.clearSelection()
			planeFlushPaint(self.pathPlane)
			self.draw_paths()
		else:

			self.table.selectRow(option)
			self.redrawPaths()
			band = self.bins[option]

			for i in self.paths:
				if i.reward >= (band):
					x_tmp = []
					y_tmp = []
					for j in i.elements:
						x,y = self.convertToGridCoords(j,20,20)
						x_tmp.append(int(float(x/20.0)*self._dem.width() + tile_x))
						y_tmp.append(int(float(y/20.0)*self._dem.height() + tile_y))
					planeAddPaint(self.pathPlane, 200, x_tmp, y_tmp, QColor(250,251,0,50))




	def operator_toast(self, button):
		msg = TraverseDecision()

		_,_, result = self.getResults_client(self._goalID,'present')
		if button == 1 and result == 1:
			self.score_delta = 1
		elif button == 1 and result == 0: 
			self.score_delta = -1
		elif button == 0:
			self.score_delta = -0.25
		else:
			self.score_delta = -2
		self.current_score +=self.score_delta

		msg.total_score = self.current_score
		msg.score_delta = self.score_delta
		msg.time_remaining = self.time_remaining
		self.decision_pub.publish(msg)

		print "Opening a new popup window..."
		self.setEnabled(False)
		self.shotClock.stop()
		self.survey = SurveyWidget()
		self.survey.setGeometry(QRect(100, 100, 800, 800))
		self.survey.submit_btn.clicked.connect(self.survey_submit)
		self.survey.show()

	def traverse_history(self):
		msg = PrevTraverse()
		msg.current_goal = self._goalID
		#msg.header = rospy.Time.now()
		self.history_pub.publish(msg)

		print "Opening a new popup window..."
		self.setEnabled(False)
		dem = self._dem_item.pixmap()
		haz = self.hazmapItem.pixmap()
		time = self.prev_time_remaining
		goalID = self.allGoals[self.count-1][0]

		goalLoc = self.allGoals[self.count-1][1]

		actions,reward,result= self.getResults_client(goalID,'past')

		self.trav_hist = HistoryWidget(dem,haz,time,goalID,goalLoc,actions,reward,result,self.prev_rewards,self.score_delta)
		self.trav_hist.setGeometry(QRect(100, 100, 1000, 1000))
		self.trav_hist.submit_btn.clicked.connect(self.trav_submit)
		self.trav_hist.show()

	def trav_submit(self):
		self.trav_hist.close()
		self.setEnabled(True)

	def survey_submit(self):
		self.paths = None
		self.prev_time_remaining = self.time_remaining
		self.prev_rewards = self.rewards
		self.score.setText('Current Score: ' + str(self.current_score))
		self.time_remaining = 120
		self.timer.setText('Time Remaining: ' + str(self.time_remaining) + ' seconds')
		self.shotClock.start(1000)
		self.count = self.count +1
		self.setEnabled(True)
		self.goals_changed.emit()
		#elf.buildTable()
		#self.go_btn.setEnabled(False)
		#self.go_btn.setStyleSheet("background-color: grey; color: white")
		self.survey.close()

		#self.no_btn.setEnabled(False)
		#self.no_btn.setStyleSheet("background-color: grey; color: white")
		
		
	def checkbox_callback(self,val, box):
		msg = OptionSelect()

		try:
			self.span.remove()
			self.notspan.remove()
		except:
			pass
		try:
			self.hist.canvas.span.remove()
			self.hist.canvas.notspan.remove()
		except:
			pass

		self.hist.canvas.draw()

		if val == True:
			ax_min, ax_max = self.hist.canvas.ax.get_xlim()
			self.hist.canvas.ax.set_xlim([0,ax_max])
			self.span = self.hist.canvas.ax.axvspan(0, self.bins[box], color='red', alpha=0.5)
			self.notspan = self.hist.canvas.ax.axvspan(self.bins[box], ax_max, color='green', alpha=0.15)
			self.hist.canvas.draw()

			msg.option = box 
			msg.boundary = True
			self.option_pub.publish(msg)

		elif val == False:
			msg.option = 7
			msg.boundary = False
			self.option_pub.publish(msg)


		#self.hist.canvas.onclick()

		'''count = 0
		for i in range(0, len(self.cb_list)):
			count = count + self.cb_list[i].checkState()
			
		if count == 2:
			self.go_btn.setEnabled(True)
			self.go_btn.setStyleSheet("background-color: green; color: white")
			self.no_btn.setEnabled(False)
			self.no_btn.setStyleSheet("background-color: grey; color: white")
		elif count == 0:
			self.no_btn.setEnabled(True)
			self.no_btn.setStyleSheet("background-color: red; color: white")
			self.go_btn.setEnabled(False)
			self.go_btn.setStyleSheet("background-color: grey; color: white")
		else: 
			self.go_btn.setEnabled(False)
			self.go_btn.setStyleSheet("background-color: grey; color: white")
			self.no_btn.setEnabled(False)
			self.no_btn.setStyleSheet("background-color: grey; color: white")'''

	def buildTable(self):

		self.cb1  = QtWidgets.QRadioButton( parent=self.table )
		self.cb2  = QtWidgets.QRadioButton( parent=self.table )
		self.cb3  = QtWidgets.QRadioButton( parent=self.table )
		self.cb4  = QtWidgets.QRadioButton( parent=self.table )
		self.cb5  = QtWidgets.QRadioButton( parent=self.table )
		self.cb6  = QtWidgets.QRadioButton( parent=self.table )

		self.cb_list = [self.cb1, self.cb2, self.cb3, self.cb4, self.cb5, self.cb6]
		for i in range(0,self.table.rowCount()):
			self.cb_list[i]  = QtWidgets.QRadioButton( parent=self.table )
			#self.cb_list[i].setTristate(False)
			#elf.cb_list[i].setChecked(0)
			self.cb_list[i].clicked.connect(lambda val, i=i: self.checkbox_callback(val, i))

			self.item_p = QTableWidgetItem(str(0))
			self.table.setToolTip('Investigate the distribution of potential outcomes')
			
			self.item_p.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)

			self.table.setCellWidget(i,0, self.cb_list[i])


		#self.table.resizeColumnsToContents()


	def getGoals_client(self):
		try:
			goal = rospy.ServiceProxy('/policy/policy_server/GetGoalList', GetGoalList)
			response = goal()
			row = response.goals

			return response.ids, row
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def getRewards_client(self, id):
		try:
			goal = rospy.ServiceProxy('/policy/policy_server/GetMCSims', GetMCSims)
			response = goal(id)
			return response.rewards, response.results

		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def getPaths_client(self, id):
		try:
			goal = rospy.ServiceProxy('/policy/policy_server/GetPaths', GetPaths)
			response = goal(id)

			return response.paths
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def getResults_client(self, id, temp):
		try:
			goal = rospy.ServiceProxy('/policy/policy_server/GetResults', GetResults)
			response = goal(id, temp)

			return response.actions, response.reward, response.result
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def getBins_client(self, id):
		try:
			goal = rospy.ServiceProxy('/policy/policy_server/Bins', Bins)
			response = goal(id)


			return response.bins
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def getPerf_client(self, id):
		try:
			goal = rospy.ServiceProxy('/policy/policy_server/GetPerf', GetPerf)
			response = goal(id)


			return response.reward, response.actions
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def setCurrentGoal_client(self,id):
		try:
			goal = rospy.ServiceProxy('/policy/policy_server/SetCurrentGoal', SetCurrentGoal)

			response = goal(id)
			return response.goal
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def state_callback(self, data):			
		self.steer = data.steer
		self.robot_odom_changed.emit()
		#worldRoll, worldPitch, self.worldYaw = euler_from_quaternion([data.pose.orientation.x,
		#								  data.pose.orientation.y,
		#								  data.pose.orientation.z,
		#								  data.pose.orientation.w],'sxyz')


		#self.location_update.emit(self.worldX, self.worldY, self.worldYaw, self._robotFuel)

	def hist_clicked(self, event):
		msg = OptionSelect()
		'''print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
			('double' if event.dblclick else 'single', event.button,
			 event.x, event.y, event.xdata, event.ydata))'''

		#self.rewards = self.getRewards_client((self._goal))
		max_reward = max(self.rewards)
		width = max_reward
		location = 7
		msg.boundary = False

		if event.xdata:
			ax_min, ax_max = event.canvas.ax.get_xlim()
			event.canvas.ax.set_xlim([0,ax_max])
			try:
				location = self.bins.index(min([x for x in self.bins if event.xdata <= x]))
			except:
				location = 7
			
		try:
			self.span.remove()
			self.notspan.remove()
		except:
			pass
		if location < 6:
			self.span = event.canvas.ax.axvspan(0, self.bins[location], color='red', alpha=0.5)
			self.notspan = event.canvas.ax.axvspan(self.bins[location], ax_max, color='green', alpha=0.15)
		event.canvas.draw()

		if event.ydata and event.xdata and location != 7:
			msg.option = int(location)
			msg.boundary = True
			self.cb_list[location].setChecked(1)
		else:
			for i in self.cb_list:
				i.setAutoExclusive(False)
				i.setChecked(0)
				i.setAutoExclusive(True)

		self.option_pub.publish(msg)


	def makeHist(self):
		if self.count != 0:
			self.histLayout.removeWidget(self.hist)
		self.hist = MplWidget(self.bins)
		self.hist.setStyleSheet("MplWidget {background-color: white; border: 4px inset grey;}")


		self.hist.canvas.draw()


		self.histLayout.addWidget(self.hist)

		self.hist.canvas.mpl_connect('button_press_event', self.hist_clicked)
		#Histogram stuff
		self.hist.canvas.ax.clear()


		mu = "%.1f" % np.mean(self.rewards)
		std = "%.2f" % np.std(self.rewards)
		samples = len(self.rewards)

		self.hist.canvas.ax.set_xlim(0,max(self.rewards)+100)

		self.hist.canvas.ax.set_xlabel('Accumulated Reward')
		self.hist.canvas.ax.set_ylabel('Samples out of ' + str(samples))
		self.hist.canvas.ax.set_title(r'Histogram of IQ: $\mu=100$, $\sigma=15$')


		self.hist.canvas.ax.hist(self.rewards, 50)


		for i in range(0,len(self.bins)):
			self.hist.canvas.ax.axvline(x=self.bins[i], linestyle = '--', color = 'red')


		self.hist.canvas.ax.set_title(r'Histogram of Potential Rewards: $\mu=$ ' + (mu) + r', $\sigma=$' + (std) + ', Samples used: ' + str(samples))
		self.hist.canvas.draw()

	'''def draw_paths(self):
		self.paths = self.getPaths_client(self._goalID)	
		self.planeFlushPaint(self.pathPlane)

		for i in self.paths:
			print i.reward
			for j in i.elements:
				x,y = self.convertToGridCoords(j,20,20)
				x_norm = float(x/20.0)*self._dem.width()
				y_norm = float(y/20.0)*self._dem.height()
				self.planeAddPaint(self.pathPlane, 200, int(x_norm),int(y_norm)) #maybe drawLine
				#print x, y

		self.pathPlane.setZValue(2)'''


	def draw_paths(self):
		if self.paths == None:
			self.paths = self.getPaths_client(self._goalID)	
		planeFlushPaint(self.pathPlane)
		tile_x = (float(self._dem.width())/20.0)/2
		tile_y = (float(self._dem.height())/20.0)/2
		#self.pathDict = {}

		for i in self.paths:
			x_norm = []
			y_norm = []
			#self.pathDict[str(i.reward)] = (i.elements)
			for j in i.elements:
				x,y = self.convertToGridCoords(j,20,20)
				x_norm.append(int(float(x/20.0)*self._dem.width() + tile_x))
				y_norm.append(int(float(y/20.0)*self._dem.height() + tile_y))
			planeAddPaint(self.pathPlane, 200, x_norm, y_norm, QColor(0,251,0,30)) 
					#print x, y

		self.pathPlane.setZValue(2)

	def redrawPaths(self):
		planeFlushPaint(self.pathPlane)
		tile_x = (float(self._dem.width())/20.0)/2
		tile_y = (float(self._dem.height())/20.0)/2
		counter = 0
		for i in self.paths:
			x_norm = []
			y_norm = []
			counter = counter +1
			for j in i.elements:
				x,y = self.convertToGridCoords(j,20,20)
				x_norm.append(int(float(x/20.0)*self._dem.width() + tile_x))
				y_norm.append(int(float(y/20.0)*self._dem.height() + tile_y))
			planeAddPaint(self.pathPlane, 200, x_norm, y_norm, QColor(211,211,211,30)) 
					#print x, y
	def avg_paths(self):
		first = []
		sec = []
		thir = []
		four = []
		fifth = []
		sixth = []
		lengths = []

		for i in range(0,len(self.paths)):
			if int(self.paths[i].reward) > self.bins[0]:
				first.append(len(self.paths[i].elements))
			if int(self.paths[i].reward) > self.bins[1]:
				sec.append(len(self.paths[i].elements))
			if int(self.paths[i].reward) > self.bins[2]:
				thir.append(len(self.paths[i].elements))
			if int(self.paths[i].reward) > self.bins[3]:
				four.append(len(self.paths[i].elements))
			if int(self.paths[i].reward) > self.bins[4]:
				fifth.append(len(self.paths[i].elements))
			if int(self.paths[i].reward) > self.bins[5]:
				sixth.append(len(self.paths[i].elements))
		lengths = [first, sec, thir, four, fifth, sixth]
		for i in range(self.num_options): 
			tiles = QTableWidgetItem("%.2f" % np.mean(lengths[i]))
			tiles.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
			self.table.setItem(i, 2, tiles)

	def count_rewards(self):
		first = []
		sec = []
		thir = []
		four = []
		fifth = []
		sixth = []
		lengths = []
		g1 = []
		g2 = []
		g3 = []
		g4 = []
		g5 = []
		g6 = []

		for i in range(0,len(self.rewards)):
			if self.rewards[i] > self.bins[0]:
				first.append(self.rewards[i])
				g1.append(self.sim_results[i])
			if self.rewards[i] > self.bins[1]:
				sec.append(self.rewards[i])
				g2.append(self.sim_results[i])
			if self.rewards[i] > self.bins[2]:
				thir.append(self.rewards[i])
				g3.append(self.sim_results[i])
			if self.rewards[i] > self.bins[3]:
				four.append(self.rewards[i])
				g4.append(self.sim_results[i])
			if self.rewards[i] > self.bins[4]:
				fifth.append(self.rewards[i])
				g5.append(self.sim_results[i])
			if self.rewards[i] > self.bins[5]:
				sixth.append(self.rewards[i])
				g6.append(self.sim_results[i])

		lengths = [first, sec, thir, four, fifth,sixth]
		goals = [g1,g2,g3,g4,g5,g6]

		for i in range(self.num_options): 
			traces = QTableWidgetItem(str(int(float(len(lengths[i]))/float((len(self.rewards)))*100)) + '%')
			try:
				goal = QTableWidgetItem(str(int(float(sum(goals[i]))/float((len(goals[i])))*100)) + '%')
			except:
				goal = QTableWidgetItem(str(0) + '%')
			avg = QTableWidgetItem(str(int(self.bins[i])))
			traces.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
			avg.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
			self.table.setItem(i, 3, traces)
			self.table.setItem(i, 4, avg)
			self.table.setItem(i, 5, goal)

	def drawIdeal(self):
		reward, actions = self.getPerf_client(self._goalID)
		y_min,y_max = self.hist.canvas.ax.get_ylim()

		self.hist.canvas.ax.axvline(x=reward, linestyle = '--', color = '#00FF70', linewidth = 4)
		self.hist.canvas.ax.plot(reward,0, marker = '^', markersize = 25, color = '#00FF70')
		self.hist.canvas.ax.plot(reward,y_max, marker = 'v', markersize = 25, color = '#00FF70')

		'''if reward in self.bins:
			reward += 150'''

		self.bins.append(reward)

	def convertToGridCoords(self,i, width, height):
		y = i//width
		x = i % width
		return x, y


	def _updateGoal(self):
		#Redraw the goal locations
		#print 'Updating goal locations'
		#If this is the first time we've seen this robot, create its icon
		if self._goalIcon is None:
			thisGoal = RobotWidget(str(self._goalID), QColor(self._colors[1][0], self._colors[1][1], self._colors[1][2]))
			thisGoal.setFont(QFont("SansSerif", max(self.h / 20.0,3), QFont.Bold))
			thisGoal.setBrush(QBrush(QColor(self._colors[1][0], self._colors[1][1], self._colors[1][2])))          
			self._goalIcon = thisGoal
			self.minimapScene.addItem(thisGoal)

		self.allGoals = zip(self.goal_titles, self.row) #Zip the tuples together so I get a list of tuples (instead of a tuple of lists)
		self.allGoals = sorted(self.allGoals, key=lambda param: param[0]) #Sort the combined list by the goal ID
		self.allGoalsDict = dict(self.allGoals)


		self._goalID = self.allGoals[self.count][0]
		self.setCurrentGoal_client(self._goalID)	


		self.msg.pose.position.x = self.allGoalsDict[self.allGoals[self.count-1][0]].x
		self.msg.pose.position.y = self.allGoalsDict[self.allGoals[self.count-1][0]].y
		self.current_state_pub.publish(self.msg)

		if self.count == 1:
			self.prev_btn.setEnabled(True)
			self.prev_btn.setStyleSheet('background-color: green; color: white')


		self.bins = self.getBins_client(self._goalID)
		self.bins = list(self.bins)
		self.rewards, self.sim_results = self.getRewards_client(self._goalID)

		self.max_reward = max(self.rewards)	


		try:
			#Update the label's text:
			self._goalIcon.setText(str(self._goalID))
			self.goal.setText('Current Goal: ' + self._goalID)
			#Pick up the world coordinates
			self._goalLocations = [self._goal[1], self._goal[2]]

			world = list(copy.deepcopy(self._goalLocations))

			iconBounds = self._goalIcon.boundingRect()

			world[0] = world[0]/self.demDownsample
			world[1] = world[1]/self.demDownsample
			
			#Adjust the world coords so that the icon is centered on the goal
			self.gworld[0] = world[0] - iconBounds.width()/2 
			self.gworld[1] = world[1] - iconBounds.height()/2 #mirror the y coord

			#        world[1] = self.h - (world[1] + iconBounds.height()/2) #mirror the y coord
			#        print 'Ymax:', self.h
			print 'Drawing goal ', self._goalID, ' at ', world
			self._goalIcon.setPos(QPointF(self.gworld[0], self.gworld[1]))
			self.robot_odom_changed.emit()

		except:
			pass

	def update_bins(self):

		labels = {-1: 'Very Bad', -0.5: 'Bad', -0.1: 'Fair', 0.1: 'Good', 0.5 : 'Very good'}
		outcome = []

		#Outcome Assessment
		for i in range(0,len((self.bins))):
			outcome.append(outcomeAssessment(np.array(self.rewards), self.bins[i]))
			if outcome[i]:
				value = labels[max([x for x in labels.keys() if x <= outcome[i]])]
				self.item_p = QTableWidgetItem(value)
			else: 
				#self.item_p = QTableWidgetItem('Guarenteed')
				self.cb_list[i].setEnabled(False)
				for j in range(0,self.table.columnCount()):
					blank = QTableWidgetItem('')
					blank.setFlags(Qt.ItemIsSelectable)
					self.table.setItem(i,j,blank)
			self.item_p.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
			self.table.setItem(i, 1, self.item_p)
			if outcome[i] > 0 and outcome[i] != None:
				self.table.item(i,1).setBackground(QtGui.QColor(0, 250, 0, 150*outcome[i]))
			elif outcome[i] < 0 and outcome[i] != None:
				self.table.item(i,1).setBackground(QtGui.QColor(250, 0, 0, 150*-1*outcome[i]))	
			else:
				pass

		for i in range(1,self.table.columnCount()):
			if i != 1:
				self.table.item(5,i).setBackground(QtGui.QColor(0, 255, 112, 250))

		self.table.update()


	def _update(self):
		if self._dem_item:
			self.minimapScene.removeItem(self._dem_item)

		pixmap = QPixmap.fromImage(self._dem)
		#pixmap = pixmap.scaled(self.sketchPlane.width(), self.sketchPlane.height())
		self._dem_item = self.minimapScene.addPixmap(pixmap) #.scaled(self.w*100,self.h*100))
		self._dem_item.setPos(QPointF(0, 0))

		scale = 0.5
		print self.w, self.h
		
		self.minimapScene.setSceneRect(0,0, self.w, self.h)
		

		self.minimapView.fitInView(self.minimapScene.sceneRect())

		#self.centerOn(self._dem_item)
		#self.show()
		bounds = self.minimapScene.sceneRect()
		print 'Bounds:', bounds
		#self.minimapView.setFixedSize(self.w, self.h)

		#Overlay the hazmap now that the dem is loaded
		self.hazmap_sub = rospy.Subscriber('hazmap', Image, self.hazmap_cb)

		#Get Goal Prepped ------------------------------------------------------
		self.goal_titles, self.row = self.getGoals_client()


		self.goals_changed.emit()
		self.beliefOpacitySlider.valueChanged.connect(self.sliderChanged);
		self.shotClock.start(1000) 



	def _updateRobot(self):
		#Redraw the robot locations
		location = self.allGoalsDict[self.allGoals[self.count-1][0]]

		#print 'Updating robot locations'
		#If this is the first time we've seen this robot, create its icon
		#if self._robotIcon == None:
		#	thisRobot = QArrow.QArrow(color=QColor(self._colors[0][0], self._colors[0][1], self._colors[0][2]))
		#	self._robotIcon = thisRobot
		#self.msg.pose.position.x = location.x
		#self.msg.pose.position.y = location.y
		#self.current_state_pub.publish(self.msg)
		world = list(copy.deepcopy([location.x,location.y]))

		iconBounds = self.thisRobot.boundingRect()

		world[0] = world[0]/self.demDownsample
		world[1] = world[1]/self.demDownsample
		
		#Adjust the world coords so that the icon is centered on the goal
		world[0] = world[0] - iconBounds.width()/2 
		world[1] = world[1] - iconBounds.height()/2 #mirror the y coord
		
		dx = (self.gworld[0] - world[0])
		dy = (self.gworld[1] - world[1])

		self.thisRobot.setTransformOriginPoint(QPoint(iconBounds.width()/2, iconBounds.height()/2))
		self.thisRobot.setPos(QPointF(world[0], world[1]))
		self.thisRobot.setRotation(math.atan(dy/dx)*180/math.pi -90) #Pointing directly at goal
		#try:
		#	self.thisRobot.setRotation(self.steer*180/math.pi) #First step policy advice
		#except:
		#	pass

	def hazmap_cb(self, msg):
		#Unlike the dem, the hazmap is pretty standard - gray8 image
		self.hazmap = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")

		self.hazmapImage = QImage(self.hazmap, msg.width, msg.height, QImage.Format_Grayscale8)
		self.hazmap_changed.emit(self.a)
	

	def _updateHazmap(self, a):
		if self.hazmapItem:
			self.minimapScene.removeItem(self.hazmapItem)
		print 'Rendering hazmap'
		
		hazTrans = QImage(self.hazmapImage.width(), self.hazmapImage.height(), QImage.Format_ARGB32)
		#hazTrans.fill(Qt.transparent)


		for row in range(0, self.hazmapImage.height()):
			for col in range(0, self.hazmapImage.width()):
				#Change the colormap to be clear for clear areas, red translucent for obstacles

				pixColor = self.hazmap[row,col]

				if pixColor == 0:
					#hazTrans.setPixelColor(col, row, QColor(255, 0, 0, 32))
					hazTrans.setPixel(col,row,qRgba(255,0,0,a))
				else:
					   #hazTrans.setPixelColor(col, row, QColor(0, 0, 0, 0))
					hazTrans.setPixel(col,row,qRgba(255,255,255,0))


		self.hazmapItem = self.minimapScene.addPixmap(QPixmap.fromImage(hazTrans)) #.scaled(self.w*100,self.h*100))
		self.hazmapItem.setPos(QPointF(0, 0))
		trans = QTransform()
		#print 'Translating by:', bounds.width()
		trans.scale(self.w/hazTrans.width(),self.h/hazTrans.height())
		#trans.translate(0, -bounds.height())
		self.hazmapItem.setTransform(trans)
		
		# Everything must be mirrored
		#self._mirror(self.hazmapItem)
		
	def dem_cb(self, msg):


		#self.resolution = msg.info.resolution
		self.w = msg.width
		self.h = msg.height

		print 'Got DEM encoded as:', msg.encoding
		print 'message length:', len(msg.data), 'type:', type(msg.data)
		print 'width:', msg.width
		print 'height:', msg.height
		
		a = np.array(struct.unpack('<%dd' % (msg.width*msg.height), msg.data), dtype=np.float64, copy=False, order='C')

   
		rawDEM = a.reshape((self.h, self.w))
		rawDEM = cv2.resize(rawDEM, (self.h//self.demDownsample, self.w//self.demDownsample), interpolation = cv2.INTER_LINEAR)
		self.h = rawDEM.shape[0]
		self.w = rawDEM.shape[1]

		#Scale to a 8-bit grayscale image:
		self.grayDEM = np.zeros(rawDEM.shape, dtype=np.uint8)
		minZ = np.min(np.min(rawDEM))
		maxZ = np.max(np.max(rawDEM))
		dynRange = maxZ - minZ

		print 'Max Z:', maxZ
		print 'Min Z:', minZ
		
		for i in range(0, self.h):
			for j in range(0, self.w):
				self.grayDEM[i][j] = (rawDEM[i][j] - minZ) * 255/dynRange

		#use OpenCV2 to interpolate the dem into something reasonably sized
		print 'Grayscale conversion complete'

		#Needs to be a class variable so that at QImage built on top of this numpy array has
		#a valid underlying buffer - local ars
		#self.resizedDEM = cv2.resize(self.grayDEM, (500,500), interpolation = cv2.INTER_LINEAR)

		print 'Image resize complete'
		
		self.h = self.grayDEM.shape[0]
		self.w = self.grayDEM.shape[1]
		image = QImage(self.grayDEM.reshape((self.h*self.w)), self.w, self.h, QImage.Format_Grayscale8)

		#        for i in reversed(range(101)):
		#            image.setColor(100 - i, qRgb(i* 2.55, i * 2.55, i * 2.55))
		#        image.setColor(101, qRgb(255, 0, 0))  # not used indices
		#        image.setColor(255, qRgb(200, 200, 200))  # color for unknown value -1

		self._dem = image       
		self.pathPlane = self.minimapScene.addPixmap(makeTransparentPlane(self._dem.width(), self._dem.height()))
		self.dem_changed.emit()
		
	def goal_cb(self, msg):
		 #Resolve the odometry to a screen coordinate for display

		worldX = msg.pose.x
		worldY = msg.pose.y

		print 'Got Goal at: ' + str(worldX) + ',' + str(worldY)

		self._goal = [msg.id, worldX, worldY]
		#self.goals_changed.emit()






def main():
		app = QApplication(sys.argv)
		coretools_app = SimulationWindow()
		signal.signal(signal.SIGINT, lambda *a: app.quit())
		sys.exit(app.exec_())


if __name__ == '__main__':
	main()
	try:
		main()
	except rospy.ROSInterruptException:
		pass

