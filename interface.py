#!/usr/bin/env python2.7

from PyQt5 import QtGui, QtCore, QtWidgets
from PyQt5.QtWidgets import *; 
from PyQt5.QtGui import *;
from PyQt5.QtCore import *;
import sys,os, os.path

import rospy
import signal
import QCircle
import math
import RobotIcon
from traadre_msgs.msg import *
from traadre_msgs.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np

import cv2
import copy
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError

from mplwidget import MplWidget
from surveywidget import SurveyWidget
from historywidget import HistoryWidget

def outcomeAssessment(samples, R_inf):
	L_samples=np.unique(np.where(samples<R_inf))
	U_samples=np.unique(np.where(samples>R_inf))
	if not L_samples.any():
		return None
	samples=list(samples)
	LPM=sum([float(samples[x]*samples.count(samples[x]))/float(len(samples)) for x in L_samples])
	UPM=sum([float(samples[x]*samples.count(samples[x]))/float(len(samples)) for x in U_samples])
	xO=(float(2)/(1+np.exp(-np.log(float(UPM)/float(LPM)))))-1

	return xO

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

	def __init__(self):

		super(SimulationWindow,self).__init__()

		self.layout = QGridLayout(); 
		self.setLayout(self.layout); 


		#self.setGeometry(300, 300, 250, 150)
		self.setWindowTitle("DLA BluePrint");
		self.setStyleSheet("background-color:slategray;")
		self.populateInterface(); 
		self.populated = True


		self.make_connections();
		self.showMaximized();
		print 'Columns: ' + str(self.layout.columnCount())
		print 'Rows: ' + str(self.layout.rowCount())
		self.setWindowState(Qt.WindowMaximized);
		self.dem_sub = rospy.Subscriber('dem', Image, self.dem_cb)
		self.steer_sub = rospy.Subscriber('current_steer', Steering, self.state_callback)
		self.goal_sub = rospy.Subscriber('current_goal', NamedGoal, self.goal_cb)
		self.option_sub = rospy.Subscriber('option', OptionSelect, self.option_cb)
		self.current_state_pub = rospy.Publisher('state', RobotState, queue_size=10)

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
		self._dem_item = None
		self._goalIcon = None
		self.gworld = [0,0]
		self.hazmapItem = None
		self.current_score = 0
		self._goalID = 'Start'
		self._robotIcon = None
		self.demDownsample = 4
		self.count = 0
		self.num_options = 5
		self.paths = None


		#Minimap ---------------------------
		self.minimapView = QGraphicsView(self); 
		self.minimapScene = QGraphicsScene(self);
		self.image =  QPixmap('images/DEM.png')


		self.minimapView.setScene(self.minimapScene);
		#self.minimapView.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		#self.minimapView.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		self.minimapView.setStyleSheet("background-color: beige; border: 4px inset grey;")
		self.layout.addWidget(self.minimapView,1,1,2,8);


		#Add circle ---------------------------------------------
		self.thisRobot = QCircle.QArrow(color=QColor(0,250,0,255))
		self.minimapScene.addItem(self.thisRobot);
		self.thisRobot.setZValue(2)


		#Hazard slider --------------------------------
		sliderLayout = QGridLayout(); 
		self.beliefOpacitySlider = QSlider(Qt.Horizontal); 
		self.beliefOpacitySlider.setSliderPosition(30)
		self.beliefOpacitySlider.setTickPosition(QSlider.TicksBelow)
		self.beliefOpacitySlider.setTickInterval(10); 

		sliderLayout.addWidget(self.beliefOpacitySlider,0,0);

		self.layout.addLayout(sliderLayout,9,1,1,8) 

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
		self.layout.addWidget(stateGroup,10,1,1,8)

		#self.timer.setToolTip("Time remaining to make decision, at 0 there will be a penalty")
		#self.timer.setStyleSheet("""QToolTip { background-color: black; color: white; border: black solid 1px }""")
		#self.goal.setToolTip("Current navigation goal from [A,F]")

		#------------------------------------------
		self.pushLayout = QGridLayout();

		goGroup = QGroupBox()
		goGroup.setLayout(self.pushLayout)

		goGroup.setStyleSheet("QGroupBox {background-color: beige; border: 4px inset grey;}")

		self.go_btn = QPushButton('Go',self)
		self.go_btn.setEnabled(False)
		self.pushLayout.addWidget(self.go_btn,5,19,1,4); 
		self.go_btn.setStyleSheet("background-color: grey; color: white")

		self.no_btn = QPushButton('No Go',self)
		self.no_btn.setEnabled(False)
		self.pushLayout.addWidget(self.no_btn,6,19,1,4); 
		self.no_btn.setStyleSheet("background-color: grey; color: white")

		self.info_btn = QPushButton('Explain',self)
		self.info_btn.setEnabled(False)
		self.pushLayout.addWidget(self.info_btn,7,19,1,4); 
		self.info_btn.setStyleSheet("background-color: grey; color: white")

		self.prev_btn = QPushButton('Previous Traverse',self)
		self.prev_btn.setEnabled(False)
		self.pushLayout.addWidget(self.prev_btn,8,19,1,4); 
		self.prev_btn.setStyleSheet("background-color: grey; color: white")
		

		self.go_btn.clicked.connect(self.operator_toast)
		self.no_btn.clicked.connect(self.operator_toast)
		self.prev_btn.clicked.connect(self.traverse_history)

		#--------------------------------------------------------------------
		self.table = QTableWidget(self.num_options,6,self)

		self.table.setHorizontalHeaderLabels(('Toggle', 'Outcome', 'Solver','Reward','Fuel Cost', 'Avg. Tiles'))
		self.table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		self.table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		self.table.setStyleSheet("background-color: rgb(192,192,192)")
		self.table.setMaximumWidth(self.table.horizontalHeader().length()+25)
		self.table.setMaximumHeight(self.table.verticalHeader().length()+25)


		self.pushLayout.addWidget(self.table,5,16,3,1); 
		self.layout.addWidget(goGroup,9,16,2,10)


		#-----------------------------------------------------
		self.histLayout = QGridLayout();

		histGroup = QGroupBox()
		histGroup.setLayout(self.histLayout)

		histGroup.setStyleSheet("QGroupBox {background-color: beige; border: 4px inset grey;}")
		histGroup.setToolTip('Choose one of reward distribution bins')

		self.layout.addWidget(histGroup, 1,17,2,8)



	def sliderChanged(self):
		self.a = 255*self.beliefOpacitySlider.sliderPosition()/100
		self.hazmap_changed.emit(self.a)

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

	def option_cb(self, data):
		self.option_changed.emit(data.option, data.boundary)

	def _updateOption(self, option, boundary):

		if boundary == False:
			self.table.clearSelection()
		else:
			self.table.selectRow(option)
		'''self.planeFlushPaint(self.pathPlane)
		x_tmp = []
		y_tmp = []
		tile_x = (float(self._dem.width())/20.0)/2
		tile_y = (float(self._dem.height())/20.0)/2
		for i in self.pathDict.keys():
			band = self.max_reward*float(1.0/6.0)*option
			if int(i) > band:
				for j in self.pathDict[i]:
					x,y = self.convertToGridCoords(j,20,20)
					x_tmp.append(int(float(x/20.0)*self._dem.width() + tile_x))
					y_tmp.append(int(float(y/20.0)*self._dem.height() + tile_y))
					self.planeAddPaint(self.pathPlane, 200, x_tmp, y_tmp, QColor(250,251,0,50)) '''

	def operator_toast(self):
		print "Opening a new popup window..."
		self.setEnabled(False)
		self.survey = SurveyWidget()
		self.survey.setGeometry(QRect(100, 100, 800, 800))
		self.survey.submit_btn.clicked.connect(self.survey_submit)
		self.survey.show()

	def traverse_history(self):
		print "Opening a new popup window..."
		self.setEnabled(False)
		self.trav_hist = HistoryWidget()
		self.trav_hist.setGeometry(QRect(100, 100, 800, 800))
		self.trav_hist.submit_btn.clicked.connect(self.trav_submit)
		self.trav_hist.show()


	def trav_submit(self):
		self.trav_hist.close()
		self.setEnabled(True)

	def survey_submit(self):
		self.time_remaining = 120
		self.timer.setText('Time Remaining: ' + str(self.time_remaining) + ' seconds')
		self.count = self.count +1
		self.buildTable()
		self.go_btn.setEnabled(False)
		self.go_btn.setStyleSheet("background-color: grey; color: white")
		self.survey.close()
		self.setEnabled(True)

	def make_connections(self): 
		'''self.minimapView.mousePressEvent = lambda event:imageMousePress(event,self); 
		self.minimapView.mouseMoveEvent = lambda event:imageMouseMove(event,self); 
		self.minimapView.mouseReleaseEvent = lambda event:imageMouseRelease(event,self);

		self.goGroup.mousePressEvent = lambda event:imageMousePress(event,self); 
		self.goGroup.mouseMoveEvent = lambda event:imageMouseMove(event,self); 
		self.goGroup.mouseReleaseEvent = lambda event:imageMouseRelease(event,self);

		self.table.mousePressEvent = lambda event:imageMousePress(event,self); 
		self.table.mouseMoveEvent = lambda event:imageMouseMove(event,self); 
		self.table.mouseReleaseEvent = lambda event:imageMouseRelease(event,self);'''
		pass
		
	def checkbox_callback(self):
		count = 0
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
			self.no_btn.setStyleSheet("background-color: grey; color: white")

	def buildTable(self):
		self.goals_changed.emit()

		cb1  = QtWidgets.QCheckBox( parent=self.table )
		cb2  = QtWidgets.QCheckBox( parent=self.table )
		cb3  = QtWidgets.QCheckBox( parent=self.table )
		cb4  = QtWidgets.QCheckBox( parent=self.table )
		cb5  = QtWidgets.QCheckBox( parent=self.table )

		self.cb_list = [cb1, cb2, cb3, cb4, cb5]
		for i in range(0,self.table.rowCount()):
			self.cb_list[i]  = QtWidgets.QCheckBox( parent=self.table )
			self.cb_list[i].setTristate(False)
			self.cb_list[i].setChecked(1)
			self.cb_list[i].stateChanged.connect(self.checkbox_callback)

			self.item_p = QTableWidgetItem(str(0))
			item_q = QTableWidgetItem(str(0.0))
			self.table.setToolTip('Choose one of the navigation options')
			item_reward = QTableWidgetItem(str(50))
			item_fuel = QTableWidgetItem(str(100 + 10*i))
			
			item_q.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
			item_reward.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
			item_fuel.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)

			self.table.setCellWidget(i,0, self.cb_list[i])
			self.table.setItem(i, 2, item_q)
			self.table.setItem(i, 3, item_reward)
			self.table.setItem(i, 4, item_fuel)

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

			return response.rewards
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def getPaths_client(self, id):
		try:
			goal = rospy.ServiceProxy('/policy/policy_server/GetPaths', GetPaths)
			response = goal(id)

			return response.paths
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

	def makeHist(self):
		if self.count == 0:
			self.hist = MplWidget()
			self.hist.setStyleSheet("MplWidget {background-color: white; border: 4px inset grey;}")


			self.hist.canvas.draw()

			self.histLayout.addWidget(self.hist)

		#Histogram stuff
		self.hist.canvas.ax.clear()
		self.rewards = self.getRewards_client(self._goalID)	

		mu = "%.1f" % np.mean(self.rewards)
		std = "%.2f" % np.std(self.rewards)
		self.max_reward = max(self.rewards)
		samples = len(self.rewards)

		self.hist.canvas.ax.set_xlabel('Accumulated Reward')
		self.hist.canvas.ax.set_ylabel('Samples out of ' + str(samples))
		self.hist.canvas.ax.set_title(r'Histogram of IQ: $\mu=100$, $\sigma=15$')


		self.hist.canvas.ax.hist(self.rewards, 50)



		self.hist.canvas.ax.axvline(x=self.max_reward*(1.0/float(self.num_options+1)), linestyle = '--', color = 'red')
		self.hist.canvas.ax.axvline(x=self.max_reward*(2.0/float(self.num_options+1)), linestyle = '--', color = 'red')
		self.hist.canvas.ax.axvline(x=self.max_reward*(3.0/float(self.num_options+1)), linestyle = '--', color = 'red')
		self.hist.canvas.ax.axvline(x=self.max_reward*(4.0/float(self.num_options+1)), linestyle = '--', color = 'red')
		self.hist.canvas.ax.axvline(x=self.max_reward*(5.0/float(self.num_options+1)), linestyle = '--', color = 'red')

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
		self.paths = self.getPaths_client(self._goalID)	
		self.planeFlushPaint(self.pathPlane)
		tile_x = (float(self._dem.width())/20.0)/2
		tile_y = (float(self._dem.height())/20.0)/2
		memory = []
		counter = 0
		self.pathDict = {}
		for i in self.paths:
			x_norm = []
			y_norm = []
			counter = counter +1
			if i not in memory:
				self.pathDict[str(i.reward)] = (i.elements)
				memory.append(i)
				for j in i.elements:
					x,y = self.convertToGridCoords(j,20,20)
					x_norm.append(int(float(x/20.0)*self._dem.width() + tile_x))
					y_norm.append(int(float(y/20.0)*self._dem.height() + tile_y))
					self.planeAddPaint(self.pathPlane, 200, x_norm, y_norm, QColor(0,251,0,5)) 
					#print x, y

		self.pathPlane.setZValue(2)

	def avg_paths(self):
		band = self.max_reward*float(1.0/6.0)
		first = []
		sec = []
		thir = []
		four = []
		fifth = []
		lengths = []
		for i in self.pathDict.keys():
			if int(i) > band:
				first.append(len(self.pathDict[i]))
			if int(i) > 2*band:
				sec.append(len(self.pathDict[i]))
			if int(i) > 3*band:
				thir.append(len(self.pathDict[i]))
			if int(i) > 4*band:
				four.append(len(self.pathDict[i]))
			if int(i) > 5*band:
				fifth.append(len(self.pathDict[i]))
		lengths = [first, sec, thir, four, fifth]
		for i in range(self.num_options): 
			value = QTableWidgetItem("%.2f" % np.mean(lengths[i]))
			value.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
			self.table.setItem(i, 5, value)

	def convertToGridCoords(self,i, width, height):
		y = i//width
		x = i % width
		return x, y

	def makeTransparentPlane(self, width, height):
		testMap = QPixmap(width,height); 
		testMap.fill(QColor(0,0,0,0)); 
		return testMap; 

	'''def planeAddPaint(self,planeWidget,value,x,y,col=None,pen=None):
		pm = planeWidget.pixmap(); 
		pm.toImage()
		painter = QPainter(pm); 

		if(pen is None):
			if(col is None):
				pen = QPen(QColor(0,0,150,value)); 
			else:
				pen = QPen(col); 
		pen.setWidth(5)
		painter.setPen(pen)
		painter.drawPoint(x,y); 
		painter.end(); 
		planeWidget.setPixmap(pm); '''


	def planeAddPaint(self,planeWidget,value,x,y,col,pen=None):
		pm = planeWidget.pixmap(); 
		pm.toImage()
		painter = QPainter(pm); 

		if(pen is None):
			if(col is None):
				pen = QPen(QColor(0,0,150,value)); 
			else:
				pen = QPen(col); 
		pen.setWidth(5)
		painter.setPen(pen)
		for p in range(len(x)-1):
			painter.drawLine(x[p],y[p],x[p+1],y[p+1]); 
		painter.end(); 
		planeWidget.setPixmap(pm); 

	def planeFlushPaint(self,planeWidget,col = None,pen=None):
		pm = planeWidget.pixmap(); 
		pm.fill(QColor(0,0,0,0)); 

		painter = QPainter(pm); 
		if(pen is None):
			if(col is None):
				pen = QPen(QColor(0,0,0,255)); 
			else:
				pen = QPen(col); 
		painter.setPen(pen)

		painter.end(); 
		planeWidget.setPixmap(pm); 

	def _updateGoal(self):
		#Redraw the goal locations
		#print 'Updating goal locations'
		#If this is the first time we've seen this robot, create its icon
		if self._goalIcon is None:
			thisGoal = RobotIcon.RobotWidget(str(self._goalID), QColor(self._colors[1][0], self._colors[1][1], self._colors[1][2]))
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

		self.makeHist()
		self.draw_paths()
		self.avg_paths()

		
		#Outcome Assessment
		for i in range(1,self.table.rowCount()+1):
			outcome = outcomeAssessment(np.array(self.rewards), self.max_reward*0.16*(i))
			if outcome:
				self.item_p = QTableWidgetItem(str("%.5f" % outcome))
			else: 
				self.item_p = QTableWidgetItem('N/A')
			self.item_p.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
			self.table.setItem(i-1, 1, self.item_p)
			if outcome > 0:
				self.table.item(i-1,1).setBackground(QtGui.QColor(0, 250, 0, 150*outcome))
			else:
				self.table.item(i-1,1).setBackground(QtGui.QColor(250, 0, 0, 150*-1*outcome))	


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
		bounds = self.minimapView.sceneRect()
		print 'Bounds:', bounds
		#self.minimapView.setFixedSize(self.w, self.h)

		#Overlay the hazmap now that the dem is loaded
		self.hazmap_sub = rospy.Subscriber('hazmap', Image, self.hazmap_cb)

		#Get Goal Prepped ------------------------------------------------------
		self.goal_titles, self.row = self.getGoals_client()


		self.buildTable()
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
		self.pathPlane = self.minimapScene.addPixmap(self.makeTransparentPlane(self._dem.width(), self._dem.height()))
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

