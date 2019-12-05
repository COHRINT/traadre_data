#!/usr/bin/env python2.7

from PyQt5 import QtGui, QtCore, QtWidgets
from PyQt5.QtWidgets import *; 
from PyQt5.QtGui import *;
from PyQt5.QtCore import *;
import sys,os, os.path

import rospy
import signal
import QArrow
import math
import RobotIcon
from traadre_msgs.msg import *
from traadre_msgs.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *

import numpy as np
import cv2
import copy

from cv_bridge import CvBridge, CvBridgeError

def makeTransparentPlane(wind):
	
	scale = QPixmap('images/DEM.png')
	testMap = QPixmap(scale.size().width()/10,scale.size().height()/10); 
	testMap.fill(QtCore.Qt.transparent); 
	return testMap; 


class SimulationWindow(QWidget):
	sketch = pyqtSignal()
	defogSignal = pyqtSignal(int, int, int, int, list, int)
	cameraSketch = pyqtSignal()
	dem_changed = pyqtSignal()
	robot_odom_changed = pyqtSignal()
	goals_changed = pyqtSignal()
	hazmap_changed = pyqtSignal(int)
	rightClick = pyqtSignal(int,int)
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
		self._dem_item = None
		self._goal = None
		self._goalIcon = None
		self.gworld = [0,0]
		self.hazmapItem = None
		self._goalID = 'Start'
		self._robotIcon = None
		self.demDownsample = 4
		self.count = 0

		#Minimap ---------------------------
		self.minimapView = QGraphicsView(self); 
		self.minimapScene = QGraphicsScene(self);
		self.image =  QPixmap('images/DEM.png')

		#self.minimapScene.addPixmap(self.image); 
		#make sketchPlane --------------------
		#self.sketchPlane = self.minimapScene.addPixmap(makeTransparentPlane(self)); 

		self.minimapView.setScene(self.minimapScene);
		#self.minimapView.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		#self.minimapView.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		self.layout.addWidget(self.minimapView,1,1,2,8);


		#Add arrow
		self.thisRobot = QArrow.QArrow(color=QColor(0,150,0,200))
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

		#---------------------------------------------
		self.time = QLineEdit(); 

		timer = QTime()
		timer.start()
		self.time.setText(str(timer.elapsed()))
		self.time.setStyleSheet("background-color: white; border: 4px inset grey;")


		#self.layout.addWidget(self.time,5,20,1,3) 

		#----------------------------------------------
		self.stateLayout = QGridLayout();

		stateGroup = QGroupBox()
		stateGroup.setLayout(self.stateLayout)

		stateGroup.setStyleSheet("background-color: white; border: 4px inset grey;")

		self.fuel = QLabel()
		self.xP = QLabel()
		self.xQ = QLabel()
		self.goal = QLabel()

		self.fuel.setText('Fuel Remaining: 100%')
		self.xP.setText('Outcome Assessment: 0.1')
		self.xQ.setText('Solver Quality: 0.8')
		self.goal.setText('Current Goal: ' + self._goalID)

		self.stateLayout.addWidget(self.fuel,11,1,1,1); 
		self.stateLayout.addWidget(self.xQ,12,1,1,1);
		self.stateLayout.addWidget(self.goal,12,4,1,1);
		self.stateLayout.addWidget(self.xP,11,4,1,1); 
		self.layout.addWidget(stateGroup,10,1,1,8)

		#------------------------------------------
		self.pushLayout = QGridLayout();

		goGroup = QGroupBox()
		goGroup.setLayout(self.pushLayout)

		goGroup.setStyleSheet("QGroupBox {background-color: white; border: 4px inset grey;}")

		self.go_btn = QPushButton('Go',self)

		self.pushLayout.addWidget(self.go_btn,6,19,1,1); 
		self.go_btn.setStyleSheet("background-color: green; color: white")

		self.go_btn.clicked.connect(self.operator_toast)

		self.table = QTableWidget(9,5,self)

		self.table.setHorizontalHeaderLabels(('Goal ID', 'Pos X', 'Pos Y','Reward','Fuel Cost'))
		self.table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		#self.table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		self.table.setStyleSheet("background-color: grey")
		self.table.setMaximumWidth(self.table.horizontalHeader().length()+30)
		self.table.setMaximumHeight(self.table.verticalHeader().length()+30)


		self.pushLayout.addWidget(self.table,6,16,1,1); 
		self.layout.addWidget(goGroup,9,16,2,10)

	def sliderChanged(self):
		self.a = 255*self.beliefOpacitySlider.sliderPosition()/100
		self.hazmap_changed.emit(self.a)


	def operator_toast(self):
		toast = QInputDialog()
		self.val, okPressed = toast.getInt(self, "Confidence","Rate Your Confidence:", 1, 0, 5, 1)

		if okPressed:
			self.count = self.count +1
			self.goals_changed.emit()

	def make_connections(self): 
		#Handler for final sketches
		pass

	def buildTable(self):

		for i in range(0,self.table.rowCount()):
			self.item = QTableWidgetItem(self.allGoals[i][0])
			itemx = QTableWidgetItem( '%1.2f' % self.allGoals[i][1].x)
			itemy = QTableWidgetItem( '%1.2f' % self.allGoals[i][1].y)
			item_theta = QTableWidgetItem( '%1.2f' % self.allGoals[i][1].theta)
			item_fuel = QTableWidgetItem(str(100))
			self.item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
			itemx.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
			itemy.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
			item_theta.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
			item_fuel.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)

			self.table.setItem(i, 0, self.item)
			self.table.setItem(i, 1, itemx)
			self.table.setItem(i, 2, itemy)
			self.table.setItem(i, 3, item_theta)
			self.table.setItem(i, 4, item_fuel)

		self.goals_changed.emit()
	def getGoals_client(self):
		try:
			goal = rospy.ServiceProxy('/policy/policy_server/GetGoalList', GetGoalList)
			response = goal()
			row = response.goals

			return response.ids, row
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



		
		self._goalID = self.allGoals[self.count][0]
		print self._goalID
		self.setCurrentGoal_client(self._goalID)		



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


		self.goal_titles, self.row = self.getGoals_client()


		self.allGoals = zip(self.goal_titles, self.row) #Zip the tuples together so I get a list of tuples (instead of a tuple of lists)
		self.allGoals = sorted(self.allGoals, key=lambda param: param[0]) #Sort the combined list by the goal ID
		self.allGoalsDict = dict(self.allGoals)


		self.buildTable()
		self.beliefOpacitySlider.valueChanged.connect(self.sliderChanged); 


	def _updateRobot(self):
		#Redraw the robot locations
		location = self.allGoalsDict[self.allGoals[self.count-1][0]]
		print self.allGoals[self.count][1].x

		#print 'Updating robot locations'
		#If this is the first time we've seen this robot, create its icon
		#if self._robotIcon == None:
		#	thisRobot = QArrow.QArrow(color=QColor(self._colors[0][0], self._colors[0][1], self._colors[0][2]))
		#	self._robotIcon = thisRobot
		self.msg.pose.position.x = location.x
		self.msg.pose.position.y = location.y
		self.current_state_pub.publish(self.msg)
			
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
		#self.thisRobot.setRotation(math.atan(dy/dx)*180/math.pi -90) #Pointing directly at goal
		try:
			self.thisRobot.setRotation(self.steer*180/math.pi) #First step policy advice
		except:
			pass

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

