#!/usr/bin/env python2.7

# Imports
from PyQt5 import QtGui, QtCore, QtWidgets
from PyQt5.QtWidgets import *; 
from PyQt5.QtGui import *;
from PyQt5.QtCore import *;
import rospy
from traadre_msgs.msg import *
from traadre_msgs.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
import numpy as np
import RobotIcon
import QCircle
import cv2
import copy
from OA import *
from SQ import *
from plane_functions import *
from cv_bridge import CvBridge, CvBridgeError


class HistoryWidget(QtWidgets.QWidget):
	def __init__(self, dem, hazmap, time, goalID, goalLoc, actions, reward, result,rewards,sq,delta):
		QtWidgets.QWidget.__init__(self)   # Inherit from QWidget 
		self.setWindowModality(3)
		self.setWindowTitle('Previous Traverse Results')
		self.setStyleSheet("background-color:slategray;")
		self.setWindowFlags(QtCore.Qt.Window | QtCore.Qt.CustomizeWindowHint | QtCore.Qt.WindowTitleHint | QtCore.Qt.WindowStaysOnTopHint)
		self.center()
		self.layout = QGridLayout()
		self.pushLayout = QGridLayout();
		self.setLayout(self.layout)

		for i in range(1,self.layout.rowCount()):
			self.layout.setRowStretch(i,1)
		for i in range(1,self.layout.columnCount()):
			self.layout.setColumnStretch(i,1)

		self.demDownsample = 4
		self.hazmapItem = None
		self.h = 4097
		self.w = 4097
		self._colors = [(125, 0, 125), (68, 134, 252), (236, 228, 46), (102, 224, 18), (242, 156, 6), (240, 64, 10), (196, 30, 250)]

		self.minimapView = QGraphicsView(self); 
		self.minimapScene = QGraphicsScene(self);
		self.minimapView.setScene(self.minimapScene);
		self.minimapView.setStyleSheet("background-color: beige; border: 4px inset grey;")


		self.layout.addWidget(self.minimapView,1,0,4,7);



		self._dem_item = self.minimapScene.addPixmap(dem)
		self.hazmapItem = self.minimapScene.addPixmap(hazmap)
		self.hazmapItem.setPos(QPointF(0, 0))
		self._dem_item.setPos(QPointF(0, 0))

		#print 'Translating by:', bounds.width()

		htrans = QTransform()
		dtrans = QTransform()
		dtrans.scale(0.71,0.68)
		htrans.scale(36,34.5)
		self.hazmapItem.setTransform(htrans)
		self._dem_item.setTransform(dtrans)
		self.minimapView.setSceneRect(self.minimapScene.sceneRect())

		#-------------------------------------------------------------------------
		self.stateLayout = QGridLayout();

		stateGroup = QGroupBox()
		stateGroup.setLayout(self.stateLayout)

		stateGroup.setStyleSheet("background-color: beige; border: 4px inset grey; font: 15pt Lato")
		self.current_score = 0
		#self.solver_quality = solverQuality(np.array(rewards), np.array(vi_rewards))


		self.outcome_assessment = outcomeAssessment(np.array(rewards),reward)
		labels = {-1: 'Very Bad', -0.5: 'Bad', -0.1: 'Fair', 0.1: 'Good', 0.5 : 'Very good'}

		value = labels[max([x for x in labels.keys() if x <= self.outcome_assessment])]

		self.reward = QLabel()
		self.reward.setStyleSheet("background-color: white")
		self.oa = QLabel()
		self.oa.setStyleSheet("background-color: white")
		self.sq = QLabel()
		self.sq.setStyleSheet("background-color: white")
		self.timer = QLabel()
		self.timer.setStyleSheet("background-color: white")
		self.goal = QLabel()

		self.goal.setStyleSheet("background-color: white")
		self.score = QLabel()
		#self.score.setToolTip("Current score")
		self.score.setStyleSheet("background-color: white")

		self.oa.setText('Outcome: ' + value)

		'''if self.outcome_assessment > 0 and self.outcome_assessment != None:
			color = QtGui.QColor(0, 250, 0, 150*self.outcome_assessment)
		elif self.outcome_assessment < 0 and self.outcome_assessment != None:
			color = QtGui.QColor(250, 0, 0, 150*-1*self.outcome_assessment)
		values = "{r}, {g}, {b}, {a}".format(r = 250,g = 0,b = 0,a = 20)
		self.oa.setStyleSheet("background-color:rgba("+values+"")'''

		self.current_score = delta


		self.sq.setText('Solver: ' + str(sq))
		self.reward.setText('Accumulated Reward: ' + str(int(reward)))
		self.goal.setText('Goal: '+ goalID)
		self.timer.setText('Time Remaining: ' + str(time) + ' seconds')
		self.score.setText('Score Earned: ' + str(self.current_score))

		self.stateLayout.addWidget(self.reward,12,1,1,1); 
		self.stateLayout.addWidget(self.goal,11,1);
		self.stateLayout.addWidget(self.timer,11,2); 
		self.stateLayout.addWidget(self.score,12,2); 
		self.stateLayout.addWidget(self.oa,13,1); 
		self.stateLayout.addWidget(self.sq,13,2); 

		self.layout.addWidget(stateGroup,10,0,4,7)

		# Submit
		Group = QGroupBox()
		Group.setLayout(self.pushLayout)

		Group.setStyleSheet("QGroupBox {background-color: beige; border: 4px inset grey;}")

		self.submit_btn = QPushButton('OK',self)
		self.submit_btn.setStyleSheet(("background-color: green; color: white"))
		self.pushLayout.addWidget(self.submit_btn,13,0,2,3); 


		self.layout.addWidget(Group,15,1,2,5)

		#------------------------------
		self.drawLetter(goalID,goalLoc)
		self.pathPlane = self.minimapScene.addPixmap(makeTransparentPlane(self.minimapScene.width(), self.minimapScene.height()))
		self.draw_paths(actions,result)

		#-----------------------------
		'''self.thisRobot = QCircle.QArrow(color=QColor(0,250,0,255))
		self.minimapScene.addItem(self.thisRobot);
		self.thisRobot.setZValue(2)'''

	def center(self):
		'''Centers the window on the screen.'''
		resolution = QtWidgets.QDesktopWidget().screenGeometry()
		self.move((resolution.width()),
				  (resolution.height())) 

	def drawLetter(self,goalID,goalLoc):
		self.gworld = [0,0]

		thisGoal = RobotIcon.RobotWidget(str(goalID), QColor(self._colors[1][0], self._colors[1][1], self._colors[1][2]))
		thisGoal.setFont(QFont("SansSerif", max(1300 / 36.0,3), QFont.Bold))
		thisGoal.setBrush(QBrush(QColor(self._colors[1][0], self._colors[1][1], self._colors[1][2])))          
		self._goalIcon = thisGoal
		self.minimapScene.addItem(self._goalIcon)

		#Update the label's text:
		self._goalIcon.setText(str(goalID))
		#Pick up the world coordinates
		self._goalLocations = [goalLoc.x, goalLoc.y]

		world = list(copy.deepcopy(self._goalLocations))

		iconBounds = self._goalIcon.boundingRect()

		world[0] = (world[0]/self.w)*self.minimapScene.width()
		world[1] = (world[1]/self.h)*self.minimapScene.height()
		
		#Adjust the world coords so that the icon is centered on the goal
		self.gworld[0] = (world[0])- iconBounds.width()/2 
		self.gworld[1] = (world[1])- iconBounds.height()/2 #mirror the y coord

		self._goalIcon.setPos(QPointF(self.gworld[0], self.gworld[1]))


	def draw_paths(self, actions, result):
		planeFlushPaint(self.pathPlane)
		tile_x = (float(self.minimapScene.width())/20.0)/2
		tile_y = (float(self.minimapScene.height())/20.0)/2
		counter = 0
		x_norm = []
		y_norm = []

		for j in actions:
			x,y = self.convertToGridCoords(j,20,20)
			x_norm.append(int(float(x/20.0)*self.minimapScene.width() + tile_x))
			y_norm.append(int(float(y/20.0)*self.minimapScene.height() + tile_y))
		if result == 1:
			planeAddPaint(self.pathPlane, 200, x_norm, y_norm, QColor(0,251,0,60))
		else:  
			planeAddPaint(self.pathPlane, 200, x_norm, y_norm, QColor(251,0,0,60))
					#print x, y

		self.pathPlane.setZValue(2)

	def convertToGridCoords(self,i, width, height):
		y = i//width
		x = i % width
		return x, y

