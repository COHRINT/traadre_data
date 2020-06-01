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
import cv2
import copy
from cv_bridge import CvBridge, CvBridgeError


class HistoryWidget(QtWidgets.QWidget):
	def __init__(self, dem, hazmap):
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
		self.time_remaining = 120
		self.current_score = 0

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
		self.score.setToolTip("Current score")
		self.score.setStyleSheet("background-color: white")

		self.oa.setText('Outcome: ')
		self.sq.setText('Solver: ')
		self.reward.setText('Accumulated Reward: ')
		self.goal.setText('Goal: ')
		self.timer.setText('Time Used: ' + str(self.time_remaining) + ' seconds')
		self.score.setText('Score Achieved: ' + str(self.current_score))

		self.stateLayout.addWidget(self.reward,12,1,1,1); 
		self.stateLayout.addWidget(self.goal,11,1);
		self.stateLayout.addWidget(self.timer,11,2); 
		self.stateLayout.addWidget(self.score,12,2); 
		self.stateLayout.addWidget(self.oa,13,1); 
		self.stateLayout.addWidget(self.sq,13,2); 

		self.layout.addWidget(stateGroup,10,0,4,7)

		print self.layout.columnCount()
		print self.layout.rowCount()
		# Submit
		Group = QGroupBox()
		Group.setLayout(self.pushLayout)

		Group.setStyleSheet("QGroupBox {background-color: beige; border: 4px inset grey;}")

		self.submit_btn = QPushButton('OK',self)
		self.submit_btn.setStyleSheet(("background-color: green; color: white"))
		self.pushLayout.addWidget(self.submit_btn,13,0,2,3); 


		self.layout.addWidget(Group,15,1,2,5)

	def center(self):
		'''Centers the window on the screen.'''
		resolution = QtWidgets.QDesktopWidget().screenGeometry()
		self.move((resolution.width()),
				  (resolution.height())) 


	def makeTransparentPlane(self, width, height):
		testMap = QPixmap(width,height); 
		testMap.fill(QColor(0,0,0,0)); 
		return testMap; 