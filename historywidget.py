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
	def __init__(self, dem_item, hazmap_item):
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

		self.minimapView = QGraphicsView(self); 
		self.minimapScene = QGraphicsScene(self);
		self.minimapView.setScene(self.minimapScene);
		self.minimapView.setStyleSheet("background-color: beige; border: 4px inset grey;")


		self.layout.addWidget(self.minimapView,1,0,3,8);

		# Submit
		Group = QGroupBox()
		Group.setLayout(self.pushLayout)

		Group.setStyleSheet("QGroupBox {background-color: beige; border: 4px inset grey;}")

		self.submit_btn = QPushButton('OK',self)
		self.submit_btn.setStyleSheet(("background-color: green; color: white"))
		self.pushLayout.addWidget(self.submit_btn,9,0,2,5); 


		self.layout.addWidget(Group,9,0,1,8)

		self.minimapScene.addItem(dem_item)
		self.minimapScene.addItem(hazmap_item)
		#self.dem_sub = rospy.Subscriber('dem', Image, self.dem_cb)
		self.minimapView.fitInView(self.minimapScene.sceneRect())


	def center(self):
		'''Centers the window on the screen.'''
		resolution = QtWidgets.QDesktopWidget().screenGeometry()
		self.move((resolution.width()),
				  (resolution.height())) 


	def makeTransparentPlane(self, width, height):
		testMap = QPixmap(width,height); 
		testMap.fill(QColor(0,0,0,0)); 
		return testMap; 