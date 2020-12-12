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
from code.mplwidget import MplWidget

class SQComp(QtWidgets.QWidget):
	def __init__(self, rewards, vi_rewards):
		QtWidgets.QWidget.__init__(self)   # Inherit from QWidget 
		self.setWindowModality(3)
		self.setWindowTitle('Previous Traverse Results')
		self.setStyleSheet("background-color:slategray;")
		self.setWindowFlags(QtCore.Qt.Window | QtCore.Qt.CustomizeWindowHint | QtCore.Qt.WindowTitleHint | QtCore.Qt.WindowStaysOnTopHint)

		self.layout = QGridLayout()
		self.pushLayout = QGridLayout();
		self.setLayout(self.layout)

		for i in range(1,self.layout.rowCount()):
			self.layout.setRowStretch(i,1)
		for i in range(1,self.layout.columnCount()):
			self.layout.setColumnStretch(i,1)


		self.hist = self.makeHist(rewards,'Approximate Solver')
		self.hist2 = self.makeHist(vi_rewards, 'Trusted Solver')

		self.layout.addWidget(self.hist,0,0,3,4);
		self.layout.addWidget(self.hist2,4,0,3,4);


		# Submit
		Group = QGroupBox()
		Group.setLayout(self.pushLayout)

		Group.setStyleSheet("QGroupBox {background-color: beige; border: 4px inset grey;}")

		self.submit_btn = QPushButton('OK',self)
		self.submit_btn.setStyleSheet(("background-color: green; color: white"))
		self.pushLayout.addWidget(self.submit_btn); 


		self.layout.addWidget(Group,3,5,2,2)

	def makeHist(self, rewards, title):
		hist = MplWidget()
		hist.setStyleSheet("MplWidget {background-color: white; border: 4px inset grey;}")
		hist.canvas.draw()
		
		mu = "%.1f" % np.mean(rewards)
		std = "%.2f" % np.std(rewards)
		samples = len(rewards)

		hist.canvas.ax.set_xlim(min(rewards)-10,max(rewards)+10)

		hist.canvas.ax.set_xlabel('Accumulated Reward')
		hist.canvas.ax.set_ylabel('Samples out of ' + str(samples))
		hist.canvas.ax.set_title(title)


		hist.canvas.ax.hist(rewards, 50)
		return hist