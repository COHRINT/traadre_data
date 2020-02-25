#!/usr/bin/env python2.7

# Imports
from PyQt5 import QtGui, QtCore, QtWidgets
from PyQt5.QtWidgets import *; 
from PyQt5.QtGui import *;
from PyQt5.QtCore import *;

class SurveyWidget(QtWidgets.QWidget):
	def __init__(self, parent=None):
		QtWidgets.QWidget.__init__(self)   # Inherit from QWidget 
		self.setWindowModality(3)
		self.setWindowTitle('Provide Input')
		self.setStyleSheet("background-color:slategray;")
		self.setWindowFlags(QtCore.Qt.Window | QtCore.Qt.CustomizeWindowHint | QtCore.Qt.WindowTitleHint | QtCore.Qt.WindowStaysOnTopHint)
		self.center()

		
		self.layout = QtWidgets.QGridLayout()

		self.displayLayout = QGridLayout();
		self.pushLayout = QGridLayout();

		# Slider Group1 

		sliderGroup1 = QGroupBox()
		sliderGroup1.setStyleSheet("QGroupBox {background-color: white; border: 4px inset grey;}")

		hbox1 = QtWidgets.QHBoxLayout()
		hbox2 = QtWidgets.QHBoxLayout()
		hbox3 = QtWidgets.QHBoxLayout()
		hbox4 = QtWidgets.QHBoxLayout()
		hbox5 = QtWidgets.QHBoxLayout()
		vbox = QtWidgets.QVBoxLayout()
		s1  = QtWidgets.QSlider(Qt.Horizontal)
		s2  = QtWidgets.QSlider(Qt.Horizontal)
		s3  = QtWidgets.QSlider(Qt.Horizontal)
		s4  = QtWidgets.QSlider(Qt.Horizontal)
		s5  = QtWidgets.QSlider(Qt.Horizontal)

		self.s_list = [s1, s2, s3, s4, s5]
		self.h_list = [hbox1,hbox2,hbox3,hbox4,hbox5]
		for i in range(0,len(self.s_list)):
			self.s_list[i].setMinimum(0)
			self.s_list[i].setMaximum(5)
			self.s_list[i].setTickPosition(QSlider.TicksBelow)
			self.h_list[i].addWidget(self.s_list[i])


			vbox.addLayout(self.h_list[i])
		sliderGroup1.setLayout(vbox)

		self.layout.addWidget(sliderGroup1,0,0,10,4)

		# Submit
		Group = QGroupBox()
		Group.setLayout(self.pushLayout)

		Group.setStyleSheet("QGroupBox {background-color: white; border: 4px inset grey;}")

		self.submit_btn = QPushButton('Submit',self)
		self.pushLayout.addWidget(self.submit_btn,11,0,2,4); 

		self.layout.addWidget(Group,11,0,2,4)

		self.setLayout(self.layout)

	def center(self):
		'''Centers the window on the screen.'''
		resolution = QtWidgets.QDesktopWidget().screenGeometry()
		self.move((resolution.width()),
				  (resolution.height())) 



