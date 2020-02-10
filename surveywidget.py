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
		self.center()

		
		self.layout = QtWidgets.QGridLayout()


		self.pushLayout = QGridLayout();

		# Slider Group1 
		sliderGroup1 = QGroupBox()
		sliderGroup1.setLayout(self.pushLayout)
		sliderGroup1.setStyleSheet("QGroupBox {background-color: white; border: 4px inset grey;}")

		self.layout.addWidget(sliderGroup1,0,0,4,10)

		# Slider Group2
		sliderGroup2 = QGroupBox()
		sliderGroup2.setLayout(self.pushLayout)
		sliderGroup2.setStyleSheet("QGroupBox {background-color: white; border: 4px inset grey;}")

		self.layout.addWidget(sliderGroup2,6,0,4,10)

		# Submit
		Group = QGroupBox()
		Group.setLayout(self.pushLayout)

		Group.setStyleSheet("QGroupBox {background-color: white; border: 4px inset grey;}")

		self.submit_btn = QPushButton('Submit',self)
		self.pushLayout.addWidget(self.submit_btn,6,19,1,4); 

		self.layout.addWidget(Group,4,16,1,2)



		self.setLayout(self.layout)

	def center(self):
		'''Centers the window on the screen.'''
		resolution = QtWidgets.QDesktopWidget().screenGeometry()
		self.move((resolution.width()),
				  (resolution.height())) 



