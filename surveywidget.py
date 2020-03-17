#!/usr/bin/env python2.7

# Imports
from PyQt5 import QtGui, QtCore, QtWidgets
from PyQt5.QtWidgets import *; 
from PyQt5.QtGui import *;
from PyQt5.QtCore import *;
import rospy
from traadre_msgs.msg import *

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
		self.survey_pub = rospy.Publisher('survey', Survey, queue_size=10)

		# Slider Group1 

		sliderGroup1 = QGroupBox()
		sliderGroup1.setStyleSheet("QGroupBox {background-color: beige; border: 4px inset grey;}")

		hbox1 = QtWidgets.QVBoxLayout()
		hbox2 = QtWidgets.QVBoxLayout()
		hbox3 = QtWidgets.QVBoxLayout()
		hbox4 = QtWidgets.QVBoxLayout()
		hbox5 = QtWidgets.QVBoxLayout()
		vbox = QtWidgets.QVBoxLayout()
		s1  = QtWidgets.QSlider(Qt.Horizontal)
		s2  = QtWidgets.QSlider(Qt.Horizontal)
		s3  = QtWidgets.QSlider(Qt.Horizontal)
		s4  = QtWidgets.QSlider(Qt.Horizontal)
		s5  = QtWidgets.QSlider(Qt.Horizontal)
		q1 = QLabel()
		q2 = QLabel()
		q3 = QLabel()
		q4 = QLabel()
		q5 = QLabel()
		self.msg = Survey()

		self.q_list = [q1, q2, q3, q4, q5]
		self.qs_list = ['To what extent was the rover able to reach its destination?', 'To what extent do you feel you were able to predict the rovers performance from task to task?', 'To what extent could you rely on the rover to perform its job?', 'To what extent did the rover perform similarly in related tasks?', 'In future interaction, to what extent would you trust the rover?']
		self.msg.questions = self.qs_list
		self.s_list = [s1, s2, s3, s4, s5]
		self.h_list = [hbox1,hbox2,hbox3,hbox4,hbox5]
		for i in range(0,len(self.s_list)):
			self.s_list[i].setMinimum(0)
			self.s_list[i].setMaximum(5)
			self.s_list[i].setTickPosition(QSlider.TicksBelow)
			self.s_list[i].setStyleSheet("background-color: beige; color: white")
			self.q_list[i].setText(self.qs_list[i])
			self.q_list[i].setStyleSheet("background-color: beige; color: black")
			self.h_list[i].addWidget(self.q_list[i])
			self.h_list[i].addWidget(self.s_list[i])



			vbox.addLayout(self.h_list[i])
		sliderGroup1.setLayout(vbox)

		self.layout.addWidget(sliderGroup1,0,0,10,4)

		# Submit
		Group = QGroupBox()
		Group.setLayout(self.pushLayout)

		Group.setStyleSheet("QGroupBox {background-color: beige; border: 4px inset grey;}")

		self.submit_btn = QPushButton('Submit',self)
		self.submit_btn.setStyleSheet(("background-color: green; color: white"))
		self.submit_btn.clicked.connect(self.submit)
		self.pushLayout.addWidget(self.submit_btn,11,0,2,4); 


		self.layout.addWidget(Group,11,0,2,4)

		self.setLayout(self.layout)

	def center(self):
		'''Centers the window on the screen.'''
		resolution = QtWidgets.QDesktopWidget().screenGeometry()
		self.move((resolution.width()),
				  (resolution.height())) 



	def submit(self):
		answers = []
		for i in range(len(self.q_list)):
			answers.append(self.s_list[i].value())

		self.msg.answers = answers
		self.survey_pub.publish(self.msg)