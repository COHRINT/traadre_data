#!/usr/bin/env python2.7

from PyQt5 import QtGui, QtCore, QtWidgets
from PyQt5.QtWidgets import *; 
from PyQt5.QtGui import *;
from PyQt5.QtCore import *;
import sys,os, os.path

import rospy
import signal



def makeTransparentPlane(wind):
	
	scale = QPixmap('images/DEM.png')
	testMap = QPixmap(scale.size().width(),scale.size().height()); 
	testMap.fill(QtCore.Qt.transparent); 
	return testMap; 


class SimulationWindow(QWidget):
	sketch = pyqtSignal()
	defogSignal = pyqtSignal(int, int, int, int, list, int)
	cameraSketch = pyqtSignal()
	dronePixMap = pyqtSignal(QImage)
	beliefPixMap = pyqtSignal(QImage)
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

		#self.setWindowState(Qt.WindowMaximized);

		# self.populated = False
		#self.show();


		rospy.init_node('interface')

		rate = rospy.Rate(10) # 10hz
		# self.sketchPub = rospy.Publisher('/Sketch', sketch, queue_size=10)


	def populateInterface(self):

		#Minimap ---------------------------
		self.minimapView = QGraphicsView(self); 
		self.minimapScene = QGraphicsScene(self);
		image =  QPixmap('images/DEM.png')

		self.minimapScene.addPixmap(image); 
		self.minimapView.setScene(self.minimapScene);
		self.minimapView.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		self.minimapView.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

		#make sketchPlane --------------------
		self.sketchPlane = makeTransparentPlane(self); 

		self.layout.addWidget(self.minimapView,1,1,5,5);


		#------------------------------------------------
		self.pushLayout = QGridLayout();

		goGroup = QGroupBox()
		goGroup.setLayout(self.pushLayout)

		goGroup.setStyleSheet("QGroupBox {background-color: white; border: 4px inset grey;}")

		self.go_btn = QPushButton('Go',self)
		self.nogo_btn = QPushButton('No Go',self)

		self.pushLayout.addWidget(self.go_btn,9,16,1,3); 
		self.go_btn.setStyleSheet("background-color: green")

		self.pushLayout.addWidget(self.nogo_btn,9,19,1,3); 
		self.nogo_btn.setStyleSheet("background-color: red")

		self.go_btn.clicked.connect(self.operator_toast)
		self.nogo_btn.clicked.connect(self.operator_toast)
		self.layout.addWidget(goGroup,9,16,2,14)

		#Hazard slider --------------------------------
		sliderLayout = QGridLayout(); 
		self.beliefOpacitySlider = QSlider(Qt.Horizontal); 
		self.beliefOpacitySlider.setSliderPosition(30)
		self.beliefOpacitySlider.setTickPosition(QSlider.TicksBelow)
		self.beliefOpacitySlider.setTickInterval(10); 

		sliderLayout.addWidget(self.beliefOpacitySlider,0,0); 
		belLabel = QLabel("Hazards"); 
		belLabel.setAlignment(Qt.AlignLeft); 
		sliderLayout.addWidget(belLabel,0,1,1,2); 


		self.layout.addLayout(sliderLayout,9,1,1,6) 

		#---------------------------------------------
		self.time = QLineEdit(); 

		timer = QTime()
		timer.start()
		self.time.setText(str(timer.elapsed()))
		self.time.setStyleSheet("background-color: white; border: 4px inset grey;")


		self.layout.addWidget(self.time,5,20,1,3) 

		#----------------------------------------------
		self.stateLayout = QGridLayout();

		stateGroup = QGroupBox()
		stateGroup.setLayout(self.stateLayout)

		stateGroup.setStyleSheet("QGroupBox {background-color: white; border: 4px inset grey;}")

		self.fuel = QLabel()
		self.xP = QLabel()
		self.xQ = QLabel()
		self.goal = QLabel()

		self.fuel.setText('Fuel Remaining: 100%')
		self.xP.setText('Outcome Assessment: 0.1')
		self.xQ.setText('Solver Quality: 0.8')
		self.goal.setText('Current Goal: A')

		self.stateLayout.addWidget(self.fuel,11,1,1,1); 
		self.stateLayout.addWidget(self.xQ,12,1,1,1);
		self.stateLayout.addWidget(self.goal,12,4,1,1);
		self.stateLayout.addWidget(self.xP,11,4,1,1); 
		self.layout.addWidget(stateGroup,11,1,1,4)

	def operator_toast(self):
		toast = QInputDialog()
		self.val, okPressed = toast.getInt(self, "Confidence","Rate Your Confidence:", 1, 0, 5, 1)

	def make_connections(self): 
		#Handler for final sketches
		pass


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

