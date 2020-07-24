#!/usr/bin/env python2.7

import numpy as np
from PyQt5 import QtGui, QtCore, QtWidgets
from PyQt5.QtWidgets import *; 
from PyQt5.QtGui import *;
from PyQt5.QtCore import *;

def planeFlushPaint(planeWidget,col = None,pen=None):
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

def planeAddPaint(planeWidget,value,x,y,col,pen=None):
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

def makeTransparentPlane(width, height):
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



