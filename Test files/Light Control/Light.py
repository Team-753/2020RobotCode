from networktables import NetworkTables
import logging
import sys
import time
import threading
import board
import neopixel

pixels = neopixel.Neopixel(board.D18)


cond = threading.Condition()
notified = False
def connectionListener(connected, info):
	print(info, '; Connected=%s' % connected)
	with cond:
		notified = True 
		cond.notify()

# To see messages from networktables, you must setup logging 
NetworkTables.initialize() 
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)


while True:
	if Networktables.getTable('SmartDashboard').getBoolean('Light',False):
		pixels.fill((0,255,0))
	else:
		pixels.fill((0,0,0))