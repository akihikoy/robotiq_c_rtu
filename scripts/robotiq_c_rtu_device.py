#! /usr/bin/python

# Copyright (c) 2015, Nihon Binary Co., Ltd.
# All rights reserved.
# 
# Robotiq C Model (Modbus-RTU) Device management class
# 

import sys
import time
import thread
import comModbusRtu

CMD_SLEEP_TIME = 0.1

OUT_SIZE=6
IN_SIZE=6

# Index of output register
OUT_ACTION = 0
OUT_POSITION = 3
OUT_SPEED = 4
OUT_FORCE = 5

# Bit flag for ACTION REQUEST
ACTION_ACTIVE = 0x01
ACTION_GOTOREQPOS = 0x08
ACTION_AUTORELEASE = 0x10

# Index of input register
IN_STATUS = 0
IN_FAULT = 2
IN_POSITION = 4
IN_CURRENT = 5

# Data and bitmask of STATUS
STATUS_ACTIVE_MASK = 0x01
STATUS_BUSY_MASK = 0x08
STATUS_GSTAT_MASK = 0x30
STATUS_GSTAT_RESET = 0x00
STATUS_GSTAT_ACTIVATIONINPROGRESS = 0x20
STATUS_GSTAT_ACTIVATIONISCOMPLETED = 0x30
STATUS_OBJECT_MASK = 0xC0
STATUS_OBJECT_INMOTION = 0x00
STATUS_OBJECT_DETECTOPENING = 0x80
STATUS_OBJECT_DETECTCLOSING = 0x40
STATUS_OBJECT_NOTDETECTED = 0xC0


class gripper:

	_out = [0, 0, 0, 0, 255, 255]
	_in = [0, 0, 0, 0, 0, 0]
	

	def __init__(self):
		self.com = None
		self._lock = thread.allocate_lock()
	
	def openGripper(self, port):
		if self.com == None:
			self.com = comModbusRtu.communication()
			if self.com.connectToDevice(port):
				# successfully connected
				print "Gripper opened at port " + port
				self._initGripper()	
				return True
			else:
				print "Error: Connection to " + port + " is failed."
			self.com = None
		else:
			print "Error: Gripper is already opened."
		return False
		
	def closeGripper(self):
		if self.com != None:
			self.com.disconnectFromDevice()
			self.com = None
		else:
			print "Error: Gripper is not initialized"
	
	def _initGripper(self):
		# De-activation
		print "Resetting gripper"
		self._out[OUT_ACTION] &= ~ACTION_ACTIVE
		self.sendOutputCommand(False)
		
		# Activation
		print "Activating gripper"
		self._out[OUT_ACTION] |= ACTION_ACTIVE
		self.sendOutputCommand(False)
		self._waitForActivation()
		
		# Set move to current pos
		print "Enabling gripper"
		self._out[OUT_ACTION] |= ACTION_GOTOREQPOS;
		self.sendOutputCommand()
		
		print "Gripper Initialized!"
		
		
	def _waitForActivation(self):
		while True:
			self._in = self.getInputData()
			#print "Input:", self._in
			#print "STATUS_GSTAT_ACTIVATIONISCOMPLETED", STATUS_GSTAT_ACTIVATIONISCOMPLETED
			#print "self._in[IN_STATUS] & STATUS_OBJECT_MASK", self._in[IN_STATUS] & STATUS_OBJECT_MASK
			if (self._in[IN_STATUS] & STATUS_GSTAT_MASK == STATUS_GSTAT_ACTIVATIONISCOMPLETED):
				break;
			time.sleep(CMD_SLEEP_TIME)
			
	def waitForBusy(self):
		while True:
			self._in = self.getInputData()
			#print "Input:", self._in
			#print "self._in[IN_STATUS] & STATUS_BUSY_MASK", self._in[IN_STATUS] & STATUS_BUSY_MASK
			#print "self._in[IN_STATUS] & STATUS_GSTAT_MASK", self._in[IN_STATUS] & STATUS_GSTAT_MASK
			#print "STATUS_GSTAT_ACTIVATIONISCOMPLETED", STATUS_GSTAT_ACTIVATIONISCOMPLETED
			#print "self._in[IN_STATUS] & STATUS_OBJECT_MASK", self._in[IN_STATUS] & STATUS_OBJECT_MASK
			#print "STATUS_OBJECT_INMOTION", STATUS_OBJECT_INMOTION
			#print "STATUS_OBJECT_DETECTOPENING", STATUS_OBJECT_DETECTOPENING
			#print "STATUS_OBJECT_DETECTCLOSING", STATUS_OBJECT_DETECTCLOSING
			#print "STATUS_OBJECT_NOTDETECTED", STATUS_OBJECT_NOTDETECTED
			if not (self._in[IN_STATUS] & STATUS_BUSY_MASK) \
				or not (self._in[IN_STATUS] & STATUS_OBJECT_MASK == STATUS_OBJECT_INMOTION):
				break;
			time.sleep(CMD_SLEEP_TIME)
	
	def getInputData(self):
		self._lock.acquire()
		data = self.com.getStatus(IN_SIZE);
		self._lock.release()
		return data

	def setPosition(self, pos, send=False, sync=False):
		#print "setPosition(): ", pos, send, sync
		self._out[OUT_POSITION] = pos
		if send:
			self.sendOutputCommand(sync)
			
	def setSpeed(self, speed, send=False, sync=False):
		#print "setSpeed(): ", speed, send, sync
		self._out[OUT_SPEED] = speed
		if send:
			self.sendOutputCommand(sync)

	def setForce(self, force, send=False, sync=False):
		#print "setForce(): ", force, send, sync
		self._out[OUT_FORCE] = force
		if send:
			self.sendOutputCommand(sync)

	def sendOutputCommand(self, sync=False):
		#print "sendOutputCommand(): ", self._out
		self._lock.acquire()
		self.com.sendCommand(self._out)
		self._lock.release()
		if CMD_SLEEP_TIME != 0:
			time.sleep(CMD_SLEEP_TIME)
		if sync:
			self.waitForBusy()


#com = comModbusRtu.communication()
#com.connectToDevice('/dev/ttyUSB0')


#com.sendCommand([0,0,255,0,100,0])
#time.sleep(1)

#com.sendCommand([1,0,255,0,100,0])
#time.sleep(1);


def main():
	print ""
	print "Robotiq Gripper C-Model(Modbus-RTU) Device Test"
	print "Copyright (c) 2015 Nihon Binary Co., Ltd."
	print ""
	print "Usage: " + sys.argv[0] + " [port]"
	print ""
	print "Default value:"
	print "\tport=/dev/ttyUSB0"
	print ""

	port = "/dev/ttyUSB0"
	if len(sys.argv) >= 2:
		port = sys.argv[1]

	c = gripper()
	if c.openGripper(port):

		c.setSpeed(0)
		c.setPosition(255, True, False)
		time.sleep(0.1)
		c.setSpeed(25, True)
		time.sleep(0.1)
		c.setSpeed(50, True)
		time.sleep(0.1)
		c.setSpeed(75, True)
		time.sleep(0.1)
		c.setSpeed(100, True)
		c.waitForBusy()

		c.setPosition(0, True, False)
		time.sleep(0.1)
		c.setSpeed(75, True)
		time.sleep(0.1)
		c.setSpeed(50, True)
		time.sleep(0.1)
		c.setSpeed(25, True)
		time.sleep(0.1)
		c.setSpeed(0, True)
		c.waitForBusy()
	else:
		print "Failed to open Gripper: "+port

if __name__ == '__main__':
	main()
			
			


