#! /usr/bin/python

# Copyright (c) 2015, Nihon Binary Co., Ltd.
# All rights reserved.
# 
# Command Line Parameter:
#	rosrun robotiq_c_rtu robotiq_c_rtu_node.py [device_name] [port]
# 
# Default parameter:
# 	device_name = gripper
# 	port = /dev/ttyUSB0
# 
# Subscribe topic: 
#	/robotiq/device_name/command/position
# 	/robotiq/device_name/command/speed
# 	/device_name/command/force
# 
# Publish topic:
#	/device_name/current/position
# 	/device_name/current/current
# 	/device_name/current/fault
# 	/device_name/current/busy
# 	/device_name/current/object
# 

# python library
import sys
import time

# ROS library
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Bool

# robotiq library
import robotiq_c_rtu_device

# parameter
port = "/dev/ttyUSB0"
device_name = "gripper"

def initGripper():
	global gripper
	global port
	print "Initializing Gripper..."
	gripper = robotiq_c_rtu_device.gripper()
	return gripper.openGripper(port)
	
def releaseGripper():
	global gripper
	print "Releasing Gripper..."
	gripper.closeGripper()
	gripper = None

def positionCallback(data):
	global gripper
	#print "command position: ", data.data
	#print "position thread: ", thread.get_ident()
	gripper.setPosition(data.data, True, False)
	#print "---"
	
def speedCallback(data):
	global gripper
	#print "command speed: ", data.data
	gripper.setSpeed(data.data, False, False)
	#print "---"
	
def forceCallback(data):
	global gripper
	#print "command force: ", data.data
	gripper.setForce(data.data, False, False)
	#print "---"
	
def setupSubscriber():
	print "Setting up the subscribers..."
	
	print "Subscribe topic: /robotiq/"+device_name+"/command/position"
	rospy.Subscriber("/robotiq/"+device_name+"/command/position", Int32, positionCallback)
	print "Subscribe topic: /robotiq/"+device_name+"/command/speed"
	rospy.Subscriber("/robotiq/"+device_name+"/command/speed", Int32, speedCallback)
	print "Subscribe topic: /robotiq/"+device_name+"/command/force"
	rospy.Subscriber("/robotiq/"+device_name+"/command/force", Int32, forceCallback)
	#rospy.spin()

def startPbulish():
	global gripper
	print "Start publishing."
	
	print "Publish topic: /robotiq/"+device_name+"/current/position"
	pub_position = rospy.Publisher("/robotiq/"+device_name+"/current/position", Int32, queue_size=10)
	print "Publish topic: /robotiq/"+device_name+"/current/current"
	pub_current = rospy.Publisher("/robotiq/"+device_name+"/current/current", Int32, queue_size=10)
	print "Publish topic: /robotiq/"+device_name+"/current/fault"
	pub_fault = rospy.Publisher("/robotiq/"+device_name+"/current/fault", Int32, queue_size=10)
	print "Publish topic: /robotiq/"+device_name+"/current/busy"
	pub_busy = rospy.Publisher("/robotiq/"+device_name+"/current/busy", Bool, queue_size=10)
	print "Publish topic: /robotiq/"+device_name+"/current/object"
	pub_object = rospy.Publisher("/robotiq/"+device_name+"/current/object", Int32, queue_size=10)
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
		#print "start publish"
		#print "main thread: ", thread.get_ident()
		data = gripper.getInputData()
		#print "position ", data[robotiq_c_rtu_device.IN_POSITION]
		pub_position.publish(data[robotiq_c_rtu_device.IN_POSITION])
		#print "current ", data[robotiq_c_rtu_device.IN_CURRENT]
		pub_current.publish(data[robotiq_c_rtu_device.IN_CURRENT])
		#print "fault ", data[robotiq_c_rtu_device.IN_FAULT]
		pub_fault.publish(data[robotiq_c_rtu_device.IN_FAULT])
		busy = not (data[robotiq_c_rtu_device.IN_STATUS] & robotiq_c_rtu_device.STATUS_BUSY_MASK) \
				or not (data[robotiq_c_rtu_device.IN_STATUS] & robotiq_c_rtu_device.STATUS_OBJECT_MASK == robotiq_c_rtu_device.STATUS_OBJECT_INMOTION)
				#print "busy ", busy
		pub_busy.publish(busy)
		pub_object.publish((data[robotiq_c_rtu_device.IN_STATUS] & robotiq_c_rtu_device.STATUS_OBJECT_MASK) >> 6)
		#print "end publish"
		rate.sleep()	


def main():
	global device_name, port
	print ""
	print "Robotiq Gripper C-Model(Modbus-RTU) ROS Interface Node"
	print "Copyright (c) 2015 Nihon Binary Co., Ltd."
	print ""
	print "Usage: " + sys.argv[0] + " [device_name] [port]"
	print "\tDefault values are:"
	print "\t\tdevice_name=gripper"
	print "\t\tport=/dev/ttyUSB0"
	print ""

	if len(sys.argv) >= 2:
		device_name = sys.argv[1]
	if len(sys.argv) >= 3:
		port = sys.argv[2]

	print ""
	print "Current Parameters: "
	print "\tDevice Name: ", device_name
	print "\tPort: ", port
	print ""
	
	if initGripper():
		try:
			rospy.init_node('robotiq_c_rtu', anonymous=True)
			setupSubscriber()
			startPbulish()
		except rospy.ROSInterruptException:
			pass
			
		print ""
		releaseGripper()
		
		print ""
		print "done."
		
	else:
		print "Failed to start gripper(port="+port+")"
			
if __name__ == '__main__':
	main()
			
			
			
