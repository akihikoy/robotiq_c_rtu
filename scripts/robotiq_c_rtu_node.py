#! /usr/bin/python

# Copyright (c) 2015, Nihon Binary Co., Ltd.
# All rights reserved.
# 
# Robotiq C Model (Modbus-RTU) ROS Driver Node
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
# 	/robotiq/device_name/command/force
# 
# Publish topic:
#	/robotiq/device_name/current/position
# 	/robotiq/device_name/current/current
# 	/robotiq/device_name/current/fault
# 	/robotiq/device_name/current/busy
# 	/robotiq/device_name/current/object
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
rate = 10.0

def initGripper():
	global gripper
	global port
	global rate
	print "Initializing Gripper..."
	gripper = robotiq_c_rtu_device.gripper()
	return gripper.openGripper(port, rate)
	
def releaseGripper():
	global gripper
	print "Releasing Gripper..."
	gripper.closeGripper()
	gripper = None

def positionCallback(data):
	global gripper
	#print "position thread: ", thread.get_ident()
	#print 'command position',data.data
	tm0= time.time()
	#gripper.setPosition(data.data, True, False)
	gripper.setPosition(data.data, False, False)
	#print 'debug22',time.time()-tm0
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
	global rate
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
	rate_adjuster = rospy.Rate(rate)
	
	while not rospy.is_shutdown():
		tm0 = rospy.Time.now()
		#print "start publish", tm0
		#print "main thread: ", thread.get_ident()
		data = gripper.getInputData()
		if len(data)>0:
		  #tm1 = rospy.Time.now(); print (tm1-tm0).to_sec(); tm0 = tm1
		  #print "position ", data[robotiq_c_rtu_device.IN_POSITION]
		  pub_position.publish(data[robotiq_c_rtu_device.IN_POSITION])
		  #  print "current ", data[robotiq_c_rtu_device.IN_CURRENT]
		  pub_current.publish(data[robotiq_c_rtu_device.IN_CURRENT])
		  #print "fault ", data[robotiq_c_rtu_device.IN_FAULT]
		  pub_fault.publish(data[robotiq_c_rtu_device.IN_FAULT])
		  busy = not (data[robotiq_c_rtu_device.IN_STATUS] & robotiq_c_rtu_device.STATUS_BUSY_MASK) \
		  		or not (data[robotiq_c_rtu_device.IN_STATUS] & robotiq_c_rtu_device.STATUS_OBJECT_MASK == robotiq_c_rtu_device.STATUS_OBJECT_INMOTION)
		  		#print "busy ", busy
		  pub_busy.publish(busy)
		  pub_object.publish((data[robotiq_c_rtu_device.IN_STATUS] & robotiq_c_rtu_device.STATUS_OBJECT_MASK) >> 6)
		#print "end publish"
		gripper.sendOutputCommand(sync=False)
		rate_adjuster.sleep()


def main():
	global device_name, port, rate
	print ""
	print "Robotiq Gripper C-Model(Modbus-RTU) ROS Interface Node"
	print "Copyright (c) 2015 Nihon Binary Co., Ltd."
	print ""
	print "Usage: " + sys.argv[0] + " [device_name] [port] [rate]"
	print "\tDefault values are:"
	print "\t\tdevice_name=gripper"
	print "\t\tport=/dev/ttyUSB0"
	print "\t\rate=10.0"
	print ""

	if len(sys.argv) >= 2:
		device_name = sys.argv[1]
	if len(sys.argv) >= 3:
		port = sys.argv[2]
	if len(sys.argv)>3:  rate = float(sys.argv[3])

	print ""
	print "Current Parameters: "
	print "\tDevice Name: ", device_name
	print "\tPort: ", port
	print "\tRate: ", rate
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
			
			
			
