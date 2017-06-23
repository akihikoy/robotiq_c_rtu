# robotiq_c_rtu

## How to install

Clone this package:  
$ cd ~/ros_ws/src  
$ git clone https://github.com/NihonBinary/robotiq_c_rtu.git  
  
Install dependency:  
$ sudo apt-get install ros-indigo-soem, python-setuptools  
$ sudo easy_install pip  
$ sudo pip install pymodbus  
  
Make:  
$ cd ~/ros_ws  
$ catkin_make  
  
Put udev rules for serial USB adapter:  
$ sudo -s  
$ echo 'KERNEL=="ttyUSB*", MODE="0666"' > /etc/udev/rules.d/70-usbserial.rules  
$ exit  
  
## How to use this package

### Run robotiq_c_rtu_node ROS Node:  
$ rosrun robotiq_c_rtu robotiq_c_rtu_node.py [device_name] [serial_port] [rate]

Parameters: 

*device_name*
Name of the device which used as toppic's namespace which subscribed/published by this node. Each instance of this node for each grippers must have identical name. Default value of device_name is "gripper".

*serial_port*
Path to the device file of the serial port which connected to the gripper. Default value of serial_port is "/dev/ttyUSB0".

*rate*
Control rate in Hz.  Default value is 10.0 Hz.

### Control gripper by topics:
$ rostopic pub -1 /robotiq/*device_name*/command/speed std_msgs/Int32 -- 0  
$ rostopic pub -1 /robotiq/*device_name*/command/force std_msgs/Int32 -- 0  
$ rostopic pub -1 /robotiq/*device_name*/command/position std_msgs/Int32 -- 255  

## Test Robotiq Gripper:
$ robotiq_c_rtu_device.py [serial_port]

* The ROS node must be exited before running this test program.

Parameters: 

*serial_port*  
Path to the device file of the serial port which connected to the gripper. Default value of serial_port is "/dev/ttyUSB0".
