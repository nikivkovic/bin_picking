#!/usr/bin/env python3

import rospy
import time
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray, Int64
from getkey import getkey, keys

# Callback function
def processing_callback(req):
	global binPickingEnd, processingFinished, position
	
	if not binPickingEnd:
		rospy.loginfo('Processing finished. ')
	
	# Consider processing to be finished. Handle the flow of the script accordingly (robot control or script end).
	processingFinished = 1
	position = req.data
	
	
# Control logic function
def control():
	global pub, binPickingEnd, processingFinished, position
	
	rospy.loginfo("Press y to continue bin picking. Press any other key to stop the algo. ")
	key = getkey().lower()
	
	if (key == 'y'):
		# Continue bin picking
		rospy.loginfo('Processing started. Waiting for GPD candidate.')
		binPickingEnd = 0
		
	else:
		# Bin picking finished
		rospy.loginfo('Bin picking finished. Closing main script. ')
		binPickingEnd = 1
			
	# Publish data to processing script
	processingFinished = 0
	pub.publish(binPickingEnd)
	
	# Wait until processing script returns a response
	while not processingFinished:
		time.sleep(0.1)
		pass
	
	if not binPickingEnd: 
		# Control the robot based on position recieved by processing script
		rospy.loginfo('Controling the robot. GPD position: ' + str(position[0]) + ', ' + str(position[1]) + ', ' + str(position[2]) + '. ')
		time.sleep(5)
		
		
# Process function
def process():
	global pub, binPickingEnd
	
	rospy.init_node('main', anonymous = False)
	pub = rospy.Publisher('main_processing', Int64, queue_size = 10)
	sub = rospy.Subscriber('processing_main', Float64MultiArray, processing_callback)
	binPickingEnd = 0
	
	rospy.loginfo('Main idle. Waiting for user input. ')
	
	while ((not rospy.is_shutdown()) and (not binPickingEnd)):
		control()
		
			
if __name__ == '__main__':
	try:
		process()
		
	except rospy.ROSInterruptException:
		pass 
