#!/usr/bin/env python3

import rospy
import time
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray, Int64

# Callback function
def processing_callback(req):
	global binPickingEnd
	position = Float64MultiArray()
	position.data = []
	
	rospy.loginfo('Request acquired.')
	
	# Check if bin picking is not finished
	if (req.data == 0):
		# Processing logic
		rospy.loginfo('Processing active...')
		time.sleep(5)
		rospy.loginfo('Processing finished.')
		position.data = [1, 2, 3]
		
	if (req.data == 1):
		# Bin picking finished.
		rospy.loginfo('Bin picking finished. Closing processing script.')
		binPickingEnd = 1
	
	# Publish data to main script
	pub.publish(position)
	
	if not binPickingEnd:
		rospy.loginfo('Position sent. ')
		rospy.loginfo('Waiting for request. ')


# Process function
def process():
	global pub, binPickingEnd
	
	rospy.init_node('processing', anonymous = False)
	pub = rospy.Publisher('processing_main', Float64MultiArray, queue_size = 10)
	sub = rospy.Subscriber('main_processing', Int64, processing_callback)
	binPickingEnd = 0
	
	rospy.loginfo('Processing idle. Waiting for request.')
	
	while not binPickingEnd:
		pass
			
			
if __name__ == '__main__':
	try:
		process()
		
	except rospy.ROSInterruptException:
		pass 
