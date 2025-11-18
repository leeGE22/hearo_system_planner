#!/usr/bin/env python3

import rospy
import json
from std_msgs.msg import String

PUB = rospy.Publisher("/system_manager/command_msg", String, queue_size=1)

def create_ros_msg(task):
    ros_msg = {
        "FromUrl": "161.122.114.23",	# Liku IP
        "ToUrl": "192.168.10.110",		# Server IP
        "Task": task,					# Task
    }
    
    json_msg = json.dumps(ros_msg, indent=4)
    PUB.publish(json_msg)

if __name__ == "__main__":
	rospy.init_node('command_publisher')
	
	while not rospy.is_shutdown():
		ros_msg = ""
		print()
		print('------------------------')
		print("Study alone              : 1")
		print("Study with friend        : 2")
		print("Talk with friend         : 3")
		print("Finish Study alone       : 11")
		print("Finish Study with friend : 12")
		print("Finish Talk with friend  : 13")
		print("quit                     : 4")

		cmd = input("Enter command number: ")
		
		if cmd == "1":
			ros_msg = create_ros_msg("Study alone")
		elif cmd == "2":
			ros_msg = create_ros_msg("Study with friend")
		elif cmd == "3":
			ros_msg = create_ros_msg("Talk with friend")
		elif cmd == "11":
			ros_msg = create_ros_msg("Finish Study alone")
		elif cmd == "12":
			ros_msg = create_ros_msg("Finish Study with friend")
		elif cmd == "13":
			ros_msg = create_ros_msg("Finish Talk with friend")
		elif cmd == "44":
			ros_msg = create_ros_msg("Finish Idle")
		else:
			ros_msg = create_ros_msg("quit")
			rospy.signal_shutdown("quit")
			break
  
	rospy.spin()
