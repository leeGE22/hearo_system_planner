import smach

import rospy
import queue
import json

from std_msgs.msg import String

import dao.db_interface as db


class RobotCommunicatorS(smach.State):
    def __init__(self, robot_queues):
        smach.State.__init__(self, outcomes=['quit'],
                             input_keys=[],
                             output_keys=[])
        self.robot_queues = robot_queues
        all_robots = db.selectIdRobot()
        self.publishers = {}
        for r in all_robots:
            self.publishers[r] =  rospy.Publisher('/system_manager/robot' + str(r), String, queue_size=1)
        
    def execute(self, user_data):
        try:
            while True:
                rospy.sleep(1)
                
                for key in list(self.robot_queues):
                    for p_key in list(self.publishers):
                        if key == p_key:
                            try:
                                robot_q = self.robot_queues[key].get_nowait()
                                
                                robot_q_json = json.dumps(robot_q)
                                msg = String()
                                msg.data = robot_q_json
                                self.publishers[p_key].publish(msg)
                                
                                if self.robot_queues[key].empty():
                                    del self.robot_queues[key]
                            except queue.Empty:
                                pass
        except:
            return 'quit'