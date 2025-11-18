import smach
import smach_ros

import rospy
import json

from std_msgs.msg import String

import dao.db_interface as db


class WaitTaskS(smach_ros.MonitorState):
    def __init__(self):
        smach_ros.MonitorState.__init__(self, '/system_manager/task_msg', String, self.check_task,
                                        input_keys=[],
                                        output_keys=['task'])
        self.robot_list = []
        
    def check_task(self, user_data, res_msg):
        try:
            # JSON --> ROS
            ros_msg = json.loads(res_msg.data)
            
            if isinstance(ros_msg, str):
                if ros_msg.lower() == 'quit':
                    user_data['task'] = 'quit'
                    return False
            
            task = {}
            for k, v in ros_msg.items():
                try:
                    task[int(k)] = v
                    
                    if v['task'] == None:
                        self.robot_list = db.selectIdRobotByState('work')
                    else:
                        self.robot_list = db.selectIdRobotByState('idle') + db.selectIdRobotBySituationNState('Finish', 'move')
                        
                except ValueError:
                    rospy.logerr(f"[System Manager] Wrong Robot Id Input")
                    return True
                
            for key in task:
                if key not in self.robot_list or any(k not in task[key] for k in ['ip', 'situation', 'task', 'position', 'orientation']):
                    return True
            
            user_data['task'] = task

            # print(f"[System Manager] get new task : {task}")
            
            return False
        
        except Exception as e:
            rospy.logerr(f"[System Manager] An unexpected error occurred: {str(e)}")
            return True

class PutTaskS(smach.State):
    def __init__(self, task_queue):
        smach.State.__init__(self, outcomes=['done', 'quit'],
                            input_keys=['task'],
                            output_keys=[])
        self.task_queue = task_queue

    def execute(self, user_data):
        task = user_data.task
        if task == 'quit':
            print('[System Manager] exit the task allocation')
            return 'quit'
        
        if task == '':
            return 'done'
        
        self.task_queue.put(task)

        return 'done'


class TaskSubscriberSM(smach.StateMachine):
    def __init__(self, task_queue):
        smach.StateMachine.__init__(self, outcomes=['quit'],
                                    input_keys=[],
                                    output_keys=[])

        with self:
            self.add('WAIT_TASK', WaitTaskS(),
                    transitions={'invalid': 'PUT_TASK',
                                 'valid': 'WAIT_TASK',
                                 'preempted':'WAIT_TASK'})
            self.add('PUT_TASK', PutTaskS(task_queue),
                    transitions={'done': 'WAIT_TASK',
                                 'quit': 'quit'})