import smach

import rospy
import queue
import json

from std_msgs.msg import String

from algorithm.task_allocation import Task_Allocation


class GetContextS(smach.State):
    def __init__(self, context_queue):
        smach.State.__init__(self, outcomes=['done', 'none'],
                             input_keys=[],
                             output_keys=['context', 'trial_num'])
        self.context_queue = context_queue
        
    def execute(self, user_data):        
        try:
            user_data.context = self.context_queue.get_nowait()
            user_data.trial_num = 0
            return 'done'
        except queue.Empty:
            return 'none'
        
class TaskAllocationS(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail', 'rejected'],
                             input_keys=['context', 'trial_num'],
                             output_keys=['context', 'trial_num', 'task'])

    def execute(self, user_data):
        task = Task_Allocation(user_data.context)

        if not task:
            if user_data.trial_num >= 5:
                return 'fail'
            else:
                user_data.trial_num += 1
                return 'rejected'
            
        user_data['task'] = task
        return 'done'

class PubTaskS(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'],
                             input_keys=['task'],
                             output_keys=[])
                            
        self.pub = rospy.Publisher('/system_manager/task_msg', String, queue_size=1)

    def execute(self, user_data):
        if user_data.task == '':
            return 'done'

        task_json = json.dumps(user_data.task)
        msg = String()
        msg.data = task_json
        self.pub.publish(msg)

        # print(f'[System Manager] pub task : {user_data.task}')

        return 'done'


class TaskAllocationSM(smach.StateMachine):
    def __init__(self, context_queue):
        smach.StateMachine.__init__(self, outcomes=['quit'],
                                    input_keys=[],
                                    output_keys=[])

        with self:
            self.add('GET_CONTEXT', GetContextS(context_queue),
                    transitions={'done':'TASK_ALLOCATION',
                                 'none': 'GET_CONTEXT'})
            self.add('TASK_ALLOCATION', TaskAllocationS(),
                    transitions={'done': 'PUB_TASK',
                                 'fail':'TASK_ALLOCATION',
                                 'rejected':'GET_CONTEXT'})
            self.add('PUB_TASK', PubTaskS(),
                    transitions={'done': 'GET_CONTEXT'})