import smach

import rospy
import queue
import json

from std_msgs.msg import String

from algorithm.context_reasoning import Context_Reasoning


class GetCommandS(smach.State):
    def __init__(self, command_queue):
        smach.State.__init__(self, outcomes=['done', 'none'],
                             input_keys=[],
                             output_keys=['command', 'trial_num'])
        self.command_queue = command_queue
        
    def execute(self, user_data):        
        try:
            user_data.command = self.command_queue.get_nowait()
            user_data.trial_num = 0
            return 'done'
        except queue.Empty:
            return 'none'
        
class ContextReasoningS(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail', 'rejected'],
                             input_keys=['command', 'trial_num'],
                             output_keys=['command', 'trial_num', 'context'])

    def execute(self, user_data):
        context = Context_Reasoning(user_data.command)

        if not context:
            if user_data.trial_num >= 5:
                return 'fail'
            else:
                user_data.trial_num += 1
                return 'rejected'
            
        user_data['context'] = context
        return 'done'

class PubContextS(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'],
                             input_keys=['context'],
                             output_keys=[])
                            
        self.pub = rospy.Publisher('/system_manager/context_msg', String, queue_size=1)

    def execute(self, user_data):
        if user_data.context == '':
            return 'done'

        context_json = json.dumps(user_data.context)
        msg = String()
        msg.data = context_json
        self.pub.publish(msg)

        # print(f'[System Manager] pub context : {user_data.context}')

        return 'done'


class ContextReasoningSM(smach.StateMachine):
    def __init__(self, command_queue):
        smach.StateMachine.__init__(self, outcomes=['quit'],
                                    input_keys=[],
                                    output_keys=[])

        with self:
            self.add('GET_COMMAND', GetCommandS(command_queue),
                    transitions={'done':'CONTEXT_REASONING',
                                 'none': 'GET_COMMAND'})
            self.add('CONTEXT_REASONING', ContextReasoningS(),
                    transitions={'done': 'PUB_CONTEXT',
                                 'fail':'CONTEXT_REASONING',
                                 'rejected':'GET_COMMAND'})
            self.add('PUB_CONTEXT', PubContextS(),
                    transitions={'done': 'GET_COMMAND'})