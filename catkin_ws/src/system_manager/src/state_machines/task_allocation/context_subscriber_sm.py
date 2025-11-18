import smach
import smach_ros

import rospy
import json
import re

from std_msgs.msg import String

import dao.db_interface as db


class WaitContextS(smach_ros.MonitorState):
    def __init__(self):
        smach_ros.MonitorState.__init__(self, '/system_manager/context_msg', String, self.check_context,
                                        input_keys=[],
                                        output_keys=['context'])
        
    def check_context(self, user_data, res_msg):
        try:
            # JSON --> ROS
            ros_msg = json.loads(res_msg.data)            
            
            if isinstance(ros_msg, str):
                if ros_msg.lower() == 'quit':
                    user_data['context'] = 'quit'
                    return False
            
            task_list = db.selectTaskName()
            
            situation = None
            required_fields = {'idx', 'situation', 'position', 'orientation'}
            for key in ros_msg:
                _key = re.sub(r'\d+', '', key)
                if _key not in task_list or not required_fields.issubset(ros_msg[key]):
                    return True
                
                if situation is None:
                    situation = ros_msg[key]['situation']
                elif ros_msg[key]['situation'] != situation:
                    return True

            user_data['context'] = ros_msg

            # print(f"[System Manager] get new context : {ros_msg}")
            
            return False
        
        except Exception as e:
            rospy.logerr(f"[System Manager] An unexpected error occurred: {str(e)}")
            return True

class PutContextS(smach.State):
    def __init__(self, context_queue):
        smach.State.__init__(self, outcomes=['done', 'quit'],
                            input_keys=['context'],
                            output_keys=[])
        self.context_queue = context_queue
        self.pub = rospy.Publisher('/system_manager/task_msg', String, queue_size=1)

    def execute(self, user_data):
        context = user_data.context
        if context == 'quit':
            self.pub.publish(json.dumps('quit'))
            print('[System Manager] exit the task allocation')
            return 'quit'
        
        if context == '':
            return 'done'
        
        self.context_queue.put(context)

        return 'done'


class ContextSubscriberSM(smach.StateMachine):
    def __init__(self, context_queue):
        smach.StateMachine.__init__(self, outcomes=['quit'],
                                    input_keys=[],
                                    output_keys=[])

        with self:
            self.add('WAIT_CONTEXT', WaitContextS(),
                    transitions={'invalid': 'PUT_CONTEXT',
                                 'valid': 'WAIT_CONTEXT',
                                 'preempted':'WAIT_CONTEXT'})
            self.add('PUT_CONTEXT', PutContextS(context_queue),
                    transitions={'done': 'WAIT_CONTEXT',
                                 'quit': 'quit'})