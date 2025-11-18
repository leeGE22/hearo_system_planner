import smach
import smach_ros

import rospy
import json
import re

from std_msgs.msg import String

import dao.db_interface as db


class WaitCommandS(smach_ros.MonitorState):
    def __init__(self, command_ip, system_manaer_ip):
        smach_ros.MonitorState.__init__(self, '/system_manager/command_msg', String, self.check_command,
                                        input_keys=[],
                                        output_keys=['command'])
        self.command_ip = command_ip             # command IP address for message validity check
        self.system_manaer_ip = system_manaer_ip # system_manager IP address for message validity check
        
    def check_command(self, user_data, res_msg):
        try:
            # JSON --> ROS
            ros_msg = json.loads(res_msg.data)
            
            # Validity check: IP address
            if ros_msg['ToUrl'] != self.system_manaer_ip or ros_msg['FromUrl'] != self.command_ip:
                return True

            user_data['command'] = ros_msg['Task']

            # print(f"[System Manager] get new command : {ros_msg['Task']}")
            
            return False
        
        except Exception as e:
            rospy.logerr(f"[System Manager] An unexpected error occurred: {str(e)}")
            return True

class PutCommandS(smach.State):
    def __init__(self, command_queue):
        smach.State.__init__(self, outcomes=['done', 'quit'],
                            input_keys=['command'],
                            output_keys=[])
        self.command_queue = command_queue
        self.pub = rospy.Publisher('/system_manager/context_msg', String, queue_size=1)
        self.fin_pub = rospy.Publisher('/system_manager/task_msg', String, queue_size=1)

    def execute(self, user_data):
        command = user_data.command.lower()
        if command == 'quit':
            self.pub.publish(json.dumps('quit'))
            print('[System Manager] exit the context reasoning')
            return 'quit'
        
        if command == '':
            return 'done'
        
        if 'finish' in command:
            situation = re.sub(r'(?i)finish', '', user_data.command).strip()
            
            # DB에서 현재 situation이 변수 situation과 동일한 로봇 id, ip 가져와서 리스트 만들기
            sit_robot_ids = db.selectIdRobotBySituation(situation)
            task = {}
            for i, id in enumerate(sit_robot_ids):
                initPos = db.selectPosInitByIdRobot(id)
                json_initPos = '{}'
                if isinstance(initPos, str):
                    json_initPos = initPos
                pos = json.loads(json_initPos)
                
                task[id] = {
                    'ip': db.selectIpByIdRobot(id),
                    'situation': situation,
                    'task': None,
                    'position': pos['position'],
                    'orientation': pos['orientation']
                }
            
            task_json = json.dumps(task)
            msg = String()
            msg.data = task_json
            self.fin_pub.publish(msg)
            
        else:
            self.command_queue.put(user_data.command)
            
        return 'done'


class CommandSubscriberSM(smach.StateMachine):
    def __init__(self, command_ip, system_manaer_ip, command_queue):
        smach.StateMachine.__init__(self, outcomes=['quit'],
                                    input_keys=[],
                                    output_keys=[])

        with self:
            self.add('WAIT_COMMAND', WaitCommandS(command_ip, system_manaer_ip),
                    transitions={'invalid': 'PUT_COMMAND',
                                 'valid': 'WAIT_COMMAND',
                                 'preempted':'WAIT_COMMAND'})
            self.add('PUT_COMMAND', PutCommandS(command_queue),
                    transitions={'done': 'WAIT_COMMAND',
                                 'quit': 'quit'})
