import smach

import queue


class GetTaskS(smach.State):
    def __init__(self, task_queue):
        smach.State.__init__(self, outcomes=['done', 'none'],
                             input_keys=[],
                             output_keys=['task'])
        self.task_queue = task_queue
        
    def execute(self, user_data):        
        try:
            user_data.task = self.task_queue.get_nowait()
            return 'done'
        except queue.Empty:
            return 'none'
        
class TaskDistributingS(smach.State):
    def __init__(self, system_manager_ip, robot_queues):
        smach.State.__init__(self, outcomes=['done'],
                             input_keys=['task'],
                             output_keys=['task'])
        self.system_manager_ip = system_manager_ip
        self.robot_queues = robot_queues

    def create_ros_msg(self, ip, situation, task, position, orientation):
        return {
            "FromUrl": self.system_manager_ip,	# Server IP
            "ToUrl": ip,		                # Robot IP
            "Situation": situation,             # situation
            "Task": task,					    # Task
            "Position": position,				# position
            "Orientation": orientation,			# orientation
        }

    def execute(self, user_data):
        # user_data.task = {1: {'ip': '192.168.10.1', 'situation': 'Study alone', 'task': 'big table', 'position': {'x': 1.0, 'y': 2.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.7071, 'w': 0.7071}},
        #                   4: {'ip': '192.168.10.4', 'situation': 'Study alone', 'task': 'chair', 'position': {'x': 2.0, 'y': 2.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.7071, 'w': 0.7071}}}
        for key in user_data.task:
            if key not in self.robot_queues:
                self.robot_queues[key] = queue.Queue()
                
            self.robot_queues[key].put(self.create_ros_msg(user_data.task[key]['ip'], user_data.task[key]['situation'], user_data.task[key]['task'], user_data.task[key]['position'], user_data.task[key]['orientation']))

        return 'done'


class TaskDistributorSM(smach.StateMachine):
    def __init__(self, system_manager_ip, task_queue, robot_queues):
        smach.StateMachine.__init__(self, outcomes=['quit'],
                                    input_keys=[],
                                    output_keys=[])

        with self:
            self.add('GET_TASK', GetTaskS(task_queue),
                    transitions={'done':'TASK_DISTRIBUTING',
                                 'none': 'GET_TASK'})
            self.add('TASK_DISTRIBUTING', TaskDistributingS(system_manager_ip, robot_queues),
                    transitions={'done': 'GET_TASK'})