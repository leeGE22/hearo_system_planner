#!/usr/bin/env python3

import argparse
import logging

import smach
import smach_ros

import rospy
import queue

from state_machines.task_scheduling.task_subscriber_sm import TaskSubscriberSM
from state_machines.task_scheduling.task_distributor_sm import TaskDistributorSM
from state_machines.task_scheduling.robot_communicator_s import RobotCommunicatorS


class Filter(logging.Filter):
    def filter(self, record):
        return 'State machine' not in record.msg

class TaskScheduling:
    def __init__(self, system_manager_ip):
        # Initialize ROS
        rospy.init_node('task_scheduling_node', log_level=rospy.WARN)

        self.system_manager_ip = system_manager_ip

        # Construct the state machine
        self.task_scheduling_state_machine = smach.Concurrence(
            outcomes=['quit'],
            default_outcome='quit'
        )
        self.task_queue = queue.Queue()
        self.robot_queues = {}

        with self.task_scheduling_state_machine:
            smach.Concurrence.add('TASK_SUBSCRIBER', TaskSubscriberSM(task_queue=self.task_queue))
            smach.Concurrence.add('TASK_DISTRIBUTOR', TaskDistributorSM(system_manager_ip=self.system_manager_ip, task_queue=self.task_queue, robot_queues=self.robot_queues))
            smach.Concurrence.add('ROBOT_COMMUNICATOR', RobotCommunicatorS(robot_queues=self.robot_queues))
            

    def execute(self):
        outcome = self.task_scheduling_state_machine.execute()
        print(f'[System Manager] final state is {outcome}. Done.')


if __name__ == '__main__':
    # Parse arguments
    parser = argparse.ArgumentParser(description='Task Scheduling of System Manager')
    parser.add_argument('-sip', '--system_manager_ip',
                        type=str,
                        required=True,
                        help='System Manager IP')
    parser.add_argument('-l', '--log',
                        action='store_true',
                        default=False,
                        help='Enable the ROS SMACH log')
    parser.add_argument('-v', '--viewer',
                        action='store_true',
                        default=False,
                        help='Enable the ROS SMACH viewer')
    args, unknown = parser.parse_known_args()

    # Create a system manager
    task_scheduling = TaskScheduling(system_manager_ip=args.system_manager_ip)
    print('[System Manager] Task Scheduling is ready.')

    # Disable the ROS SMACH logs during state initializations, transitions, etc.
    if not args.log:
        logging.getLogger('rosout').addFilter(Filter())

    # Enable the ROS SMACH viewer
    sis = None
    if args.viewer:
        sis = smach_ros.IntrospectionServer('task_scheduling', task_scheduling.task_scheduling_state_machine, 'task_scheduling')
        sis.start()

    # Run the system manager
    task_scheduling.execute()
    rospy.spin()

    if sis is not None:
        sis.stop()