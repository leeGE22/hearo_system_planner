#!/usr/bin/env python3

import argparse
import logging

import smach
import smach_ros

import rospy
import queue

from state_machines.task_allocation.context_subscriber_sm import ContextSubscriberSM
from state_machines.task_allocation.task_allocation_sm import TaskAllocationSM


class Filter(logging.Filter):
    def filter(self, record):
        return 'State machine' not in record.msg

class TaskAllocation:
    def __init__(self):
        # Initialize ROS
        rospy.init_node('task_allocation_node', log_level=rospy.WARN)

        # Construct the state machine
        self.task_allocation_state_machine = smach.Concurrence(
            outcomes=['quit'],
            default_outcome='quit'
        )
        self.context_queue = queue.Queue()

        with self.task_allocation_state_machine:
            smach.Concurrence.add('CONTEXT_SUBSCRIBER', ContextSubscriberSM(context_queue=self.context_queue))
            smach.Concurrence.add('TASK_ALLOCATION', TaskAllocationSM(context_queue=self.context_queue))

    def execute(self):
        outcome = self.task_allocation_state_machine.execute()
        print(f'[System Manager] final state is {outcome}. Done.')


if __name__ == '__main__':
    # Parse arguments
    parser = argparse.ArgumentParser(description='Task Allocation of System Manager')
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
    task_allocation = TaskAllocation()
    print('[System Manager] Task Allocation is ready.')

    # Disable the ROS SMACH logs during state initializations, transitions, etc.
    if not args.log:
        logging.getLogger('rosout').addFilter(Filter())

    # Enable the ROS SMACH viewer
    sis = None
    if args.viewer:
        sis = smach_ros.IntrospectionServer('task_allocation', task_allocation.task_allocation_state_machine, 'task_allocation')
        sis.start()

    # Run the system manager
    task_allocation.execute()
    rospy.spin()

    if sis is not None:
        sis.stop()