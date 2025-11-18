#!/usr/bin/env python3

import argparse
import logging

import smach
import smach_ros

import rospy
import queue

from state_machines.context_reasoning.command_subscriber_sm import CommandSubscriberSM
from state_machines.context_reasoning.context_reasoning_sm import ContextReasoningSM


class Filter(logging.Filter):
    def filter(self, record):
        return 'State machine' not in record.msg

class ContextReasoning:
    def __init__(self, command_ip, system_manager_ip):
        # Initialize ROS
        rospy.init_node('context_reasoning_node', log_level=rospy.WARN)
        
        self.command_ip = command_ip
        self.system_manaer_ip = system_manager_ip

        # Construct the state machine
        self.context_reasoning_state_machine = smach.Concurrence(
            outcomes=['quit'],
            default_outcome='quit'
        )
        self.command_queue = queue.Queue()

        with self.context_reasoning_state_machine:
            smach.Concurrence.add('COMMAND_SUBSCRIBER', CommandSubscriberSM(command_ip=self.command_ip, system_manaer_ip=self.system_manaer_ip, command_queue=self.command_queue))
            smach.Concurrence.add('CONTEXT_REASONING', ContextReasoningSM(command_queue=self.command_queue))

    def execute(self):
        outcome = self.context_reasoning_state_machine.execute()
        print(f'[System Manager] final state is {outcome}. Done.')


if __name__ == '__main__':
    # Parse arguments
    parser = argparse.ArgumentParser(description='Context Reasoning of System Manager')
    parser.add_argument('-cip', '--command_ip',
                        type=str,
                        required=True,
                        help='Command IP')
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
    context_reasoning = ContextReasoning(command_ip=args.command_ip, system_manager_ip=args.system_manager_ip)
    print('[System Manager] Context Reasoning is ready.')

    # Disable the ROS SMACH logs during state initializations, transitions, etc.
    if not args.log:
        logging.getLogger('rosout').addFilter(Filter())

    # Enable the ROS SMACH viewer
    sis = None
    if args.viewer:
        sis = smach_ros.IntrospectionServer('context_reasoning', context_reasoning.context_reasoning_state_machine, 'context_reasoning')
        sis.start()

    # Run the system manager
    context_reasoning.execute()
    rospy.spin()

    if sis is not None:
        sis.stop()
        