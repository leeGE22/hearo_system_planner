import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile

from functools import partial

from std_msgs.msg import String

from robot_bridge.dao import db_interface as db

class Listener(Node):
    def __init__(self):
        super().__init__('robot_bridge_node')

        qos_profile = QoSProfile(depth=10)

        all_robots = db.selectIdRobot()
        print('RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR')
        print(all_robots)
        print('RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR')
        
        # self._pubs = {}
        # for r in all_robots:
        #     self._pubs[r] =  self.create_publisher(String, '/robot' + str(r), qos_profile)

        self._subs = {}
        for r in all_robots:
            self._subs[r] =  self.create_subscription(String, '/system_manager/robot' + str(r), partial(self.callback, robot_id=r), qos_profile)
        self._subs  # prevent unused variable warning

    def callback(self, msg, robot_id):
        self.get_logger().info(f'[{robot_id}] says: {msg.data}')
        # self._pubs[robot_id].publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()