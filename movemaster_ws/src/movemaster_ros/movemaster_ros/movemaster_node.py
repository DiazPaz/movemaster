#! /usr/bin/env python3

import rclpy
import sys

from rclpy.node import Node

sys.path.insert(0,'/home/movemaster/movemaster/main')

from teach_pendant_backend import TeachPendantBackend

class Movemaster_Node(Node):
    def __init__(self):
        super().__init__('movemaster_node')
        self.get_logger().info("Movemaster Node Initiated")

        self.axis=TeachPendantBackend(channel='can0',device_id=1)
        self.get_logger().info('TeachPendantBackend Initiated')

def main(args=None):
    rclpy.init(args=args)

    node=Movemaster_Node()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()



if __name__=='__main__':
    main()
