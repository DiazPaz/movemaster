#! /usr/bin/env python3

import rclpy
import sys

from rclpy.node import Node

from sensor_msgs.msg import JointState

sys.path.insert(0,'/home/movemaster/movemaster/main')

from teach_pendant_backend import TeachPendantBackend

class Movemaster_Node(Node):
    def __init__(self):
        super().__init__('movemaster_node')
        self.get_logger().info("Movemaster Node Initiated")

        self.axis=TeachPendantBackend(channel='can0',device_id=1)
        self.get_logger().info('TeachPendantBackend Initiated')

        self.axis.start()
        self.get_logger().info('CAN BUS: OK')

        # Start SparkMax

        self.get_logger().info('Starting SparkMax...')

        cmd=self.axis.initialize(status_period_ms=20)

        try:
            result=cmd.result(6.0)
            self.get_logger().info('SparkMax: OK (result= {result})')

        except Exception as e:
            self.get_logger().info(f'SparkMax did not initialize: {e}')
            self.axis.close()
            raise

        # ROS 2 PUBLISHER

        self.joint_state_publisher=self.create_publisher(JointState, '/joint_states', 10)
        
        self.telemetry_timer=self.create_timer(0.020,self.publish_telemetry)

        self.get_logger().info('Publisher "/joint_states" iniciado')

        def publish_telemetry(self):

            telemetry = self.axis.telemetry()

            # Si todavía no tenemos una posición válida,
            # no publicamos el mensaje.
            if telemetry.pv_rot is None:
                return

            msg = JointState()

            msg.header.stamp = self.get_clock().now().to_msg()

            # Nombre de nuestra primera articulación
            msg.name = ['joint_1']

            # Spark MAX -> ROS 2
            #
            # posición:
            # rotaciones -> radianes
            #
            # velocidad:
            # RPM -> rad/s

            position_rad = telemetry.pv_rot * 2.0 * 3.141592653589793

            velocity_rad_s = 0.0

            if telemetry.velocity_rpm is not None:
                velocity_rad_s = (
                    telemetry.velocity_rpm
                    * 2.0
                    * 3.141592653589793
                    / 60.0
                )

            msg.position = [position_rad]
            msg.velocity = [velocity_rad_s]

            self.joint_state_publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    node=Movemaster_Node()
    try:
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        pass
    
    finally:
        node.axis.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
