import rclpy
from rclpy.node import Node
from ax_ros_interfaces.srv import RunAxTrial
import numpy as np
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor

class AxTrialServer(Node):

    def __init__(self):
        super().__init__('ax_test_server')
        self.srv = self.create_service(RunAxTrial, 'simple_ax_trial', self.simple_ax_trial_callback)
        self.declare_parameter('x', 2.0)

    def simple_ax_trial_callback(self, request, response):
        x = self.get_parameter('x').value

        self.get_logger().info('Incoming Ax trial request with test parameter x: %f' % (x))
        response.result = [np.square(x-5)]
        self.get_logger().info('Sending Ax trial result: %f\n' % (response.result[0]))
        return response


def main(args=None):
    rclpy.init(args=args)

    ax_trial_server = AxTrialServer()
    rclpy.spin(ax_trial_server)  
    rclpy.shutdown()

if __name__ == '__main__':
    main()
