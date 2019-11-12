import rclpy
from rclpy.node import Node
from ax_ros_interfaces.srv import RunAxTrial
import numpy as np
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from .gazebo_interface import GazeboInterface

class NavAxTrialRunner(Node):

    def __init__(self):
        super().__init__('ax_test_server')
        self.gb_interface = GazeboInterface()
        self.srv = self.create_service(RunAxTrial, 'nav2_ax_trial', self.ax_trial_callback)

    def ax_trial_callback(self, request, response):
        self.gb_interface.reset_gazebo_world()
        response.result = [42.0]
        return response


def main(args=None):
    rclpy.init(args=args)

    nav2_trial_runner = NavAxTrialRunner()
    rclpy.spin(nav2_trial_runner)  
    rclpy.shutdown()

if __name__ == '__main__':
    main()
